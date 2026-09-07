#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>


class AdmittancePreview : public rclcpp::Node
{
public:
  AdmittancePreview()
  : Node("admittance_preview")
  {
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("tool_frame", "tool0");

    // Conservative translational admittance parameters.
    declare_parameter<double>("virtual_mass", 20.0);
    declare_parameter<double>("damping", 120.0);

    // Damped least-squares Jacobian inversion.
    declare_parameter<double>("dls_lambda", 0.05);

    // Preview safety limits.
    declare_parameter<double>("maximum_force", 10.0);
    declare_parameter<double>("maximum_cartesian_speed", 0.020);
    declare_parameter<double>("maximum_joint_speed", 0.100);

    declare_parameter<double>("stale_timeout", 0.10);
    declare_parameter<double>("update_rate", 100.0);

    base_frame_ =
      get_parameter("base_frame").as_string();

    tool_frame_ =
      get_parameter("tool_frame").as_string();

    virtual_mass_ =
      get_parameter("virtual_mass").as_double();

    damping_ =
      get_parameter("damping").as_double();

    dls_lambda_ =
      get_parameter("dls_lambda").as_double();

    maximum_force_ =
      get_parameter("maximum_force").as_double();

    maximum_cartesian_speed_ =
      get_parameter("maximum_cartesian_speed").as_double();

    maximum_joint_speed_ =
      get_parameter("maximum_joint_speed").as_double();

    stale_timeout_ =
      get_parameter("stale_timeout").as_double();

    update_rate_ =
      get_parameter("update_rate").as_double();

    validate_parameters();
    initialize_kinematics();

    wrench_subscription_ =
      create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/admittance/processed_wrench",
        100,
        std::bind(
          &AdmittancePreview::wrench_callback,
          this,
          std::placeholders::_1));

    processing_ready_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
        "/admittance/ready",
        10,
        std::bind(
          &AdmittancePreview::processing_ready_callback,
          this,
          std::placeholders::_1));

    joint_subscription_ =
      create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        100,
        std::bind(
          &AdmittancePreview::joint_callback,
          this,
          std::placeholders::_1));

    joint_velocity_preview_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
        "/admittance/joint_velocity_preview",
        10);

    cartesian_velocity_preview_publisher_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(
        "/admittance/cartesian_velocity_preview",
        10);

    ready_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
        "/admittance/preview_ready",
        10);

    const auto timer_period =
      std::chrono::duration<double>(1.0 / update_rate_);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        timer_period),
      std::bind(&AdmittancePreview::update, this));

    last_update_time_ = std::chrono::steady_clock::now();

    publish_zero_preview();
    publish_ready(false);

    RCLCPP_INFO(
      get_logger(),
      "Admittance Phase-2 computational preview started.");

    RCLCPP_INFO(
      get_logger(),
      "KDL chain: %s -> %s, joints=%u.",
      base_frame_.c_str(),
      tool_frame_.c_str(),
      chain_.getNrOfJoints());

    RCLCPP_INFO(
      get_logger(),
      "Virtual mass=%.3f kg, damping=%.3f N.s/m.",
      virtual_mass_,
      damping_);

    RCLCPP_INFO(
      get_logger(),
      "Limits: force=%.3f N, Cartesian speed=%.4f m/s, "
      "joint speed=%.4f rad/s.",
      maximum_force_,
      maximum_cartesian_speed_,
      maximum_joint_speed_);

    RCLCPP_WARN(
      get_logger(),
      "PREVIEW ONLY: no robot controller command topic is published.");

    RCLCPP_WARN(
      get_logger(),
      "The forward velocity controller must remain inactive.");
  }

private:
  void validate_parameters()
  {
    if (virtual_mass_ <= 0.0) {
      throw std::runtime_error(
              "virtual_mass must be greater than zero");
    }

    if (damping_ <= 0.0) {
      throw std::runtime_error(
              "damping must be greater than zero");
    }

    if (dls_lambda_ <= 0.0) {
      throw std::runtime_error(
              "dls_lambda must be greater than zero");
    }

    if (maximum_force_ <= 0.0) {
      throw std::runtime_error(
              "maximum_force must be greater than zero");
    }

    if (maximum_cartesian_speed_ <= 0.0) {
      throw std::runtime_error(
              "maximum_cartesian_speed must be greater than zero");
    }

    if (maximum_joint_speed_ <= 0.0) {
      throw std::runtime_error(
              "maximum_joint_speed must be greater than zero");
    }

    if (stale_timeout_ <= 0.0) {
      throw std::runtime_error(
              "stale_timeout must be greater than zero");
    }

    if (update_rate_ <= 0.0) {
      throw std::runtime_error(
              "update_rate must be greater than zero");
    }
  }

  void initialize_kinematics()
  {
    auto parameter_client =
      std::make_shared<rclcpp::SyncParametersClient>(
        this,
        "robot_state_publisher");

    RCLCPP_INFO(
      get_logger(),
      "Waiting for robot_state_publisher parameter service...");

    if (!parameter_client->wait_for_service(
        std::chrono::seconds(10)))
    {
      throw std::runtime_error(
              "robot_state_publisher parameter service is unavailable");
    }

    if (!parameter_client->has_parameter("robot_description")) {
      throw std::runtime_error(
              "robot_state_publisher has no robot_description parameter");
    }

    const std::string robot_description =
      parameter_client->get_parameter<std::string>(
        "robot_description");

    urdf::Model model;

    if (!model.initString(robot_description)) {
      throw std::runtime_error(
              "Failed to parse robot_description as URDF");
    }

    KDL::Tree tree;

    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
      throw std::runtime_error(
              "Failed to construct a KDL tree from the URDF");
    }

    if (!tree.getChain(base_frame_, tool_frame_, chain_)) {
      throw std::runtime_error(
              "Failed to construct the requested KDL chain");
    }

    if (chain_.getNrOfJoints() != 6U) {
      throw std::runtime_error(
              "Expected a six-joint KDL chain");
    }

    joint_names_.clear();

    for (const auto & segment : chain_.segments) {
      const auto & joint = segment.getJoint();

      if (joint.getType() != KDL::Joint::None) {
        joint_names_.push_back(joint.getName());
      }
    }

    if (joint_names_.size() != 6U) {
      throw std::runtime_error(
              "Failed to extract six actuated joint names");
    }

    joint_positions_ =
      KDL::JntArray(chain_.getNrOfJoints());

    jacobian_ =
      KDL::Jacobian(chain_.getNrOfJoints());

    fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(
        chain_);

    jacobian_solver_ =
      std::make_unique<KDL::ChainJntToJacSolver>(
        chain_);

    RCLCPP_INFO(
      get_logger(),
      "Required joint order:");

    for (std::size_t index = 0;
      index < joint_names_.size();
      ++index)
    {
      RCLCPP_INFO(
        get_logger(),
        "  %zu: %s",
        index,
        joint_names_.at(index).c_str());
    }
  }

  void wrench_callback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr message)
  {
    const Eigen::Vector3d force(
      message->wrench.force.x,
      message->wrench.force.y,
      message->wrench.force.z);

    if (!force.allFinite()) {
      wrench_received_ = false;
      cartesian_velocity_base_.setZero();
      return;
    }

    processed_force_tool_ = force;
    wrench_received_ = true;
    last_wrench_time_ = std::chrono::steady_clock::now();
  }

  void processing_ready_callback(
    const std_msgs::msg::Bool::SharedPtr message)
  {
    processing_ready_ = message->data;

    if (!processing_ready_) {
      cartesian_velocity_base_.setZero();
    }
  }

  void joint_callback(
    const sensor_msgs::msg::JointState::SharedPtr message)
  {
    if (message->name.size() != message->position.size()) {
      joints_received_ = false;
      cartesian_velocity_base_.setZero();
      return;
    }

    std::unordered_map<std::string, double> positions;

    for (std::size_t index = 0;
      index < message->name.size();
      ++index)
    {
      positions.emplace(
        message->name.at(index),
        message->position.at(index));
    }

    for (std::size_t index = 0;
      index < joint_names_.size();
      ++index)
    {
      const auto iterator =
        positions.find(joint_names_.at(index));

      if (iterator == positions.end()) {
        joints_received_ = false;
        cartesian_velocity_base_.setZero();
        return;
      }

      if (!std::isfinite(iterator->second)) {
        joints_received_ = false;
        cartesian_velocity_base_.setZero();
        return;
      }

      joint_positions_(index) = iterator->second;
    }

    joints_received_ = true;
    last_joint_time_ = std::chrono::steady_clock::now();
  }

  static void limit_vector_norm(
    Eigen::Vector3d & vector,
    const double maximum_norm)
  {
    const double norm = vector.norm();

    if (norm > maximum_norm && norm > 0.0) {
      vector *= maximum_norm / norm;
    }
  }

  void update()
  {
    const auto steady_now =
      std::chrono::steady_clock::now();

    double dt =
      std::chrono::duration<double>(
        steady_now - last_update_time_).count();

    last_update_time_ = steady_now;

    // Prevent a scheduling delay from producing a large integration step.
    dt = std::clamp(
      dt,
      0.001,
      0.050);

    if (!processing_ready_ ||
      !wrench_received_ ||
      !joints_received_)
    {
      stop_preview();
      return;
    }

    const double wrench_age =
      std::chrono::duration<double>(
        steady_now - last_wrench_time_).count();

    const double joint_age =
      std::chrono::duration<double>(
        steady_now - last_joint_time_).count();

    if (wrench_age > stale_timeout_ ||
      joint_age > stale_timeout_)
    {
      stop_preview();
      return;
    }

    KDL::Frame tool_pose;

    if (fk_solver_->JntToCart(
        joint_positions_,
        tool_pose) < 0)
    {
      stop_preview();
      return;
    }

    if (jacobian_solver_->JntToJac(
        joint_positions_,
        jacobian_) < 0)
    {
      stop_preview();
      return;
    }

    Eigen::Vector3d limited_force_tool =
      processed_force_tool_;

    limit_vector_norm(
      limited_force_tool,
      maximum_force_);

    // The processed wrench is expressed in tool0.
    // The KDL Jacobian is expressed in base_link.
    // Rotate the force into base_link before the admittance model.
    const KDL::Vector force_tool_kdl(
      limited_force_tool.x(),
      limited_force_tool.y(),
      limited_force_tool.z());

    const KDL::Vector force_base_kdl =
      tool_pose.M * force_tool_kdl;

    const Eigen::Vector3d force_base(
      force_base_kdl.x(),
      force_base_kdl.y(),
      force_base_kdl.z());

    // Translational admittance:
    //
    // M * acceleration + B * velocity = external force
    const Eigen::Vector3d acceleration_base =
      (
        force_base -
        damping_ * cartesian_velocity_base_
      ) / virtual_mass_;

    cartesian_velocity_base_ +=
      acceleration_base * dt;

    limit_vector_norm(
      cartesian_velocity_base_,
      maximum_cartesian_speed_);

    Eigen::MatrixXd jacobian_matrix(6, 6);

    for (int row = 0; row < 6; ++row) {
      for (int column = 0; column < 6; ++column) {
        jacobian_matrix(row, column) =
          jacobian_(row, column);
      }
    }

    Eigen::VectorXd desired_twist =
      Eigen::VectorXd::Zero(6);

    desired_twist(0) = cartesian_velocity_base_.x();
    desired_twist(1) = cartesian_velocity_base_.y();
    desired_twist(2) = cartesian_velocity_base_.z();

    Eigen::JacobiSVD<Eigen::MatrixXd> decomposition(
      jacobian_matrix,
      Eigen::ComputeThinU | Eigen::ComputeThinV);

    const Eigen::VectorXd singular_values =
      decomposition.singularValues();

    Eigen::VectorXd damped_inverse =
      Eigen::VectorXd::Zero(singular_values.size());

    const double lambda_squared =
      dls_lambda_ * dls_lambda_;

    for (Eigen::Index index = 0;
      index < singular_values.size();
      ++index)
    {
      const double singular_value =
        singular_values(index);

      damped_inverse(index) =
        singular_value /
        (
          singular_value * singular_value +
          lambda_squared
        );
    }

    const Eigen::VectorXd joint_velocity =
      decomposition.matrixV() *
      damped_inverse.asDiagonal() *
      decomposition.matrixU().transpose() *
      desired_twist;

    std_msgs::msg::Float64MultiArray joint_message;
    joint_message.data.resize(6, 0.0);

    for (Eigen::Index index = 0;
      index < joint_velocity.size();
      ++index)
    {
      joint_message.data.at(
        static_cast<std::size_t>(index)) =
        std::clamp(
          joint_velocity(index),
          -maximum_joint_speed_,
          maximum_joint_speed_);
    }

    joint_velocity_preview_publisher_->publish(
      joint_message);

    geometry_msgs::msg::TwistStamped cartesian_message;
    cartesian_message.header.stamp = now();
    cartesian_message.header.frame_id = base_frame_;

    cartesian_message.twist.linear.x =
      cartesian_velocity_base_.x();

    cartesian_message.twist.linear.y =
      cartesian_velocity_base_.y();

    cartesian_message.twist.linear.z =
      cartesian_velocity_base_.z();

    cartesian_message.twist.angular.x = 0.0;
    cartesian_message.twist.angular.y = 0.0;
    cartesian_message.twist.angular.z = 0.0;

    cartesian_velocity_preview_publisher_->publish(
      cartesian_message);

    publish_ready(true);
  }

  void stop_preview()
  {
    cartesian_velocity_base_.setZero();
    publish_zero_preview();
    publish_ready(false);
  }

  void publish_zero_preview()
  {
    std_msgs::msg::Float64MultiArray joint_message;
    joint_message.data.resize(6, 0.0);

    joint_velocity_preview_publisher_->publish(
      joint_message);

    geometry_msgs::msg::TwistStamped cartesian_message;
    cartesian_message.header.stamp = now();
    cartesian_message.header.frame_id = base_frame_;

    cartesian_velocity_preview_publisher_->publish(
      cartesian_message);
  }

  void publish_ready(const bool ready)
  {
    // Publish continuously so late subscribers can observe the current state.
    std_msgs::msg::Bool message;
    message.data = ready;
    ready_publisher_->publish(message);

    last_published_ready_ = ready;
    ready_state_published_ = true;
  }

  std::string base_frame_;
  std::string tool_frame_;

  double virtual_mass_{20.0};
  double damping_{120.0};
  double dls_lambda_{0.05};
  double maximum_force_{10.0};
  double maximum_cartesian_speed_{0.020};
  double maximum_joint_speed_{0.100};
  double stale_timeout_{0.10};
  double update_rate_{100.0};

  KDL::Chain chain_;
  KDL::JntArray joint_positions_;
  KDL::Jacobian jacobian_;

  std::unique_ptr<KDL::ChainFkSolverPos_recursive>
    fk_solver_;

  std::unique_ptr<KDL::ChainJntToJacSolver>
    jacobian_solver_;

  std::vector<std::string> joint_names_;

  Eigen::Vector3d processed_force_tool_{
    Eigen::Vector3d::Zero()};

  Eigen::Vector3d cartesian_velocity_base_{
    Eigen::Vector3d::Zero()};

  bool processing_ready_{false};
  bool wrench_received_{false};
  bool joints_received_{false};

  bool ready_state_published_{false};
  bool last_published_ready_{false};

  std::chrono::steady_clock::time_point
    last_wrench_time_;

  std::chrono::steady_clock::time_point
    last_joint_time_;

  std::chrono::steady_clock::time_point
    last_update_time_;

  rclcpp::Subscription<
    geometry_msgs::msg::WrenchStamped>::SharedPtr
    wrench_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr
    processing_ready_subscription_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr
    joint_subscription_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_velocity_preview_publisher_;

  rclcpp::Publisher<
    geometry_msgs::msg::TwistStamped>::SharedPtr
    cartesian_velocity_preview_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr
    ready_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<AdmittancePreview>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("admittance_preview"),
      "Preview startup failed: %s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
