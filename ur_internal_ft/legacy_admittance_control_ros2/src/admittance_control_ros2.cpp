#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <memory>

class AdmittanceControlROS2 : public rclcpp::Node {
public:
  AdmittanceControlROS2() : Node("admittance_control") {
    // Parameters
    this->declare_parameter<std::string>("sensor",         "ethercat");
    this->declare_parameter<std::string>("tool_frame",     "tool0");
    this->declare_parameter<std::string>("base_frame",     "base_link");
    this->declare_parameter<double>("M_trans",    8.0);
    this->declare_parameter<double>("M_rot",      0.07);
    this->declare_parameter<double>("c",          15.0);
    this->declare_parameter<double>("lambda_dls", 0.015);
    this->declare_parameter<double>("F_alpha",    0.015);
    this->declare_parameter<double>("V_alpha",    0.15);
    this->declare_parameter<double>("dead_cart",  0.05);
    this->declare_parameter<double>("dead_rot",   0.003);
    this->declare_parameter<double>("freq",       500.0);

    sensor_     = this->get_parameter("sensor").as_string();
    tool_frame_ = this->get_parameter("tool_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    double M_t  = this->get_parameter("M_trans").as_double();
    double M_r  = this->get_parameter("M_rot").as_double();
    double c    = this->get_parameter("c").as_double();
    lambda_     = this->get_parameter("lambda_dls").as_double();
    F_alpha_    = this->get_parameter("F_alpha").as_double();
    V_alpha_    = this->get_parameter("V_alpha").as_double();
    dead_cart_  = this->get_parameter("dead_cart").as_double();
    dead_rot_   = this->get_parameter("dead_rot").as_double();
    freq_       = this->get_parameter("freq").as_double();

    M_vec_.resize(6); B_vec_.resize(6); K_vec_.resize(6);
    M_vec_ << M_t, M_t, M_t, M_r, M_r, M_r;
    B_vec_ << c*M_t, c*M_t, c*M_t, c*M_r, c*M_r, c*M_r;
    K_vec_.setZero();

    v_k_.setZero(6); x_k_.setZero(6); a_k_.setZero(6);
    filt_f_.setZero(6); smooth_.setZero(6);

    // Sensor rotation correction
    double zcorr = (sensor_ == "ethercat" || sensor_ == "serial_rkb") ? M_PI/2.0 : 0.0;
    double c2 = std::cos(zcorr), s2 = std::sin(zcorr);
    corr_ << c2,-s2,0, s2,c2,0, 0,0,1;

    // TF2
    tf_buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_lis_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);

    // KDL from robot_description
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
    std::string urdf;
    if (param_client->wait_for_service(std::chrono::seconds(5))) {
      auto p = param_client->get_parameters({"robot_description"});
      if (!p.empty()) urdf = p[0].as_string();
    }
    if (urdf.empty()) throw std::runtime_error("No robot_description");
    urdf::Model model;
    if (!model.initString(urdf)) throw std::runtime_error("URDF parse error");
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) throw std::runtime_error("KDL tree error");
    if (!tree.getChain(base_frame_, tool_frame_, chain_)) throw std::runtime_error("KDL chain error");
    jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);

    joint_names_ = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                    "wrist_1_joint","wrist_2_joint","wrist_3_joint"};

    // Wrench topic
    std::string wtopic = "/force_sensor_eth";
    if (sensor_ == "UR") wtopic = "/ft_sensor_wrench";
    sub_w_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(wtopic, 1,
      [this](geometry_msgs::msg::WrenchStamped::SharedPtr m){ last_w_=*m; has_w_=true; });

    sub_j_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1,
      [this](sensor_msgs::msg::JointState::SharedPtr m){ last_j_=*m; has_j_=true; });

    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", 1);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0/freq_),
      std::bind(&AdmittanceControlROS2::loop, this));

    RCLCPP_INFO(this->get_logger(), "Admittance ready. sensor=%s freq=%.0f Hz", sensor_.c_str(), freq_);
  }

private:
  void loop() {
    if (!has_w_ || !has_j_) return;

    // Transform wrench to base frame
    geometry_msgs::msg::WrenchStamped wb;
    try {
      auto tf = tf_buf_->lookupTransform(base_frame_, tool_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
      tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                        tf.transform.rotation.z, tf.transform.rotation.w);
      tf2::Matrix3x3 Rtf(q);
      Eigen::Matrix3d R;
      for (int i=0;i<3;i++) for (int j=0;j<3;j++) R(i,j)=Rtf[i][j];
      Eigen::Vector3d Fv(last_w_.wrench.force.x, last_w_.wrench.force.y, last_w_.wrench.force.z);
      Eigen::Vector3d Tv(last_w_.wrench.torque.x, last_w_.wrench.torque.y, last_w_.wrench.torque.z);
      Fv = R * corr_ * Fv; Tv = R * corr_ * Tv;
      wb.wrench.force.x=Fv(0); wb.wrench.force.y=Fv(1); wb.wrench.force.z=Fv(2);
      wb.wrench.torque.x=Tv(0); wb.wrench.torque.y=Tv(1); wb.wrench.torque.z=Tv(2);
    } catch (...) { return; }

    Eigen::VectorXd f(6);
    f << wb.wrench.force.x, wb.wrench.force.y, wb.wrench.force.z,
         wb.wrench.torque.x, wb.wrench.torque.y, wb.wrench.torque.z;

    smooth_ = (1.0-F_alpha_)*smooth_ + F_alpha_*f;
    filt_f_ = smooth_;
    if (filt_f_.head<3>().norm() < dead_cart_) filt_f_.head<3>().setZero();
    if (filt_f_.tail<3>().norm() < dead_rot_)  filt_f_.tail<3>().setZero();

    // Joint positions
    std::map<std::string,size_t> idx;
    for (size_t i=0;i<last_j_.name.size();i++) idx[last_j_.name[i]]=i;
    KDL::JntArray q(chain_.getNrOfJoints());
    for (size_t j=0;j<joint_names_.size();j++) {
      auto it=idx.find(joint_names_[j]);
      if (it!=idx.end()) q(j)=last_j_.position[it->second];
    }

    // Jacobian
    KDL::Jacobian jac(chain_.getNrOfJoints());
    if (jac_solver_->JntToJac(q,jac)<0) return;
    Eigen::MatrixXd J(6, chain_.getNrOfJoints());
    for (unsigned i=0;i<6;i++) for (unsigned j=0;j<(unsigned)chain_.getNrOfJoints();j++) J(i,j)=jac(i,j);

    // Admittance
    double dt = 1.0/freq_;
    a_k_ = (filt_f_ - B_vec_.cwiseProduct(v_k_) - K_vec_.cwiseProduct(x_k_)).cwiseQuotient(M_vec_);
    v_k_ = v_k_ + a_k_*dt;
    x_k_ = v_k_*dt;

    // DLS pseudoinverse
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU|Eigen::ComputeThinV);
    Eigen::VectorXd S = svd.singularValues();
    for (int i=0;i<S.size();i++) S(i)=S(i)/(S(i)*S(i)+lambda_*lambda_);
    Eigen::MatrixXd Jp = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
    Eigen::VectorXd qdot = Jp*v_k_;

    // Velocity filter
    if (!vel_init_) { qs_ = qdot; vel_init_=true; }
    else qs_ = V_alpha_*qdot + (1.0-V_alpha_)*qs_;

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(qs_.data(), qs_.data()+qs_.size());
    pub_->publish(cmd);
  }

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_w_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_j_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_lis_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  bool has_w_=false, has_j_=false;
  geometry_msgs::msg::WrenchStamped last_w_;
  sensor_msgs::msg::JointState last_j_;

  Eigen::VectorXd v_k_,x_k_,a_k_,filt_f_,smooth_,M_vec_,B_vec_,K_vec_,qs_;
  Eigen::Matrix3d corr_;
  bool vel_init_=false;
  std::string sensor_,tool_frame_,base_frame_;
  double lambda_,F_alpha_,V_alpha_,dead_cart_,dead_rot_,freq_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AdmittanceControlROS2>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("admittance"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
