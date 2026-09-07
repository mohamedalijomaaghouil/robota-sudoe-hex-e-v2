#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>


class AdmittancePreviewGate : public rclcpp::Node
{
public:
  AdmittancePreviewGate()
  : Node("admittance_preview_gate")
  {
    declare_parameter<double>("stale_timeout", 0.35);
    declare_parameter<double>("enable_force_threshold", 0.25);
    declare_parameter<double>("automatic_disable_force", 12.0);
    declare_parameter<double>("update_rate", 100.0);

    stale_timeout_ =
      get_parameter("stale_timeout").as_double();

    enable_force_threshold_ =
      get_parameter("enable_force_threshold").as_double();

    automatic_disable_force_ =
      get_parameter("automatic_disable_force").as_double();

    update_rate_ =
      get_parameter("update_rate").as_double();

    if (stale_timeout_ <= 0.0 ||
      enable_force_threshold_ < 0.0 ||
      automatic_disable_force_ <= enable_force_threshold_ ||
      update_rate_ <= 0.0)
    {
      throw std::runtime_error("Invalid preview-gate parameters");
    }

    controller_state_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
        "/admittance/controller_state_safe",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::SharedPtr message)
        {
          controller_state_received_ = true;
          controller_state_safe_ = message->data;
          last_controller_state_time_ = now();

          if (!controller_state_safe_ && enabled_) {
            enabled_ = false;
            RCLCPP_ERROR(
              get_logger(),
              "Preview gate automatically disabled because the "
              "controller configuration is not safe for preview.");
          }
        });

    preview_ready_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
        "/admittance/preview_ready",
        10,
        [this](std_msgs::msg::Bool::SharedPtr message)
        {
          source_ready_ = message->data;
          last_ready_time_ = now();
          ready_received_ = true;

          if (!source_ready_) {
            disable("upstream preview is not ready");
          }
        });

    wrench_subscription_ =
      create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/admittance/processed_wrench",
        10,
        [this](geometry_msgs::msg::WrenchStamped::SharedPtr message)
        {
          const double x = message->wrench.force.x;
          const double y = message->wrench.force.y;
          const double z = message->wrench.force.z;

          if (!std::isfinite(x) ||
            !std::isfinite(y) ||
            !std::isfinite(z))
          {
            wrench_valid_ = false;
            disable("non-finite processed force");
            return;
          }

          force_norm_ = std::sqrt(x * x + y * y + z * z);
          wrench_valid_ = true;
          wrench_received_ = true;
          last_wrench_time_ = now();

          if (force_norm_ > automatic_disable_force_) {
            disable("force exceeded automatic-disable limit");
          }
        });

    joint_preview_subscription_ =
      create_subscription<std_msgs::msg::Float64MultiArray>(
        "/admittance/joint_velocity_preview",
        10,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr message)
        {
          if (message->data.size() != 6) {
            joint_preview_valid_ = false;
            disable("joint preview does not contain six values");
            return;
          }

          for (const double value : message->data) {
            if (!std::isfinite(value)) {
              joint_preview_valid_ = false;
              disable("non-finite joint preview");
              return;
            }
          }

          latest_joint_preview_ = *message;
          joint_preview_valid_ = true;
          joint_preview_received_ = true;
          last_joint_preview_time_ = now();
        });

    cartesian_preview_subscription_ =
      create_subscription<geometry_msgs::msg::TwistStamped>(
        "/admittance/cartesian_velocity_preview",
        10,
        [this](geometry_msgs::msg::TwistStamped::SharedPtr message)
        {
          const auto & linear = message->twist.linear;

          if (!std::isfinite(linear.x) ||
            !std::isfinite(linear.y) ||
            !std::isfinite(linear.z))
          {
            cartesian_preview_valid_ = false;
            disable("non-finite Cartesian preview");
            return;
          }

          latest_cartesian_preview_ = *message;
          cartesian_preview_valid_ = true;
          cartesian_preview_received_ = true;
          last_cartesian_preview_time_ = now();
        });

    gated_joint_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
        "/admittance/gated_joint_velocity_preview",
        10);

    gated_cartesian_publisher_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(
        "/admittance/gated_cartesian_velocity_preview",
        10);

    gate_enabled_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
        "/admittance/gate_enabled",
        10);

    enable_service_ =
      create_service<std_srvs::srv::SetBool>(
        "/admittance/preview_enable",
        std::bind(
          &AdmittancePreviewGate::enable_callback,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    const auto period = std::chrono::duration<double>(
      1.0 / update_rate_);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AdmittancePreviewGate::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "Admittance preview safety gate started.");

    RCLCPP_INFO(
      get_logger(),
      "Enable service: /admittance/preview_enable");

    RCLCPP_INFO(
      get_logger(),
      "Activation force threshold: %.3f N",
      enable_force_threshold_);

    RCLCPP_INFO(
      get_logger(),
      "Automatic-disable force: %.3f N",
      automatic_disable_force_);

    RCLCPP_WARN(
      get_logger(),
      "Gate starts DISABLED.");

    RCLCPP_WARN(
      get_logger(),
      "PREVIEW ONLY: no robot-controller command topic is published.");
  }

private:
  bool sources_are_valid(std::string & reason)
  {
    if (!controller_state_received_) {
      reason = "controller guard state has not been received";
      return false;
    }

    if (!controller_state_safe_) {
      reason = "controller configuration is not safe for preview";
      return false;
    }

    if (
      (now() - last_controller_state_time_).seconds()
      > stale_timeout_)
    {
      reason = "controller guard state is stale";
      return false;
    }

    if (!ready_received_ || !source_ready_) {
      reason = "upstream preview is not ready";
      return false;
    }

    if (!wrench_received_ || !wrench_valid_) {
      reason = "processed wrench is unavailable or invalid";
      return false;
    }

    if (!joint_preview_received_ || !joint_preview_valid_) {
      reason = "joint preview is unavailable or invalid";
      return false;
    }

    if (!cartesian_preview_received_ || !cartesian_preview_valid_) {
      reason = "Cartesian preview is unavailable or invalid";
      return false;
    }

    const auto current_time = now();

    if ((current_time - last_ready_time_).seconds() >
      stale_timeout_)
    {
      reason = "preview-ready signal is stale";
      return false;
    }

    if ((current_time - last_wrench_time_).seconds() >
      stale_timeout_)
    {
      reason = "processed wrench is stale";
      return false;
    }

    if ((current_time - last_joint_preview_time_).seconds() >
      stale_timeout_)
    {
      reason = "joint preview is stale";
      return false;
    }

    if ((current_time - last_cartesian_preview_time_).seconds() >
      stale_timeout_)
    {
      reason = "Cartesian preview is stale";
      return false;
    }

    if (force_norm_ > automatic_disable_force_) {
      reason = "force exceeds automatic-disable limit";
      return false;
    }

    return true;
  }

  void enable_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (!request->data) {
      disable("disable service request");

      response->success = true;
      response->message = "Preview gate disabled.";
      return;
    }

    std::string reason;

    if (!sources_are_valid(reason)) {
      enabled_ = false;
      response->success = false;
      response->message =
        "Enable rejected: " + reason;
      return;
    }

    if (force_norm_ > enable_force_threshold_) {
      enabled_ = false;
      response->success = false;
      response->message =
        "Enable rejected: force norm is " +
        std::to_string(force_norm_) +
        " N; retare before enabling.";
      return;
    }

    enabled_ = true;

    response->success = true;
    response->message =
      "Preview gate enabled. This remains preview-only.";

    RCLCPP_WARN(
      get_logger(),
      "Preview gate ENABLED. No real robot command is published.");
  }

  void disable(const std::string & reason)
  {
    if (enabled_) {
      RCLCPP_WARN(
        get_logger(),
        "Preview gate automatically disabled: %s",
        reason.c_str());
    }

    enabled_ = false;
  }

  void publish_zero()
  {
    std_msgs::msg::Float64MultiArray joint_message;
    joint_message.data.assign(6, 0.0);
    gated_joint_publisher_->publish(joint_message);

    geometry_msgs::msg::TwistStamped cartesian_message;
    cartesian_message.header.stamp = now();
    cartesian_message.header.frame_id = "base_link";
    gated_cartesian_publisher_->publish(cartesian_message);
  }

  void timer_callback()
  {
    std::string reason;
    const bool sources_valid = sources_are_valid(reason);

    if (enabled_ && !sources_valid) {
      disable(reason);
    }

    if (enabled_) {
      gated_joint_publisher_->publish(
        latest_joint_preview_);

      gated_cartesian_publisher_->publish(
        latest_cartesian_preview_);
    } else {
      publish_zero();
    }

    std_msgs::msg::Bool enabled_message;
    enabled_message.data = enabled_;
    gate_enabled_publisher_->publish(enabled_message);
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    controller_state_subscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    preview_ready_subscription_;

  rclcpp::Subscription<
    geometry_msgs::msg::WrenchStamped>::SharedPtr
    wrench_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_preview_subscription_;

  rclcpp::Subscription<
    geometry_msgs::msg::TwistStamped>::SharedPtr
    cartesian_preview_subscription_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    gated_joint_publisher_;

  rclcpp::Publisher<
    geometry_msgs::msg::TwistStamped>::SharedPtr
    gated_cartesian_publisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    gate_enabled_publisher_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
    enable_service_;

  rclcpp::TimerBase::SharedPtr timer_;

  std_msgs::msg::Float64MultiArray latest_joint_preview_;
  geometry_msgs::msg::TwistStamped latest_cartesian_preview_;

  rclcpp::Time last_controller_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ready_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wrench_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joint_preview_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cartesian_preview_time_{0, 0, RCL_ROS_TIME};

  double stale_timeout_{0.35};
  double enable_force_threshold_{0.25};
  double automatic_disable_force_{12.0};
  double update_rate_{100.0};
  double force_norm_{0.0};

  bool controller_state_received_{false};
  bool controller_state_safe_{false};
  bool enabled_{false};
  bool source_ready_{false};
  bool ready_received_{false};

  bool wrench_received_{false};
  bool wrench_valid_{false};

  bool joint_preview_received_{false};
  bool joint_preview_valid_{false};

  bool cartesian_preview_received_{false};
  bool cartesian_preview_valid_{false};
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<AdmittancePreviewGate>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("admittance_preview_gate"),
      "%s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
