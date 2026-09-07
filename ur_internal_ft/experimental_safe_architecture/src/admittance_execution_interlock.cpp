#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ur_dashboard_msgs/msg/robot_mode.hpp>
#include <ur_dashboard_msgs/msg/safety_mode.hpp>

using namespace std::chrono_literals;

class AdmittanceExecutionInterlock : public rclcpp::Node
{
public:
  AdmittanceExecutionInterlock()
  : Node("admittance_execution_interlock")
  {
    declare_parameter<double>("maximum_speed_scaling_percent", 30.5);
    declare_parameter<double>("speed_scaling_stale_timeout", 0.20);
    declare_parameter<double>("controller_state_stale_timeout", 0.60);

    maximum_speed_scaling_percent_ =
      get_parameter("maximum_speed_scaling_percent").as_double();

    speed_scaling_stale_timeout_ =
      get_parameter("speed_scaling_stale_timeout").as_double();

    controller_state_stale_timeout_ =
      get_parameter("controller_state_stale_timeout").as_double();

    const auto status_qos =
      rclcpp::QoS(1).reliable().transient_local();

    program_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
        "/io_and_status_controller/robot_program_running",
        status_qos,
        [this](std_msgs::msg::Bool::SharedPtr message)
        {
          program_received_ = true;
          program_running_ = message->data;
        });

    safety_subscription_ =
      create_subscription<ur_dashboard_msgs::msg::SafetyMode>(
        "/io_and_status_controller/safety_mode",
        status_qos,
        [this](ur_dashboard_msgs::msg::SafetyMode::SharedPtr message)
        {
          safety_received_ = true;
          safety_mode_ = message->mode;
        });

    robot_mode_subscription_ =
      create_subscription<ur_dashboard_msgs::msg::RobotMode>(
        "/io_and_status_controller/robot_mode",
        status_qos,
        [this](ur_dashboard_msgs::msg::RobotMode::SharedPtr message)
        {
          robot_mode_received_ = true;
          robot_mode_ = message->mode;
        });

    speed_scaling_subscription_ =
      create_subscription<std_msgs::msg::Float64>(
        "/speed_scaling_state_broadcaster/speed_scaling",
        status_qos,
        [this](std_msgs::msg::Float64::SharedPtr message)
        {
          speed_scaling_received_ = true;
          speed_scaling_percent_ = message->data;
          last_speed_scaling_time_ =
            std::chrono::steady_clock::now();
        });

    controller_client_ =
      create_client<controller_manager_msgs::srv::ListControllers>(
        "/controller_manager/list_controllers");

    interlock_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
        "/admittance/execution_interlock_ok",
        rclcpp::QoS(1).reliable().transient_local());

    controller_timer_ =
      create_wall_timer(
        200ms,
        std::bind(
          &AdmittanceExecutionInterlock::request_controller_state,
          this));

    publish_timer_ =
      create_wall_timer(
        50ms,
        std::bind(
          &AdmittanceExecutionInterlock::publish_state,
          this));

    RCLCPP_INFO(
      get_logger(),
      "Read-only execution interlock started.");

    RCLCPP_INFO(
      get_logger(),
      "Execution requires program running, NORMAL safety mode, "
      "RUNNING robot mode, speed scaling <= %.1f%%, "
      "forward velocity active, and trajectory controller inactive.",
      maximum_speed_scaling_percent_);

    RCLCPP_WARN(
      get_logger(),
      "This node never switches controllers and never publishes "
      "robot-motion commands.");
  }

private:
  using ListControllers =
    controller_manager_msgs::srv::ListControllers;

  void request_controller_state()
  {
    if (!controller_client_->service_is_ready() ||
        controller_request_pending_)
    {
      return;
    }

    controller_request_pending_ = true;

    auto request =
      std::make_shared<ListControllers::Request>();

    controller_client_->async_send_request(
      request,
      [this](rclcpp::Client<ListControllers>::SharedFuture future)
      {
        controller_request_pending_ = false;

        bool scaled_found = false;
        bool forward_found = false;
        bool scaled_inactive = false;
        bool forward_active = false;

        try {
          const auto response = future.get();

          for (const auto & controller : response->controller) {
            if (
              controller.name ==
              "scaled_joint_trajectory_controller")
            {
              scaled_found = true;
              scaled_inactive =
                controller.state == "inactive";
            }

            if (
              controller.name ==
              "forward_velocity_controller")
            {
              forward_found = true;
              forward_active =
                controller.state == "active";
            }
          }

          controller_state_received_ = true;
          controller_state_ok_ =
            scaled_found &&
            forward_found &&
            scaled_inactive &&
            forward_active;

          last_controller_state_time_ =
            std::chrono::steady_clock::now();
        } catch (const std::exception & exception) {
          controller_state_ok_ = false;

          RCLCPP_ERROR(
            get_logger(),
            "Controller-state request failed: %s",
            exception.what());
        }
      });
  }

  bool evaluate(std::string & reason) const
  {
    const auto now =
      std::chrono::steady_clock::now();

    if (!program_received_) {
      reason = "robot-program state not received";
      return false;
    }

    if (!program_running_) {
      reason = "robot program is not running";
      return false;
    }

    if (!safety_received_) {
      reason = "safety mode not received";
      return false;
    }

    if (
      safety_mode_ !=
      ur_dashboard_msgs::msg::SafetyMode::NORMAL)
    {
      reason = "robot safety mode is not NORMAL";
      return false;
    }

    if (!robot_mode_received_) {
      reason = "robot mode not received";
      return false;
    }

    if (
      robot_mode_ !=
      ur_dashboard_msgs::msg::RobotMode::RUNNING)
    {
      reason = "robot mode is not RUNNING";
      return false;
    }

    if (!speed_scaling_received_) {
      reason = "speed scaling not received";
      return false;
    }

    const double speed_age =
      std::chrono::duration<double>(
        now - last_speed_scaling_time_).count();

    if (speed_age > speed_scaling_stale_timeout_) {
      reason = "speed scaling is stale";
      return false;
    }

    if (
      !std::isfinite(speed_scaling_percent_) ||
      speed_scaling_percent_ <= 0.0 ||
      speed_scaling_percent_ >
        maximum_speed_scaling_percent_)
    {
      reason = "speed scaling is outside the permitted range";
      return false;
    }

    if (!controller_state_received_) {
      reason = "controller state not received";
      return false;
    }

    const double controller_age =
      std::chrono::duration<double>(
        now - last_controller_state_time_).count();

    if (
      controller_age >
      controller_state_stale_timeout_)
    {
      reason = "controller state is stale";
      return false;
    }

    if (!controller_state_ok_) {
      reason =
        "execution controller configuration is not active";
      return false;
    }

    reason = "all execution conditions are satisfied";
    return true;
  }

  void publish_state()
  {
    std::string reason;
    const bool safe = evaluate(reason);

    std_msgs::msg::Bool message;
    message.data = safe;
    interlock_publisher_->publish(message);

    if (
      !last_published_state_valid_ ||
      safe != last_published_state_ ||
      reason != last_reason_)
    {
      if (safe) {
        RCLCPP_WARN(
          get_logger(),
          "Execution interlock TRUE: %s.",
          reason.c_str());
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Execution interlock FALSE: %s.",
          reason.c_str());
      }

      last_published_state_valid_ = true;
      last_published_state_ = safe;
      last_reason_ = reason;
    }
  }

  double maximum_speed_scaling_percent_{30.5};
  double speed_scaling_stale_timeout_{0.20};
  double controller_state_stale_timeout_{0.60};
  double speed_scaling_percent_{0.0};

  bool program_received_{false};
  bool program_running_{false};
  bool safety_received_{false};
  bool robot_mode_received_{false};
  bool speed_scaling_received_{false};
  bool controller_state_received_{false};
  bool controller_state_ok_{false};
  bool controller_request_pending_{false};

  bool last_published_state_valid_{false};
  bool last_published_state_{false};

  uint8_t safety_mode_{0};
  int8_t robot_mode_{0};

  std::string last_reason_;

  std::chrono::steady_clock::time_point
    last_speed_scaling_time_{};

  std::chrono::steady_clock::time_point
    last_controller_state_time_{};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    program_subscription_;

  rclcpp::Subscription<
    ur_dashboard_msgs::msg::SafetyMode>::SharedPtr
    safety_subscription_;

  rclcpp::Subscription<
    ur_dashboard_msgs::msg::RobotMode>::SharedPtr
    robot_mode_subscription_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
    speed_scaling_subscription_;

  rclcpp::Client<ListControllers>::SharedPtr
    controller_client_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    interlock_publisher_;

  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<AdmittanceExecutionInterlock>());

  rclcpp::shutdown();
  return 0;
}
