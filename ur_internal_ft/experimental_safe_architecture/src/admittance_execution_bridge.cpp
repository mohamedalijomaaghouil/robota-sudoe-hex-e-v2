#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

class AdmittanceExecutionBridge : public rclcpp::Node
{
public:
  AdmittanceExecutionBridge()
  : Node("admittance_execution_bridge")
  {
    declare_parameter<double>("publish_rate", 100.0);
    declare_parameter<double>("command_stale_timeout", 0.15);
    declare_parameter<double>("interlock_stale_timeout", 0.25);
    declare_parameter<double>("ready_stale_timeout", 0.25);
    declare_parameter<double>("deadman_stale_timeout", 0.20);
    declare_parameter<double>("maximum_joint_speed", 0.005);
    declare_parameter<double>("enable_maximum_joint_speed", 0.001);

    publish_rate_ =
      get_parameter("publish_rate").as_double();
    command_stale_timeout_ =
      get_parameter("command_stale_timeout").as_double();
    interlock_stale_timeout_ =
      get_parameter("interlock_stale_timeout").as_double();
    ready_stale_timeout_ =
      get_parameter("ready_stale_timeout").as_double();
    deadman_stale_timeout_ =
      get_parameter("deadman_stale_timeout").as_double();
    maximum_joint_speed_ =
      get_parameter("maximum_joint_speed").as_double();
    enable_maximum_joint_speed_ =
      get_parameter("enable_maximum_joint_speed").as_double();

    if (
      !std::isfinite(publish_rate_) ||
      !std::isfinite(command_stale_timeout_) ||
      !std::isfinite(interlock_stale_timeout_) ||
      !std::isfinite(ready_stale_timeout_) ||
      !std::isfinite(deadman_stale_timeout_) ||
      !std::isfinite(maximum_joint_speed_) ||
      !std::isfinite(enable_maximum_joint_speed_) ||
      publish_rate_ <= 0.0 ||
      publish_rate_ > 500.0 ||
      command_stale_timeout_ <= 0.0 ||
      interlock_stale_timeout_ <= 0.0 ||
      ready_stale_timeout_ <= 0.0 ||
      deadman_stale_timeout_ <= 0.0 ||
      maximum_joint_speed_ <= 0.0 ||
      enable_maximum_joint_speed_ < 0.0 ||
      enable_maximum_joint_speed_ > maximum_joint_speed_)
    {
      throw std::runtime_error("Invalid execution-bridge parameters");
    }

    zero_command_.data.assign(6, 0.0);
    latest_command_.data.assign(6, 0.0);

    command_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
        "/forward_velocity_controller/commands",
        rclcpp::QoS(1).best_effort());

    enabled_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
        "/admittance/execution_enabled",
        rclcpp::QoS(1).reliable().transient_local());

    command_subscription_ =
      create_subscription<std_msgs::msg::Float64MultiArray>(
        "/admittance/joint_velocity_preview",
        rclcpp::QoS(1).best_effort(),
        [this](std_msgs::msg::Float64MultiArray::SharedPtr message)
        {
          command_received_ = true;
          command_valid_ = validate_command(*message);
          last_command_time_ = Clock::now();

          if (!command_valid_) {
            disable("invalid joint-velocity preview");
            latest_command_.data.assign(6, 0.0);
            return;
          }

          latest_command_ = *message;
        });

    interlock_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
        "/admittance/execution_interlock_ok",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::SharedPtr message)
        {
          interlock_received_ = true;
          interlock_ok_ = message->data;
          last_interlock_time_ = Clock::now();

          if (!interlock_ok_) {
            disable("execution interlock became false");
          }
        });

    ready_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
        "/admittance/preview_ready",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::SharedPtr message)
        {
          ready_received_ = true;
          preview_ready_ = message->data;
          last_ready_time_ = Clock::now();

          if (!preview_ready_) {
            disable("computational preview is not ready");
          }
        });

    deadman_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
        "/admittance/deadman",
        rclcpp::QoS(1).reliable(),
        [this](std_msgs::msg::Bool::SharedPtr message)
        {
          deadman_received_ = true;
          deadman_pressed_ = message->data;
          last_deadman_time_ = Clock::now();

          if (!deadman_pressed_) {
            disable("deadman released");
          }
        });

    enable_service_ =
      create_service<std_srvs::srv::SetBool>(
        "/admittance/execution_enable",
        [this](
          const std_srvs::srv::SetBool::Request::SharedPtr request,
          std_srvs::srv::SetBool::Response::SharedPtr response)
        {
          if (!request->data) {
            disable("disable service request");
            response->success = true;
            response->message = "Admittance execution disabled.";
            return;
          }

          std::string reason;
          if (!conditions_valid(reason, true)) {
            enabled_ = false;
            response->success = false;
            response->message =
              "Execution enable rejected: " + reason;
            return;
          }

          enabled_ = true;
          response->success = true;
          response->message =
            "Admittance execution enabled with all interlocks valid.";

          RCLCPP_WARN(
            get_logger(),
            "EXECUTION ENABLED with %.4f rad/s per-joint clamp.",
            maximum_joint_speed_);
        });

    const auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_));

    timer_ =
      create_wall_timer(
        period,
        [this]()
        {
          std_msgs::msg::Float64MultiArray output = zero_command_;

          if (enabled_) {
            std::string reason;

            if (!conditions_valid(reason, false)) {
              disable(reason);
            } else {
              output = latest_command_;

              for (auto & velocity : output.data) {
                velocity = std::clamp(
                  velocity,
                  -maximum_joint_speed_,
                  maximum_joint_speed_);
              }
            }
          }

          command_publisher_->publish(output);

          std_msgs::msg::Bool state;
          state.data = enabled_;
          enabled_publisher_->publish(state);
        });

    RCLCPP_WARN(
      get_logger(),
      "Interlocked execution bridge started DISABLED.");

    RCLCPP_WARN(
      get_logger(),
      "Output is hard-limited to %.4f rad/s per joint.",
      maximum_joint_speed_);

    RCLCPP_WARN(
      get_logger(),
      "This node never switches controllers.");

    RCLCPP_WARN(
      get_logger(),
      "Without enable, fresh deadman, preview readiness and "
      "execution interlock, it publishes six zeros.");
  }

private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  static double age_seconds(const TimePoint & stamp)
  {
    return std::chrono::duration<double>(
      Clock::now() - stamp).count();
  }

  bool validate_command(
    const std_msgs::msg::Float64MultiArray & command) const
  {
    if (command.data.size() != 6U) {
      return false;
    }

    return std::all_of(
      command.data.begin(),
      command.data.end(),
      [](double value)
      {
        return std::isfinite(value);
      });
  }

  double maximum_input_speed() const
  {
    double result = 0.0;

    for (const auto velocity : latest_command_.data) {
      result = std::max(result, std::abs(velocity));
    }

    return result;
  }

  bool conditions_valid(
    std::string & reason,
    bool enabling) const
  {
    if (!interlock_received_) {
      reason = "execution interlock was not received";
      return false;
    }

    if (!interlock_ok_) {
      reason = "execution interlock is false";
      return false;
    }

    if (
      age_seconds(last_interlock_time_) >
      interlock_stale_timeout_)
    {
      reason = "execution interlock is stale";
      return false;
    }

    if (!ready_received_ || !preview_ready_) {
      reason = "computational preview is not ready";
      return false;
    }

    if (age_seconds(last_ready_time_) > ready_stale_timeout_) {
      reason = "preview-ready state is stale";
      return false;
    }

    if (!command_received_ || !command_valid_) {
      reason = "valid six-joint preview was not received";
      return false;
    }

    if (age_seconds(last_command_time_) > command_stale_timeout_) {
      reason = "joint-velocity preview is stale";
      return false;
    }

    if (!deadman_received_ || !deadman_pressed_) {
      reason = "deadman is not pressed";
      return false;
    }

    if (age_seconds(last_deadman_time_) > deadman_stale_timeout_) {
      reason = "deadman heartbeat is stale";
      return false;
    }

    if (
      enabling &&
      maximum_input_speed() > enable_maximum_joint_speed_)
    {
      reason =
        "preview is not sufficiently close to zero for activation";
      return false;
    }

    return true;
  }

  void disable(const std::string & reason)
  {
    if (enabled_) {
      enabled_ = false;

      RCLCPP_ERROR(
        get_logger(),
        "Execution automatically disabled: %s",
        reason.c_str());
    }
  }

  double publish_rate_{100.0};
  double command_stale_timeout_{0.15};
  double interlock_stale_timeout_{0.25};
  double ready_stale_timeout_{0.25};
  double deadman_stale_timeout_{0.20};
  double maximum_joint_speed_{0.005};
  double enable_maximum_joint_speed_{0.001};

  bool enabled_{false};
  bool command_received_{false};
  bool command_valid_{false};
  bool interlock_received_{false};
  bool interlock_ok_{false};
  bool ready_received_{false};
  bool preview_ready_{false};
  bool deadman_received_{false};
  bool deadman_pressed_{false};

  TimePoint last_command_time_{};
  TimePoint last_interlock_time_{};
  TimePoint last_ready_time_{};
  TimePoint last_deadman_time_{};

  std_msgs::msg::Float64MultiArray zero_command_;
  std_msgs::msg::Float64MultiArray latest_command_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    command_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr
    enabled_publisher_;

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    command_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr
    interlock_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr
    ready_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr
    deadman_subscription_;

  rclcpp::Service<
    std_srvs::srv::SetBool>::SharedPtr
    enable_service_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<AdmittanceExecutionBridge>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("admittance_execution_bridge"),
      "%s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
