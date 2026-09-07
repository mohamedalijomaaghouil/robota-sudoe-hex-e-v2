#include <chrono>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

class AdmittanceZeroCommandBridge : public rclcpp::Node
{
public:
  AdmittanceZeroCommandBridge()
  : Node("admittance_zero_command_bridge")
  {
    declare_parameter<double>("publish_rate", 100.0);

    publish_rate_ =
      get_parameter("publish_rate").as_double();

    if (
      !std::isfinite(publish_rate_) ||
      publish_rate_ <= 0.0 ||
      publish_rate_ > 500.0)
    {
      throw std::runtime_error(
        "publish_rate must be finite and in the range (0, 500]");
    }

    command_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
        "/forward_velocity_controller/commands",
        rclcpp::QoS(1).reliable());

    running_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
        "/admittance/zero_bridge_running",
        rclcpp::QoS(1).reliable().transient_local());

    zero_command_.data.assign(6, 0.0);

    const auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_));

    timer_ =
      create_wall_timer(
        period,
        [this]()
        {
          // This node has deliberately no command input.
          // Every published joint command is hard-coded to zero.
          zero_command_.data.assign(6, 0.0);
          command_publisher_->publish(zero_command_);

          std_msgs::msg::Bool state;
          state.data = true;
          running_publisher_->publish(state);
        });

    RCLCPP_WARN(
      get_logger(),
      "ZERO-ONLY command bridge started at %.1f Hz.",
      publish_rate_);

    RCLCPP_WARN(
      get_logger(),
      "Every command contains exactly six zero velocities.");

    RCLCPP_WARN(
      get_logger(),
      "No nonzero-command input exists in this node.");

    RCLCPP_WARN(
      get_logger(),
      "For this validation, forward_velocity_controller must remain INACTIVE.");
  }

private:
  double publish_rate_{100.0};

  std_msgs::msg::Float64MultiArray zero_command_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    command_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr
    running_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<AdmittanceZeroCommandBridge>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("admittance_zero_command_bridge"),
      "Fatal error: %s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
