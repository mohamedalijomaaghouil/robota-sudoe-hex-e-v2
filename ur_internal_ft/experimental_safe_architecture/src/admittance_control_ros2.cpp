#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

class SafeAdmittanceControl : public rclcpp::Node
{
public:
  SafeAdmittanceControl()
  : Node("safe_admittance_control")
  {
    declare_parameter<std::string>(
      "wrench_topic", "/ft_sensor_wrench");
    declare_parameter<int>("tare_samples", 500);
    declare_parameter<double>("filter_alpha", 0.10);
    declare_parameter<double>("deadband_x", 1.10);
    declare_parameter<double>("deadband_y", 1.00);
    declare_parameter<double>("deadband_z", 0.60);
    declare_parameter<double>("stale_timeout", 0.10);

    wrench_topic_ =
      get_parameter("wrench_topic").as_string();
    tare_samples_ =
      get_parameter("tare_samples").as_int();
    alpha_ =
      get_parameter("filter_alpha").as_double();
    deadband_x_ =
      get_parameter("deadband_x").as_double();
    deadband_y_ =
      get_parameter("deadband_y").as_double();
    deadband_z_ =
      get_parameter("deadband_z").as_double();
    stale_timeout_ =
      get_parameter("stale_timeout").as_double();

    processed_publisher_ =
      create_publisher<geometry_msgs::msg::WrenchStamped>(
        "/admittance/processed_wrench", 10);

    ready_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
        "/admittance/ready", 10);

    retare_service_ =
      create_service<std_srvs::srv::Trigger>(
        "/admittance/retare",
        std::bind(
          &SafeAdmittanceControl::retare_callback,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    wrench_subscription_ =
      create_subscription<geometry_msgs::msg::WrenchStamped>(
        wrench_topic_,
        100,
        std::bind(
          &SafeAdmittanceControl::wrench_callback,
          this,
          std::placeholders::_1));

    watchdog_timer_ =
      create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(
          &SafeAdmittanceControl::watchdog_callback,
          this));

    RCLCPP_INFO(
      get_logger(),
      "Safe admittance Phase 1 started.");

    RCLCPP_INFO(
      get_logger(),
      "Input wrench topic: %s",
      wrench_topic_.c_str());

    RCLCPP_WARN(
      get_logger(),
      "Keep the complete tool stationary and unloaded "
      "during the %d-sample startup tare.",
      tare_samples_);

    RCLCPP_WARN(
      get_logger(),
      "DIAGNOSTIC ONLY: this node has no robot-motion publisher.");
  }

private:
  static double apply_soft_deadband(
    const double value,
    const double threshold)
  {
    if (value > threshold) {
      return value - threshold;
    }

    if (value < -threshold) {
      return value + threshold;
    }

    return 0.0;
  }

  void publish_ready(const bool value)
  {
    std_msgs::msg::Bool message;
    message.data = value;
    ready_publisher_->publish(message);
  }

  void retare_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    tare_complete_ = false;
    filter_initialized_ = false;

    tare_count_ = 0;
    tare_sum_x_ = 0.0;
    tare_sum_y_ = 0.0;
    tare_sum_z_ = 0.0;

    bias_x_ = 0.0;
    bias_y_ = 0.0;
    bias_z_ = 0.0;

    filtered_x_ = 0.0;
    filtered_y_ = 0.0;
    filtered_z_ = 0.0;

    publish_ready(false);

    response->success = true;
    response->message =
      "Software retare started. Keep the tool stationary and unloaded.";

    RCLCPP_WARN(
      get_logger(),
      "Software retare requested. Keep the complete tool stationary "
      "and unloaded during the %d-sample tare.",
      tare_samples_);
  }

  void wrench_callback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr message)
  {
    const double force_x = message->wrench.force.x;
    const double force_y = message->wrench.force.y;
    const double force_z = message->wrench.force.z;

    if (
      !std::isfinite(force_x) ||
      !std::isfinite(force_y) ||
      !std::isfinite(force_z))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Discarding non-finite F/T sensor sample.");

      publish_ready(false);
      return;
    }

    last_message_time_ = std::chrono::steady_clock::now();
    received_message_ = true;

    if (!tare_complete_) {
      tare_sum_x_ += force_x;
      tare_sum_y_ += force_y;
      tare_sum_z_ += force_z;
      tare_count_ += 1;

      if (
        tare_count_ == 1 ||
        tare_count_ % 100 == 0)
      {
        RCLCPP_INFO(
          get_logger(),
          "Collecting startup tare: %d/%d",
          tare_count_,
          tare_samples_);
      }

      if (tare_count_ >= tare_samples_) {
        bias_x_ = tare_sum_x_ / tare_count_;
        bias_y_ = tare_sum_y_ / tare_count_;
        bias_z_ = tare_sum_z_ / tare_count_;

        tare_complete_ = true;
        filter_initialized_ = false;

        RCLCPP_INFO(
          get_logger(),
          "Startup tare complete.");

        RCLCPP_INFO(
          get_logger(),
          "Estimated force bias [N]: "
          "[%.6f, %.6f, %.6f]",
          bias_x_,
          bias_y_,
          bias_z_);
      }

      publish_ready(false);
      return;
    }

    const double zeroed_x = force_x - bias_x_;
    const double zeroed_y = force_y - bias_y_;
    const double zeroed_z = force_z - bias_z_;

    if (!filter_initialized_) {
      filtered_x_ = zeroed_x;
      filtered_y_ = zeroed_y;
      filtered_z_ = zeroed_z;
      filter_initialized_ = true;
    } else {
      filtered_x_ =
        alpha_ * zeroed_x +
        (1.0 - alpha_) * filtered_x_;

      filtered_y_ =
        alpha_ * zeroed_y +
        (1.0 - alpha_) * filtered_y_;

      filtered_z_ =
        alpha_ * zeroed_z +
        (1.0 - alpha_) * filtered_z_;
    }

    geometry_msgs::msg::WrenchStamped output;
    output.header = message->header;
    output.header.frame_id = "tool0";

    output.wrench.force.x =
      apply_soft_deadband(filtered_x_, deadband_x_);

    output.wrench.force.y =
      apply_soft_deadband(filtered_y_, deadband_y_);

    output.wrench.force.z =
      apply_soft_deadband(filtered_z_, deadband_z_);

    // Rotation is intentionally disabled during Phase 1.
    output.wrench.torque.x = 0.0;
    output.wrench.torque.y = 0.0;
    output.wrench.torque.z = 0.0;

    processed_publisher_->publish(output);
    publish_ready(true);
  }

  void watchdog_callback()
  {
    if (!received_message_ || !tare_complete_) {
      publish_ready(false);
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double age =
      std::chrono::duration<double>(
        now - last_message_time_).count();

    if (age > stale_timeout_) {
      publish_ready(false);

      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "F/T sensor data is stale: age=%.3f seconds.",
        age);
    }
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    retare_service_;

  rclcpp::Subscription<
    geometry_msgs::msg::WrenchStamped>::SharedPtr
    wrench_subscription_;

  rclcpp::Publisher<
    geometry_msgs::msg::WrenchStamped>::SharedPtr
    processed_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr
    ready_publisher_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::string wrench_topic_;

  int tare_samples_{500};
  int tare_count_{0};

  double alpha_{0.10};
  double deadband_x_{1.10};
  double deadband_y_{1.00};
  double deadband_z_{0.60};
  double stale_timeout_{0.10};

  double tare_sum_x_{0.0};
  double tare_sum_y_{0.0};
  double tare_sum_z_{0.0};

  double bias_x_{0.0};
  double bias_y_{0.0};
  double bias_z_{0.0};

  double filtered_x_{0.0};
  double filtered_y_{0.0};
  double filtered_z_{0.0};

  bool tare_complete_{false};
  bool filter_initialized_{false};
  bool received_message_{false};

  std::chrono::steady_clock::time_point last_message_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<SafeAdmittanceControl>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("safe_admittance_control"),
      "Unhandled exception: %s",
      exception.what());
  }

  rclcpp::shutdown();
  return 0;
}
