#include <chrono>
#include <memory>
#include <string>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>


class AdmittanceControllerGuard : public rclcpp::Node
{
public:
  using ListControllers =
    controller_manager_msgs::srv::ListControllers;

  AdmittanceControllerGuard()
  : Node("admittance_controller_guard"),
    request_pending_(false),
    response_received_(false),
    evaluated_safe_(false),
    have_published_state_(false),
    last_published_state_(false),
    scaled_state_("missing"),
    forward_state_("missing")
  {
    controller_client_ =
      create_client<ListControllers>(
        "/controller_manager/list_controllers");

    state_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
        "/admittance/controller_state_safe",
        rclcpp::QoS(1).reliable().transient_local());

    last_request_time_ =
      std::chrono::steady_clock::now()
      - std::chrono::seconds(1);

    last_response_time_ =
      std::chrono::steady_clock::now()
      - std::chrono::seconds(10);

    timer_ =
      create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(
          &AdmittanceControllerGuard::timer_callback,
          this));

    RCLCPP_INFO(
      get_logger(),
      "Read-only controller-state guard started.");

    RCLCPP_INFO(
      get_logger(),
      "Safe preview configuration requires: "
      "scaled_joint_trajectory_controller=active and "
      "forward_velocity_controller=inactive.");

    RCLCPP_WARN(
      get_logger(),
      "This guard never switches controllers and never publishes "
      "robot-motion commands.");
  }

private:
  void send_request()
  {
    auto request =
      std::make_shared<ListControllers::Request>();

    request_pending_ = true;
    last_request_time_ =
      std::chrono::steady_clock::now();

    controller_client_->async_send_request(
      request,
      [this](
        rclcpp::Client<ListControllers>::SharedFuture future)
      {
        request_pending_ = false;

        bool scaled_found = false;
        bool forward_found = false;

        scaled_state_ = "missing";
        forward_state_ = "missing";

        try {
          const auto response = future.get();

          for (const auto & controller : response->controller) {
            if (
              controller.name ==
              "scaled_joint_trajectory_controller")
            {
              scaled_found = true;
              scaled_state_ = controller.state;
            }

            if (
              controller.name ==
              "forward_velocity_controller")
            {
              forward_found = true;
              forward_state_ = controller.state;
            }
          }

          evaluated_safe_ =
            scaled_found &&
            forward_found &&
            scaled_state_ == "active" &&
            forward_state_ == "inactive";

          response_received_ = true;
          last_response_time_ =
            std::chrono::steady_clock::now();
        } catch (const std::exception & exception) {
          evaluated_safe_ = false;
          response_received_ = false;

          RCLCPP_ERROR(
            get_logger(),
            "ListControllers request failed: %s",
            exception.what());
        }
      });
  }

  void publish_state(bool safe)
  {
    std_msgs::msg::Bool message;
    message.data = safe;
    state_publisher_->publish(message);

    if (
      !have_published_state_ ||
      safe != last_published_state_)
    {
      if (safe) {
        RCLCPP_INFO(
          get_logger(),
          "Controller state is SAFE for preview: "
          "scaled=%s, forward=%s.",
          scaled_state_.c_str(),
          forward_state_.c_str());
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Controller state is NOT SAFE for preview: "
          "scaled=%s, forward=%s. "
          "No motion command is issued by this guard.",
          scaled_state_.c_str(),
          forward_state_.c_str());
      }

      have_published_state_ = true;
      last_published_state_ = safe;
    }
  }

  void timer_callback()
  {
    const auto current_time =
      std::chrono::steady_clock::now();

    const auto request_age =
      std::chrono::duration<double>(
        current_time - last_request_time_).count();

    const auto response_age =
      std::chrono::duration<double>(
        current_time - last_response_time_).count();

    // Recover if a service request never returns.
    if (request_pending_ && request_age > 0.60) {
      request_pending_ = false;
      evaluated_safe_ = false;

      RCLCPP_WARN(
        get_logger(),
        "Controller-manager request timed out.");
    }

    if (
      !request_pending_ &&
      request_age >= 0.20 &&
      controller_client_->service_is_ready())
    {
      send_request();
    }

    const bool response_fresh =
      response_received_ &&
      response_age <= 0.60;

    const bool safe =
      response_fresh &&
      evaluated_safe_;

    publish_state(safe);
  }

  rclcpp::Client<ListControllers>::SharedPtr
    controller_client_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    state_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool request_pending_;
  bool response_received_;
  bool evaluated_safe_;
  bool have_published_state_;
  bool last_published_state_;

  std::string scaled_state_;
  std::string forward_state_;

  std::chrono::steady_clock::time_point
    last_request_time_;

  std::chrono::steady_clock::time_point
    last_response_time_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<AdmittanceControllerGuard>());

  rclcpp::shutdown();
  return 0;
}
