#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class NavToPoseClient : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavToPoseClient() : Node("nav_to_pose_client") {
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  void send_goal(double x, double y, double yaw_rad = 0.0) {
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available");
      return;
    }

    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();

    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = std::sin(yaw_rad * 0.5);
    goal_msg.pose.pose.orientation.w = std::cos(yaw_rad * 0.5);
    goal_msg.behavior_tree = "";

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

    options.goal_response_callback = [this](const GoalHandleNav::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        }
      };

    options.feedback_callback = [this](GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        RCLCPP_INFO(
          this->get_logger(),
          "Distance remaining: %.2f m, recoveries: %d",
          feedback->distance_remaining,
          feedback->number_of_recoveries);
      };

    options.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Navigation was canceled");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
      };

    client_->async_send_goal(goal_msg, options);
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<NavToPoseClient>();
  node->send_goal(5.0, 2.0, 0.0);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
