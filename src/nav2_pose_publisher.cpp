#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

using namespace std::chrono_literals;

class FollowWaypointsClient : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  FollowWaypointsClient() : Node("follow_waypoints_client") {
    client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

    timer_ = this->create_wall_timer(
      1s, std::bind(&FollowWaypointsClient::send_goal_once, this));
  }

private:
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_sent_{false};

  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = yawToQuaternion(yaw);

    return pose;
  }

  void send_goal_once() {
    if (goal_sent_) {
      return;
    }

    if (!client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for follow_waypoints action server...");
      return;
    }

    goal_sent_ = true;
    timer_->cancel();

    FollowWaypoints::Goal goal_msg;

    goal_msg.poses.push_back(makePose(1.0, 0.5, 0.0));
    goal_msg.poses.push_back(makePose(2.2, 0.5, 0.0));
    goal_msg.poses.push_back(makePose(2.2, 1.8, 1.57));
    goal_msg.poses.push_back(makePose(1.0, 1.8, 3.14));

    RCLCPP_INFO(this->get_logger(), "Sending %zu waypoints", goal_msg.poses.size());

    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&FollowWaypointsClient::goal_response_callback, this, std::placeholders::_1);

    send_goal_options.feedback_callback =
      std::bind(&FollowWaypointsClient::feedback_callback, this,
      std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback =
      std::bind(&FollowWaypointsClient::result_callback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr & goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
    }
  }

  void feedback_callback(
    GoalHandleFollowWaypoints::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
    RCLCPP_INFO(
      this->get_logger(),
      "Currently heading to waypoint index: %u",
      feedback->current_waypoint);
  }

  void result_callback(const GoalHandleFollowWaypoints::WrappedResult & result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Waypoint mission succeeded");
        break;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Waypoint mission aborted");
        break;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Waypoint mission canceled");
        break;

      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }

    if (!result.result->missed_waypoints.empty()) {
      std::string missed = "[";
      for (size_t i = 0; i < result.result->missed_waypoints.size(); ++i) {
        missed += std::to_string(result.result->missed_waypoints[i]);
        if (i + 1 < result.result->missed_waypoints.size()) {
          missed += ", ";
        }
      }
      missed += "]";
      RCLCPP_WARN(this->get_logger(), "Missed waypoint indices: %s", missed.c_str());
    }

    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowWaypointsClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
