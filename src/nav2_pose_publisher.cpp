#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <functional>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class FollowWaypointsClient : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  FollowWaypointsClient() : Node("follow_waypoints_client") {
	
	this->declare_parameter("planner_name", "unknown_planner");
	this->declare_parameter("controller_name", "unknown_controller");
	this->declare_parameter("run_id", 1);

	planner_name_ = this->get_parameter("planner_name").as_string();
	controller_name_ = this->get_parameter("controller_name").as_string();
	run_id_ = this->get_parameter("run_id").as_int(); 
	 
    client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
							std::bind(&FollowWaypointsClient::cmd_callback, this, std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
							"/scout/odom_filtered", 10, std::bind(&FollowWaypointsClient::odom_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
      1s, std::bind(&FollowWaypointsClient::send_goal_once, this));
  }

private:
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_;
  rclcpp::Time start_time_;
  rclcpp::Time end_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  std::string planner_name_;
  std::string controller_name_;
  int run_id_;
  
  bool first_cmd_{true};
  double smoothness_score_{0.0};
  double last_v_{0.0};
  double last_w_{0.0};
  int cmd_samples_{0};
  
  bool first_odom_{true};
  double total_distance_{0.0};
  double last_x_{0.0};
  double last_y_{0.0};
  
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
    
    total_distance_ = 0.0;
	smoothness_score_ = 0.0;
	cmd_samples_ = 0;
	first_cmd_ = true;
	first_odom_ = true;  

    if (!client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for follow_waypoints action server...");
      return;
    }

    goal_sent_ = true;
    timer_->cancel();

    FollowWaypoints::Goal goal_msg;

    goal_msg.poses.push_back(makePose(1.0905964374542236, 1.4392635822296143, 0.0));
    goal_msg.poses.push_back(makePose(2.794067144393921, 0.3269386291503906, -1.577));
    goal_msg.poses.push_back(makePose(1.4294044971466064, -0.0794987678527832, 3.14));
    goal_msg.poses.push_back(makePose(0.04559445381164551, 0.7019748687744141, 1.577));

    RCLCPP_INFO(this->get_logger(), "Sending %zu waypoints", goal_msg.poses.size());

    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&FollowWaypointsClient::goal_response_callback, this, std::placeholders::_1);

    send_goal_options.feedback_callback =
      std::bind(&FollowWaypointsClient::feedback_callback, this,
      std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback =
      std::bind(&FollowWaypointsClient::result_callback, this, std::placeholders::_1);

    start_time_ = this->now();
    client_->async_send_goal(goal_msg, send_goal_options);
  }
  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    if (first_odom_) {
      last_x_ = x;
      last_y_ = y;
      first_odom_ = false;
      return;
    }

    double dx = x - last_x_;
    double dy = y - last_y_;
    total_distance_ += std::sqrt(dx * dx + dy * dy);

    last_x_ = x;
    last_y_ = y;
  }
  
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double v = msg->linear.x;
    double w = msg->angular.z;

    if (first_cmd_) {
      last_v_ = v;
      last_w_ = w;
      first_cmd_ = false;
      return;
    }

    smoothness_score_ += std::abs(v - last_v_) + std::abs(w - last_w_);
    cmd_samples_++;

    last_v_ = v;
    last_w_ = w;
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
    end_time_ = this->now();
    double total_time = (end_time_ - start_time_).seconds();
    double avg_smoothness = (cmd_samples_ > 0) ? smoothness_score_ / cmd_samples_ : 0.0;

    std::string mission_status;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        mission_status = "SUCCEEDED";
        break;
      case rclcpp_action::ResultCode::ABORTED:
        mission_status = "ABORTED";
        break;
      case rclcpp_action::ResultCode::CANCELED:
        mission_status = "CANCELED";
        break;
      default:
        mission_status = "UNKNOWN";
        break;
    }
	
	int success = (result.code == rclcpp_action::ResultCode::SUCCEEDED) ? 1 : 0;
    size_t missed_count = result.result->missed_waypoints.size();
    
    std::ifstream infile("nav2_eval_results_"+planner_name_+".csv");
	bool file_empty = infile.peek() == std::ifstream::traits_type::eof();
	infile.close();

	std::ofstream file("nav2_eval_results.csv", std::ios::app);

	if (file_empty) {
	  file << "planner,controller,run_id,status,time_sec,distance_m,missed_waypoints,smoothness\n";
	}

	file << planner_name_ << ","
     << controller_name_ << ","
     << run_id_ << ","
     << success << ","
     << total_time << ","
     << total_distance_ << ","
     << missed_count << ","
     << avg_smoothness << "\n";

	file.close();

    RCLCPP_INFO(this->get_logger(), "===== MISSION SUMMARY =====");
    RCLCPP_INFO(this->get_logger(), "Status           : %s", mission_status.c_str());
    RCLCPP_INFO(this->get_logger(), "Total time (s)   : %.3f", total_time);
    RCLCPP_INFO(this->get_logger(), "Travel distance  : %.3f m", total_distance_);
    RCLCPP_INFO(this->get_logger(), "Missed waypoints : %zu", missed_count);
    RCLCPP_INFO(this->get_logger(), "Smoothness score : %.6f", avg_smoothness);

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
