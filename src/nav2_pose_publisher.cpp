#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <functional>
#include <fstream>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class FollowWaypointsClient : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  FollowWaypointsClient() : Node("follow_waypoints_client")
  {
    this->declare_parameter("planner_name", "unknown_planner");
    this->declare_parameter("controller_name", "unknown_controller");
    this->declare_parameter("run_id", 1);

    this->declare_parameter("waypoint_file", "");
    this->declare_parameter("input_frame", "utm");
    this->declare_parameter("goal_frame", "map");

    planner_name_ = this->get_parameter("planner_name").as_string();
    controller_name_ = this->get_parameter("controller_name").as_string();
    run_id_ = this->get_parameter("run_id").as_int();

    waypoint_file_ = this->get_parameter("waypoint_file").as_string();
    input_frame_ = this->get_parameter("input_frame").as_string();
    goal_frame_ = this->get_parameter("goal_frame").as_string();

    if (waypoint_file_.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Parameter 'waypoint_file' is empty. Please pass the txt file path.");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      std::bind(&FollowWaypointsClient::cmd_callback, this, std::placeholders::_1)
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/scout/odom_filtered",
      10,
      std::bind(&FollowWaypointsClient::odom_callback, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(
      1s,
      std::bind(&FollowWaypointsClient::send_goal_once, this)
    );
  }

private:
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Time start_time_;
  rclcpp::Time end_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::string planner_name_;
  std::string controller_name_;
  int run_id_;

  std::string waypoint_file_;
  std::string input_frame_;
  std::string goal_frame_;

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

  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  geometry_msgs::msg::PoseStamped makePose(
    double x,
    double y,
    double z,
    double yaw,
    const std::string & frame_id)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = this->now();

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation = yawToQuaternion(yaw);

    return pose;
  }

  bool transformPoseToGoalFrame(
    const geometry_msgs::msg::PoseStamped & input_pose,
    geometry_msgs::msg::PoseStamped & output_pose)
  {
    try {
      output_pose = tf_buffer_->transform(
        input_pose,
        goal_frame_,
        tf2::durationFromSec(1.0)
      );

      output_pose.header.stamp = this->now();
      output_pose.header.frame_id = goal_frame_;

      // Nav2 does not need altitude for 2D navigation.
      output_pose.pose.position.z = 0.0;

      return true;
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to transform waypoint from '%s' to '%s': %s",
        input_frame_.c_str(),
        goal_frame_.c_str(),
        ex.what()
      );
      return false;
    }
  }

  bool loadWaypointsFromFile(FollowWaypoints::Goal & goal_msg)
  {
    std::ifstream file(waypoint_file_);

    if (!file.is_open()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Cannot open waypoint file: %s",
        waypoint_file_.c_str()
      );
      return false;
    }

    std::string line;
    int loaded_count = 0;

    while (std::getline(file, line)) {
      if (line.empty()) {
        continue;
      }

      std::stringstream ss(line);

      std::string waypoint_id_str;
      double utm_easting = 0.0;
      double utm_northing = 0.0;
      double altitude = 0.0;
      double yaw_rad = 0.0;
      double yaw_deg = 0.0;

      ss >> waypoint_id_str;

      // Skip header line
      if (waypoint_id_str == "waypoint_id") {
        continue;
      }

      // Parse numeric columns
      ss >> utm_easting >> utm_northing >> altitude >> yaw_rad >> yaw_deg;

      if (ss.fail()) {
        RCLCPP_WARN(
          this->get_logger(),
          "Skipping invalid waypoint line: %s",
          line.c_str()
        );
        continue;
      }

      geometry_msgs::msg::PoseStamped utm_pose =
        makePose(utm_easting, utm_northing, altitude, yaw_rad, input_frame_);

      geometry_msgs::msg::PoseStamped map_pose;

      if (!transformPoseToGoalFrame(utm_pose, map_pose)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Could not transform waypoint ID %s. Aborting waypoint loading.",
          waypoint_id_str.c_str()
        );
        return false;
      }

      goal_msg.poses.push_back(map_pose);
      loaded_count++;

      RCLCPP_INFO(
        this->get_logger(),
        "Loaded waypoint %s: UTM(%.3f, %.3f) -> %s(%.3f, %.3f), yaw=%.3f rad",
        waypoint_id_str.c_str(),
        utm_easting,
        utm_northing,
        goal_frame_.c_str(),
        map_pose.pose.position.x,
        map_pose.pose.position.y,
        yaw_rad
      );
    }

    file.close();

    if (goal_msg.poses.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No valid waypoints loaded from file.");
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Successfully loaded %d waypoints from file.",
      loaded_count
    );

    return true;
  }

  void send_goal_once()
  {
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

    FollowWaypoints::Goal goal_msg;

    if (!loadWaypointsFromFile(goal_msg)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load waypoint file. Goal not sent.");
      rclcpp::shutdown();
      return;
    }

    goal_sent_ = true;
    timer_->cancel();

    RCLCPP_INFO(
      this->get_logger(),
      "Sending %zu waypoints to Nav2 in frame '%s'",
      goal_msg.poses.size(),
      goal_frame_.c_str()
    );

    auto send_goal_options =
      rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(
        &FollowWaypointsClient::goal_response_callback,
        this,
        std::placeholders::_1
      );

    send_goal_options.feedback_callback =
      std::bind(
        &FollowWaypointsClient::feedback_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      );

    send_goal_options.result_callback =
      std::bind(
        &FollowWaypointsClient::result_callback,
        this,
        std::placeholders::_1
      );

    start_time_ = this->now();
    client_->async_send_goal(goal_msg, send_goal_options);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
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

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
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

  void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
    }
  }

  void feedback_callback(
    GoalHandleFollowWaypoints::SharedPtr,
    const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Currently heading to waypoint index: %u",
      feedback->current_waypoint
    );
  }

  void result_callback(const GoalHandleFollowWaypoints::WrappedResult & result)
  {
    end_time_ = this->now();

    double total_time = (end_time_ - start_time_).seconds();
    double avg_smoothness =
      (cmd_samples_ > 0) ? smoothness_score_ / cmd_samples_ : 0.0;

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

    int success =
      (result.code == rclcpp_action::ResultCode::SUCCEEDED) ? 1 : 0;

    size_t missed_count = 0;
    if (result.result) {
      missed_count = result.result->missed_waypoints.size();
    }

    const std::string result_file = "nav2_eval_results.csv";

    std::ifstream infile(result_file);
    bool file_empty = !infile.good() ||
      infile.peek() == std::ifstream::traits_type::eof();
    infile.close();

    std::ofstream file(result_file, std::ios::app);

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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowWaypointsClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
