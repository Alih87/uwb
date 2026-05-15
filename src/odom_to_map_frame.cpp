#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class GpsOdomToMapFrame : public rclcpp::Node
{
public:
  GpsOdomToMapFrame()
  : Node("gps_odom_to_map_frame")
  {
    this->declare_parameter<std::string>("input_topic", "/odometry/gps");
    this->declare_parameter<std::string>("output_topic", "/odometry/gps_map");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<std::string>("child_frame_id", "base_link");

    this->declare_parameter<double>("xy_variance_floor", 0.01);
    this->declare_parameter<double>("z_variance_floor", 0.04);

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    child_frame_id_ = this->get_parameter("child_frame_id").as_string();

    xy_variance_floor_ = this->get_parameter("xy_variance_floor").as_double();
    z_variance_floor_ = this->get_parameter("z_variance_floor").as_double();

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_topic_,
      10,
      std::bind(&GpsOdomToMapFrame::odomCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Republishing %s -> %s with frame_id=%s, child_frame_id=%s",
      input_topic_.c_str(),
      output_topic_.c_str(),
      frame_id_.c_str(),
      child_frame_id_.c_str()
    );
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto out = *msg;

    out.header.frame_id = frame_id_;
    out.child_frame_id = child_frame_id_;

    out.pose.covariance[0] = std::max(out.pose.covariance[0], xy_variance_floor_);
    out.pose.covariance[7] = std::max(out.pose.covariance[7], xy_variance_floor_);
    out.pose.covariance[14] = std::max(out.pose.covariance[14], z_variance_floor_);

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  std::string child_frame_id_;

  double xy_variance_floor_;
  double z_variance_floor_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsOdomToMapFrame>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
