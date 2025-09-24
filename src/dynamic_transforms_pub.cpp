#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>

using namespace std::chrono_literals;

class DynamicTFPublisher : public rclcpp::Node {
	public:
		DynamicTFPublisher() : Node("dynamic_tf_publisher") {
			tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
			subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("scout/odom", 10,
								[this](const nav_msgs::msg::Odometry::SharedPtr msg) {this->odom_callback(msg);});
			timer_ = this->create_wall_timer(50ms, std::bind(&DynamicTFPublisher::broadcast, this));
			}
		
	private:
		std::mutex data_mutex;
		
		void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
			std::lock_guard<std::mutex> lock(data_mutex);
			odom_ = *msg;
			}
	
		void broadcast() {
			std::lock_guard<std::mutex> lock(data_mutex);
			t.header.stamp = this->get_clock()->now();
			t.header.frame_id = "laser";
			t.child_frame_id = "odom";
			
			t.transform.translation.x = odom_.pose.pose.position.x;
			t.transform.translation.y = odom_.pose.pose.position.y;
			t.transform.translation.z = 0.0;

			t.transform.rotation.x = odom_.pose.pose.orientation.x;
			t.transform.rotation.y = odom_.pose.pose.orientation.y;
			t.transform.rotation.z = odom_.pose.pose.orientation.z;
			t.transform.rotation.w = odom_.pose.pose.orientation.w;

			tf_broadcaster_->sendTransform(t);
			}
	
		std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
		rclcpp::TimerBase::SharedPtr timer_;
		geometry_msgs::msg::TransformStamped t;
		nav_msgs::msg::Odometry odom_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
	};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<DynamicTFPublisher>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
	return 0;
	}
