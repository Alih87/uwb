#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>

using namespace std::chrono_literals;

class EKFTransform : public rclcpp::Node {
public:
    EKFTransform() : Node("ekf_transform")
    {
        uwb_dynamic_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "uwb/dynamic_filtered", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){ this->dynamic_callback(msg); });

        uwb_static_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "uwb/static_filtered", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){ this->static_callback(msg); });

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		timer_ = this->create_wall_timer(50ms, std::bind(&EKFTransform::timer_callback, this));
    }

private:
    // state
    nav_msgs::msg::Odometry dynamic_odom_msg;
    nav_msgs::msg::Odometry static_odom_msg;
    
    tf2::Transform T_map_tag, T_odom_tag, T_map_odom;
    geometry_msgs::msg::TransformStamped T_map_odom_msg, T_odom_tag_msg;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr uwb_dynamic_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr uwb_static_sub;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::mutex mtx_;

    void static_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        static_odom_msg = *msg;
    }

    void dynamic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        dynamic_odom_msg = *msg;
    }

    void timer_callback() {
		nav_msgs::msg::Odometry dyn, stat;
		{
			std::lock_guard<std::mutex> lk(mtx_);
			dyn = dynamic_odom_msg;
			stat = static_odom_msg;
		}

		tf2::fromMsg(static_odom_msg.pose.pose, T_map_tag);
		tf2::fromMsg(dynamic_odom_msg.pose.pose, T_odom_tag);

		// compose: T_map_odom = T_map_tag * T_tag_odom
		T_map_odom = T_map_tag * T_odom_tag.inverse();
		
		// publish map->odom_uwb
		T_map_odom_msg.header.stamp = this->now();
		T_odom_tag_msg.header.stamp = T_map_odom_msg.header.stamp;
		
		T_map_odom_msg.header.frame_id = "map";
		T_map_odom_msg.child_frame_id  = "odom_uwb";
		T_odom_tag_msg.header.frame_id = "odom_uwb";
		T_odom_tag_msg.child_frame_id  = "tag_link";
		
		geometry_msgs::msg::Transform TF_map_odom, TF_odom_tag;
		tf2::convert(T_map_odom, TF_map_odom);
		tf2::convert(T_odom_tag, TF_odom_tag);

		T_map_odom_msg.transform = TF_map_odom;
		T_odom_tag_msg.transform = TF_odom_tag;

		tf_broadcaster_->sendTransform(T_map_odom_msg);
		tf_broadcaster_->sendTransform(T_odom_tag_msg);
	}

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKFTransform>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
