#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
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
    geometry_msgs::msg::TransformStamped T_map_odom_msg;

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
    
    void printTransform(const geometry_msgs::msg::Transform& tf_msg, const std::string& name = "")
	{
		tf2::Quaternion q(
			tf_msg.rotation.x,
			tf_msg.rotation.y,
			tf_msg.rotation.z,
			tf_msg.rotation.w
		);

		double roll, pitch, yaw;
		tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

		std::cout << "==== Transform: " << name << " ====\n";
		std::cout << "Translation:\n";
		std::cout << "  x: " << tf_msg.translation.x << "\n";
		std::cout << "  y: " << tf_msg.translation.y << "\n";
		std::cout << "  z: " << tf_msg.translation.z << "\n";

		std::cout << "Rotation (quaternion):\n";
		std::cout << "  x: " << tf_msg.rotation.x << "\n";
		std::cout << "  y: " << tf_msg.rotation.y << "\n";
		std::cout << "  z: " << tf_msg.rotation.z << "\n";
		std::cout << "  w: " << tf_msg.rotation.w << "\n";

		std::cout << "Rotation (RPY):\n";
		std::cout << "  roll:  " << roll  << "\n";
		std::cout << "  pitch: " << pitch << "\n";
		std::cout << "  yaw:   " << yaw   << "\n";
		std::cout << "=================================\n\n";
	}
	
	void printTF2Transform(const tf2::Transform& tf, const std::string& name = "")
	{
		const tf2::Vector3& t = tf.getOrigin();
		const tf2::Quaternion& q = tf.getRotation();

		double roll, pitch, yaw;
		tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

		std::cout << "==== tf2::Transform: " << name << " ====\n";

		std::cout << "Translation:\n";
		std::cout << "  x: " << t.x() << "\n";
		std::cout << "  y: " << t.y() << "\n";
		std::cout << "  z: " << t.z() << "\n";

		std::cout << "Rotation (quaternion):\n";
		std::cout << "  x: " << q.x() << "\n";
		std::cout << "  y: " << q.y() << "\n";
		std::cout << "  z: " << q.z() << "\n";
		std::cout << "  w: " << q.w() << "\n";

		std::cout << "Rotation (RPY):\n";
		std::cout << "  roll:  " << roll  << "\n";
		std::cout << "  pitch: " << pitch << "\n";
		std::cout << "  yaw:   " << yaw   << "\n";

		std::cout << "=====================================\n\n";
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

		T_map_odom = T_map_tag * T_odom_tag.inverse();
		
		//printTF2Transform(T_map_tag, "TF_map_tag");
		//printTF2Transform(T_odom_tag, "TF_odom_tag");
		
		T_map_odom_msg.header.stamp = this->now();				
		T_map_odom_msg.header.frame_id = "map";
		T_map_odom_msg.child_frame_id  = "odom_uwb";
				
		geometry_msgs::msg::Transform TF_map_odom;
		tf2::convert(T_map_odom, TF_map_odom);

		T_map_odom_msg.transform = TF_map_odom;

		tf_broadcaster_->sendTransform(T_map_odom_msg);
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
