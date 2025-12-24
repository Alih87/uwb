#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <thread>
#include <mutex>

using namespace std::chrono_literals;

class EKFTransform : public rclcpp::Node {
public:
    EKFTransform() : Node("ekf_transform")
    {
		qos_odom.best_effort();
		qos_odom.durability_volatile();
		
        uwb_dynamic_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "uwb/dynamic_filtered", 5,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){ this->dynamic_callback(msg); });

        uwb_static_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "uwb/static_filtered", 5,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){ this->static_callback(msg); });
            
        logs_pub_ = this->create_publisher<example_interfaces::msg::Float64MultiArray>("/ekf/metrics", 2);
		logs.data.resize(9, 0.0);


        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		timer_ = this->create_wall_timer(50ms, std::bind(&EKFTransform::timer_callback, this));

		delta_timer_ = this->create_wall_timer(66ms, std::bind(&EKFTransform::time_delta, this));
    }

private:
    nav_msgs::msg::Odometry dynamic_odom_msg;
    nav_msgs::msg::Odometry static_odom_msg;
    
    tf2::Transform T_map_tag, T_odom_tag, T_map_odom;
    geometry_msgs::msg::TransformStamped T_map_odom_msg;
	
	rclcpp::QoS qos_odom{rclcpp::KeepLast(3)};
    rclcpp::TimerBase::SharedPtr timer_ ,delta_timer_;
    rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr logs_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr uwb_dynamic_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr uwb_static_sub;
    
    // "logs" variable logs all the control data which helps us tune the cavariance matrices
	// logs = [x_stat, y_stat, theta_stat, x_dyn, y_dyn, theta_dyn, ]
	example_interfaces::msg::Float64MultiArray logs;
    float x_dyn_prev = 0., x_stat_prev = 0., y_dyn_prev = 0., y_stat_prev = 0., x_tf_prev = 0., y_tf_prev = 0.;
	geometry_msgs::msg::Quaternion yaw_dyn_prev, yaw_stat_prev, yaw_tf_prev;
	rclcpp::Time t_prev = this->get_clock()->now();
	double delta_t = 0.;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::mutex mtx_;
    
    void time_delta() {
		std::lock_guard<std::mutex> lk(mtx_);
		rclcpp::Time t_= this->get_clock()->now();
		if (t_ >= rclcpp::Duration(0.066667, 0) + t_prev) {
			delta_t = (t_ - t_prev).seconds();
			t_prev = t_;
		}
	}

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
		float x_dyn, x_stat, y_dyn, y_stat, x_tf, y_tf;
		geometry_msgs::msg::Quaternion yaw_dyn, yaw_stat, yaw_tf;
		tf2::Quaternion q_, q_prev;
		
		{
			std::lock_guard<std::mutex> lk(mtx_);
			dyn = dynamic_odom_msg;
			stat = static_odom_msg;
			
			x_dyn = dyn.pose.pose.position.x;
			y_dyn = dyn.pose.pose.position.y;
			yaw_dyn = dyn.pose.pose.orientation;
			
			x_stat = stat.pose.pose.position.x;
			y_stat = stat.pose.pose.position.y;
			yaw_stat = stat.pose.pose.orientation;
		}

		tf2::fromMsg(static_odom_msg.pose.pose, T_map_tag);
		tf2::fromMsg(dynamic_odom_msg.pose.pose, T_odom_tag);

		T_map_odom = T_map_tag * T_odom_tag.inverse();
		
		//printTF2Transform(T_map_tag, "TF_map_tag");
		//printTF2Transform(T_odom_tag, "TF_odom_tag");
		
		T_map_odom_msg.header.stamp = this->now();				
		T_map_odom_msg.header.frame_id = "map_uwb";
		T_map_odom_msg.child_frame_id  = "odom_uwb";

		geometry_msgs::msg::Transform TF_map_odom;
		tf2::convert(T_map_odom, TF_map_odom);

		T_map_odom_msg.transform = TF_map_odom;
		
		x_tf = T_map_odom_msg.transform.translation.x;
		y_tf = T_map_odom_msg.transform.translation.y;
		yaw_tf = T_map_odom_msg.transform.rotation;

		tf_broadcaster_->sendTransform(T_map_odom_msg);
		
		if (delta_t >= 0.066667 && std::fabs(x_dyn - x_dyn_prev) > 0) {
			logs.data[0] = (x_dyn - x_dyn_prev) / delta_t;
			logs.data[1] = (y_dyn - y_dyn_prev) / delta_t;
			tf2::fromMsg(yaw_dyn, q_);
			tf2::fromMsg(yaw_dyn_prev, q_prev);
			logs.data[2] = q_.angleShortestPath(q_prev) / delta_t;
			x_dyn_prev = x_dyn;
			y_dyn_prev = y_dyn;
			yaw_dyn_prev = yaw_dyn;
		}
		if (delta_t >= 0.066667 && std::fabs(x_stat - x_stat_prev) > 0) {
			logs.data[3] = (x_stat - x_stat_prev) / delta_t;
			logs.data[4] = (y_stat - y_stat_prev) / delta_t;
			tf2::fromMsg(yaw_stat, q_);
			tf2::fromMsg(yaw_stat_prev, q_prev);
			logs.data[5] = q_.angleShortestPath(q_prev) / delta_t;
			x_stat_prev = x_stat;
			y_stat_prev = y_stat;
			yaw_stat_prev = yaw_stat;
		}
		if (delta_t >= 0.066667 && std::fabs(x_tf - x_tf_prev) > 0) {
			logs.data[6] = (x_tf - x_tf_prev) / delta_t;
			logs.data[7] = (y_tf - y_tf_prev) / delta_t;
			tf2::fromMsg(yaw_tf, q_);
			tf2::fromMsg(yaw_tf_prev, q_prev);
			logs.data[8] = q_.angleShortestPath(q_prev) / delta_t;
			x_tf_prev = x_tf;
			y_tf_prev = y_tf;
			yaw_tf_prev = yaw_tf;
		}
		logs_pub_->publish(logs);
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
