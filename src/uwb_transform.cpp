#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <example_interfaces/msg/float64.hpp>
#include <memory>
#include <chrono>
#include <thread>
#include <string>
#include <serial/serial.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <stdexcept>
#include <mutex>
#include <cmath>

using namespace std::chrono_literals;

class UWBTransform : public rclcpp::Node {
	public:
		UWBTransform() : Node("uwb_transform"),
						buffer_(this->get_clock()),
						listener_(buffer_) {
			subscription_anc1 = this->create_subscription<example_interfaces::msg::Float64>(
							"uwb/d_anc1", 10,
							[this](const example_interfaces::msg::Float64::SharedPtr msg) {this->common_anc_callback(1, msg);});
			subscription_anc2 = this->create_subscription<example_interfaces::msg::Float64>(
							"uwb/d_anc2", 10,
							[this](const example_interfaces::msg::Float64::SharedPtr msg) {this->common_anc_callback(2, msg);});
			subscription_anc3 = this->create_subscription<example_interfaces::msg::Float64>(
							"uwb/d_anc3", 10,
							[this](const example_interfaces::msg::Float64::SharedPtr msg) {this->common_anc_callback(3, msg);});
			publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("uwb/tag", 10);
			timer_ = this->create_wall_timer(50ms, std::bind(&UWBTransform::timer_callback, this));
			
			// anchor_tf 
			anc_tf.header.stamp = this->get_clock()->now();
			anc_tf.header.frame_id = "base_link";
			anc_tf.child_frame_id = "anchor_link";
			
			anc_tf.transform.translation.x = uwb_center_x;
			anc_tf.transform.translation.y = uwb_center_y;
			anc_tf.transform.translation.z = 0.0;
			
			anc_tf.transform.rotation.x = 0.0;
			anc_tf.transform.rotation.y = 0.0;
			anc_tf.transform.rotation.z = 0.0;
			anc_tf.transform.rotation.w = 1.0;
			
			static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
			static_broadcaster_->sendTransform(anc_tf);
			
			tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
			
			// This was done to just get the vertices for the three anchor positions on the rover. They were later adjusted to align (as much as we can) to the UGV center.
			//std::cout << computeTriangle(0.875, 0.86, 0.871) << std::endl;
			}
	
	private:
		std::mutex data_mutex;
		nav_msgs::msg::Odometry uwb_odom_msg;
		geometry_msgs::msg::TransformStamped anc_tf;
		geometry_msgs::msg::TransformStamped tag_tf;
		geometry_msgs::msg::TransformStamped base_tag_tf;
	
		double x1 = -0.4375, y1 = 0.3733615;     // left-front   // distance 1   // dtl-dtr = 87.5cm
		double x2 = 0.4375, y2 =  0.3733615;     // right-front  // distance 2	 // dtl-dbc = 86cm
		double x3 = -0.010881, y3 = -0.3733615;  // rear-center  // distance 0   // dtr-dbc = 87.1cm
		double d1 = 0.0, d2 = 0.0, d3 = 0.0;
		double uwb_center_x = (x1 + x2 + x3) / 3, uwb_center_y = (y1 + y2 + y3) / 3;
		
		Eigen::Quaterniond q;
		
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
		
		rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr subscription_anc1;
		rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr subscription_anc2;
		rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr subscription_anc3;
		
		Eigen::Matrix2d A; 
		Eigen::Vector2d b, pos;
		
		std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
		std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
		
		tf2_ros::Buffer buffer_;
		tf2_ros::TransformListener listener_;
		
		void common_anc_callback(int id, const example_interfaces::msg::Float64::SharedPtr msg) {
			std::lock_guard<std::mutex> lock(data_mutex);
			if (id == 1) d1 = msg->data;
			if (id == 2) d2 = msg->data;
			if (id == 3) d3 = msg->data;
			}
			
		Eigen::Vector2d calculatePosition() {
			A << 2*(x2-x1), 2*(y2-y1),
				 2*(x3-x1), 2*(y3-y1);

			b << (x2*x2 - x1*x1) + (y2*y2 - y1*y1) + (d1*d1 - d2*d2),
				 (x3*x3 - x1*x1) + (y3*y3 - y1*y1) + (d1*d1 - d3*d3);

			
			return A.colPivHouseholderQr().solve(b);
			}
		
		Eigen::Quaterniond calculateYaw(double x, double y) {
			Eigen::AngleAxisd rollAngle(0.0,   Eigen::Vector3d::UnitX());
			Eigen::AngleAxisd pitchAngle(0.0, Eigen::Vector3d::UnitY());
			Eigen::AngleAxisd yawAngle(atan2(y, x), Eigen::Vector3d::UnitZ());
			
			return yawAngle * pitchAngle * rollAngle;
			}
			
		Eigen::Matrix<double, 3, 2> computeTriangle(double a, double b, double c) {
			if (a <= 0 || b <= 0 || c <= 0) {
				throw std::invalid_argument("Side lengths must be positive.");
			}

			// Check triangle inequality
			if (a + b <= c || a + c <= b || b + c <= a) {
				throw std::invalid_argument("Invalid triangle side lengths.");
			}

			Eigen::Matrix<double, 3, 2> vertices;

			// Fix first two vertices along the x-axis
			vertices(0,0) = 0.0;    vertices(0,1) = 0.0;   // V0
			vertices(1,0) = a;      vertices(1,1) = 0.0;   // V1

			// Solve for V2
			double x = (a*a + b*b - c*c) / (2*a);
			double y2 = b*b - x*x;

			if (y2 < 0) y2 = 0; // numerical safeguard
			double y = std::sqrt(y2);

			vertices(2,0) = x;
			vertices(2,1) = y;   // Choose +y (mirror possible at -y)

			return vertices;
			}
			
		void timer_callback() {
			std::lock_guard<std::mutex> lock(data_mutex);
			
			std::vector<geometry_msgs::msg::TransformStamped> transforms;
			pos = this->calculatePosition();
			q = this->calculateYaw(pos[0], pos[1]);
			
			uwb_odom_msg.header.stamp = this->get_clock()->now();
			tag_tf.header.stamp = uwb_odom_msg.header.stamp;
			base_tag_tf.header.stamp = uwb_odom_msg.header.stamp;
			
			uwb_odom_msg.header.frame_id = "anchor_link";
			uwb_odom_msg.child_frame_id  = "tag_link";
			tag_tf.header.frame_id = "anchor_link";
			tag_tf.child_frame_id = "tag_link";
			base_tag_tf.header.frame_id = "base_link";
			base_tag_tf.child_frame_id = "tag_link";
			
			// Translation
			uwb_odom_msg.pose.pose.position.x = pos[0];
			uwb_odom_msg.pose.pose.position.y = pos[1];
			uwb_odom_msg.pose.pose.position.z = 0.0;
			tag_tf.transform.translation.x = pos[0];
			tag_tf.transform.translation.y = pos[1];
			tag_tf.transform.translation.z = 0;
			
			// Tag orientation wrt the uwb anchor positions (x1,y1),(x2,y2) and (x3,y3)
			uwb_odom_msg.pose.pose.orientation.x = q.x();
			uwb_odom_msg.pose.pose.orientation.y = q.y();
			uwb_odom_msg.pose.pose.orientation.z = q.z();
			uwb_odom_msg.pose.pose.orientation.w = q.w();
			tag_tf.transform.rotation = uwb_odom_msg.pose.pose.orientation;
			
			uwb_odom_msg.twist.twist.linear.x = 0;
			uwb_odom_msg.twist.twist.linear.y = 0;
			uwb_odom_msg.twist.twist.linear.z = 0;
			
			uwb_odom_msg.pose.covariance = {
					0.04, 0, 0, 0, 0, 0,
					0, 0.04, 0, 0, 0, 0,
					0, 0, 99999, 0, 0, 0,
					0, 0, 0, 99999, 0, 0,
					0, 0, 0, 0, 99999, 0,
					0, 0, 0, 0, 0, 99999
				};
			uwb_odom_msg.twist.covariance = {
					99999, 0, 0, 0, 0, 0,
					0, 99999, 0, 0, 0, 0,
					0, 0, 99999, 0, 0, 0,
					0, 0, 0, 99999, 0, 0,
					0, 0, 0, 0, 99999, 0,
					0, 0, 0, 0, 0, 99999
				};
			
			publisher_->publish(uwb_odom_msg);
			tf_broadcaster_->sendTransform(tag_tf);
			
			try {
				auto base_tag_lookup = buffer_.lookupTransform(
						"base_link",
						"tag_link",
						rclcpp::Time(0)
						);
							
				base_tag_tf.transform.translation.x = base_tag_lookup.transform.translation.x;
				base_tag_tf.transform.translation.y = base_tag_lookup.transform.translation.y;
				base_tag_tf.transform.translation.z = base_tag_lookup.transform.translation.z;
				base_tag_tf.transform.rotation = base_tag_lookup.transform.rotation;
				
				tf_broadcaster_->sendTransform(base_tag_tf);
			}
			 catch(const tf2::TransformException& ex) {
				 RCLCPP_WARN(get_logger(), "TF not ready: %s", ex.what());
				 }
		}
	};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<UWBTransform>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
	return 0;
	}
