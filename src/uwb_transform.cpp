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
#include <optional>

using namespace std::chrono_literals;

class UWBTransform : public rclcpp::Node {
	public:
		UWBTransform() : Node("uwb_transform"),
						buffer_(this->get_clock()),
						listener_(buffer_) {
			// Initialize declared parameters in launch file
			this->declare_parameter<std::string>("anc0");
			this->declare_parameter<std::string>("anc1");
			this->declare_parameter<std::string>("anc2");
			this->declare_parameter<std::string>("anc3");
			this->declare_parameter<std::string>("anc4");
			
			std::string anc0_loc = this->get_parameter("anc0").as_string();
			std::string anc1_loc = this->get_parameter("anc1").as_string();
			std::string anc2_loc = this->get_parameter("anc2").as_string();
			std::string anc3_loc = this->get_parameter("anc3").as_string();
			std::string anc4_loc = this->get_parameter("anc4").as_string();
			
			std::tie(x1, y1) = parse_parameter(anc0_loc);
			std::tie(x2, y2) = parse_parameter(anc1_loc);
			std::tie(x3, y3) = parse_parameter(anc2_loc);
			std::tie(x4, y4) = parse_parameter(anc3_loc);
			std::tie(x5, y5) = parse_parameter(anc4_loc);
			
			auto dynamic_points = std::make_shared<std::vector<std::pair<double, double>>>();
			auto static_points = std::make_shared<std::vector<std::pair<double, double>>>();
			static_points->push_back({x1,y1});
			dynamic_points->push_back({x2,y2});
			dynamic_points->push_back({x3,y3});
			static_points->push_back({x4,y4});
			static_points->push_back({x5,y5});
			
			auto sum_x = std::make_shared<double>(0.0);
			auto sum_y = std::make_shared<double>(0.0);
			auto count = std::make_shared<size_t>(0);
			
			for (const auto& [x,y] : *dynamic_points) {
				if (x == 0.0 && y == 0.0) {
					continue;
					} else {
						*sum_x += x;
						*sum_y += y;
						(*count)++;
						}
				}
				if (*count == 0) {
					dynamic_center_x = 0.0;
					dynamic_center_y = 0.0;
					} else {
						dynamic_center_x = *sum_x / *count;
						dynamic_center_y = *sum_y / *count;
						}
			
			*sum_x = 0.0;
			*sum_y = 0.0;
			*count = 0;
			
			for (const auto& [x,y] : *static_points) {
				if (x == 0.0 && y == 0.0) {
					continue;
					} else {
						*sum_x += x;
						*sum_y += y;
						(*count)++;
						}
				}
				if (*count == 0) {
					static_center_x = 0.0;
					static_center_y = 0.0;
					} else {
						static_center_x = *sum_x / *count;
						static_center_y = *sum_y / *count;
						}
			
			// Initialize subscriptions
			subscription_anc1 = this->create_subscription<example_interfaces::msg::Float64>(
							"uwb/d_anc0", 10,
							[this](const example_interfaces::msg::Float64::SharedPtr msg) {this->common_anc_callback(1, msg);});
			subscription_anc2 = this->create_subscription<example_interfaces::msg::Float64>(
							"uwb/d_anc1", 10,
							[this](const example_interfaces::msg::Float64::SharedPtr msg) {this->common_anc_callback(2, msg);});
			subscription_anc3 = this->create_subscription<example_interfaces::msg::Float64>(
							"uwb/d_anc2", 10,
							[this](const example_interfaces::msg::Float64::SharedPtr msg) {this->common_anc_callback(3, msg);});
			subscription_anc4 = this->create_subscription<example_interfaces::msg::Float64>(
							"uwb/d_anc3", 10,
							[this](const example_interfaces::msg::Float64::SharedPtr msg) {this->common_anc_callback(4, msg);});
			subscription_anc5 = this->create_subscription<example_interfaces::msg::Float64>(
							"uwb/d_anc4", 10,
							[this](const example_interfaces::msg::Float64::SharedPtr msg) {this->common_anc_callback(5, msg);});
			publisher_dynamic = this->create_publisher<nav_msgs::msg::Odometry>("uwb/dynamic_odom", 10);
			publisher_static = this->create_publisher<nav_msgs::msg::Odometry>("uwb/static_odom", 10);
			timer_ = this->create_wall_timer(50ms, std::bind(&UWBTransform::timer_callback, this));
			
			// dynamic anchor_tf 
			dynamic_anc_tf.header.stamp = this->get_clock()->now();
			dynamic_anc_tf.header.frame_id = "base_link";
			dynamic_anc_tf.child_frame_id = "dynamic_anc_link";
			
			dynamic_anc_tf.transform.translation.x = dynamic_center_x;
			dynamic_anc_tf.transform.translation.y = dynamic_center_y;
			dynamic_anc_tf.transform.translation.z = 0.0;
			
			dynamic_anc_tf.transform.rotation.x = 0.0;
			dynamic_anc_tf.transform.rotation.y = 0.0;
			dynamic_anc_tf.transform.rotation.z = 1.0;
			dynamic_anc_tf.transform.rotation.w = 0.0;
			
			dynamic_anc_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
			dynamic_anc_broadcaster_->sendTransform(dynamic_anc_tf);
			
			// static anchor_tf 
			static_anc_tf.header.stamp = this->get_clock()->now();
			static_anc_tf.header.frame_id = "static_anc_link";
			static_anc_tf.child_frame_id = "odom";
			
			static_anc_tf.transform.translation.x = -static_center_x;
			static_anc_tf.transform.translation.y = static_center_y;
			static_anc_tf.transform.translation.z = 0.0;
			
			static_anc_tf.transform.rotation.x = 0.0;
			static_anc_tf.transform.rotation.y = -1.0;
			static_anc_tf.transform.rotation.z = 0.0;
			static_anc_tf.transform.rotation.w = 0.0;
			
			static_anc_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
			static_anc_broadcaster_->sendTransform(static_anc_tf);
			
			tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
			
			std::thread dynamic_task(&UWBTransform::dynamic_localization, this);
			std::thread static_task(&UWBTransform::static_localization, this);

			dynamic_task.detach();
			static_task.detach();
			}
	
	private:
		std::mutex data_mutex;
		nav_msgs::msg::Odometry dynamic_odom_msg;
		nav_msgs::msg::Odometry static_odom_msg;
		geometry_msgs::msg::TransformStamped static_anc_tf;
		geometry_msgs::msg::TransformStamped dynamic_anc_tf;
		geometry_msgs::msg::TransformStamped dynamic_tag_tf;
		geometry_msgs::msg::TransformStamped static_tag_tf;
		geometry_msgs::msg::TransformStamped base_tag_tf;
		geometry_msgs::msg::TransformStamped inter_frame_tf;
		
		double x_dynamic = 0.0, y_dynamic = 0.0;
		double x_static = 0.0, y_static = 0.0;
		double x1 = 0.0, y1 = 0.0;
		double x2 = 0.0, y2 = 0.0;
		//double x1 = -0.4375, y1 = 0.3733615;     // left-front   // distance 1   // dtl-dtr = 87.5cm
		//double x2 = 0.4375, y2 =  0.3733615;     // right-front  // distance 2	 // dtl-dbc = 86cm
		//double x3 = -0.010881, y3 = -0.3733615;  // rear-center  // distance 0   // dtr-dbc = 87.1cm
		double x3 = 0.0, y3 = 0.0;
		double x4 = 0.0, y4 = 0.0;
		double x5 = 0.0, y5 = 0.0; 
		double d1 = 0.0, d2 = 0.0, d3 = 0.0, d4 = 0.0, d5 = 0.0;
		double d2_prev = 0.0, d3_prev = 0.0;
		//double uwb_center_x = (x1 + x2 + x3) / 3, uwb_center_y = (y1 + y2 + y3) / 3;
		double static_center_x = 0.0, static_center_y = 0.0;
		double dynamic_center_x = 0.0, dynamic_center_y = 0.0;
		Eigen::Quaterniond q_dynamic, q_static;
		std::optional<int> sign;
		std::optional<int> sign_prev;
		
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_dynamic, publisher_static;
		rclcpp::TimerBase::SharedPtr timer_;
		
		rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr subscription_anc1;
		rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr subscription_anc2;
		rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr subscription_anc3;
		rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr subscription_anc4;
		rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr subscription_anc5;
		
		Eigen::Vector2d pos_static;				// In case of 3-point ranging
		std::vector<Eigen::Vector2d> pos_dynamic;	// In case of 2-point ranging
		
		std::shared_ptr<tf2_ros::StaticTransformBroadcaster> dynamic_anc_broadcaster_;
		std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_anc_broadcaster_;
		std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
		
		tf2_ros::Buffer buffer_;
		tf2_ros::TransformListener listener_;
		
		void common_anc_callback(int id, const example_interfaces::msg::Float64::SharedPtr msg) {
			std::lock_guard<std::mutex> lock(data_mutex);
			if (id == 1) d1 = msg->data;
			if (id == 2) d2 = msg->data;
			if (id == 3) d3 = msg->data;
			if (id == 4) d4 = msg->data;
			if (id == 5) d5 = msg->data;
			}
		
		std::optional<int> pickInitialSignEigen(
				const Eigen::Vector2d& A1,   // anchor 1 position
				const Eigen::Vector2d& A2,   // anchor 2 position
				const Eigen::Vector2d& candAbs, // candidate (x, |y|) at t0 (in anchor frame)
				double r1_0, double r2_0,    // ranges at t0
				double r1_1, double r2_1,    // ranges at t1
				double max_motion = 1.5      // reject if implied motion > 2 m
				)
			{
				Eigen::Vector2d dR(r1_1 - r1_0, r2_1 - r2_0);
				int best_sign = 0;
				double best_err = std::numeric_limits<double>::max();

				for (int sgn : {+1, -1}) {
					Eigen::Vector2d p0(candAbs.x(), sgn * candAbs.y());

					// Unit line-of-sight vectors to each anchor
					Eigen::Vector2d u1 = p0 - A1;
					Eigen::Vector2d u2 = p0 - A2;
					double n1 = u1.norm(), n2 = u2.norm();
					if (n1 < 1e-6 || n2 < 1e-6) continue;
					u1 /= n1; u2 /= n2;

					// Build Jacobian H (2x2)
					Eigen::Matrix2d H;
					H.row(0) = u1.transpose();
					H.row(1) = u2.transpose();

					// Solve Δp = (HᵀH)⁻¹ Hᵀ Δr  (least-squares)
					Eigen::Vector2d dP = (H.transpose() * H).ldlt().solve(H.transpose() * dR);

					// Predicted Δr and residual
					Eigen::Vector2d res = dR - H * dP;
					double err = res.squaredNorm();

					// Penalize unrealistic motion magnitude
					if (dP.norm() > max_motion)
						err += 1e6; // large penalty

					if (err < best_err) {
						best_err = err;
						best_sign = sgn;
					}
				}

				if (best_sign == 0)
					return std::nullopt; // couldn’t decide

				return best_sign;
			}
		
		std::pair<double, double> parse_parameter(const std::string &msg) {
			// Expect messages like "x,y"
			size_t comma = msg.find(',');
			if (comma == std::string::npos) return std::make_pair(0.0, 0.0);

			std::string x = msg.substr(0, comma);
			std::string y = msg.substr(comma + 1);
			
			return std::make_pair(std::stod(x), std::stod(y));
		  }
			
		Eigen::Vector2d calculate_3Point(const Eigen::Vector2d& A1, const Eigen::Vector2d& A2, const Eigen::Vector2d& A3,
										 double d1, double d2, double d3) {
			Eigen::Matrix2d A;
			Eigen::Vector2d b;
			A << 2*(A2[0]-A1[0]), 2*(A2[1]-A1[1]),
				 2*(A3[0]-A1[0]), 2*(A3[1]-A1[1]);

			b << (A2[0]*A2[0] - A1[0]*A1[0]) + (A2[1]*A2[1] - A1[1]*A1[1]) + (d1*d1 - d2*d2),
				 (A3[0]*A3[0] - A1[0]*A1[0]) + (A3[1]*A3[1] - A1[1]*A1[1]) + (d1*d1 - d3*d3);

			return A.colPivHouseholderQr().solve(b);
			}
		
		std::vector<Eigen::Vector2d> calculate_2Point(const Eigen::Vector2d& A1, const Eigen::Vector2d& A2,
													  double d1, double d2) {
			std::vector<Eigen::Vector2d> result;
			Eigen::Vector2d delta = A2 - A1;
			double D = delta.norm();
			
			if (D < 1e-9) {
				std::cerr << "Anchors are at the same position - invalid configuration\n";
				return result;
				}
			if ((D > d1+d2 || D < std::fabs(d1-d2)) && (d1 != 0.0 && d2 != 0.0)) {
				std::cerr << "Circles do not intersect - no valid tag position\n";
				return result;
				}
			
			double a = (d1*d1 - d2*d2 + D*D) / (2.0 * D);
			double h2 = d1*d1 - a*a;
			if (h2 < 0) h2 = 0;
			double h = std::sqrt(h2);
			
			Eigen::Vector2d P2 = A1 + (a / D) * delta;
			Eigen::Vector2d perp(-delta.y(), delta.x());
			perp.normalize();
			
			Eigen::Vector2d P1 = P2 + h * perp;
			Eigen::Vector2d P3 = P2 - h * perp;
			
			result.push_back(P1);
			if (h > 1e-9) {
				result.push_back(P3);
				}
			
			return result;
			}
		
		Eigen::Quaterniond calculateYaw(double x, double y) {
			Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());
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

			if (y2 < 0) y2 = 0;  // numerical safeguard
			double y = std::sqrt(y2);

			vertices(2,0) = x;
			vertices(2,1) = y;   // Choose +y (mirror possible at -y)

			return vertices;
			}
			
		void dynamic_localization() {
			while (true) {
				std::lock_guard<std::mutex> lock(data_mutex);

				// 1. Compute new candidate positions first
				pos_dynamic = this->calculate_2Point(
					Eigen::Vector2d(x2, y2),
					Eigen::Vector2d(x3, y3),
					d2, d3
				);

				// Extract raw candidate (positive branch)
				Eigen::Vector2d candidate_raw(pos_dynamic[0][0], pos_dynamic[0][1]);

				bool first_frame = (std::fabs(d2_prev - d2) == 0.0) ||
								   (std::fabs(d3_prev - d3) == 0.0);

				if (!first_frame) {
					// 2. Pick sign using the NEW candidate
					sign = this->pickInitialSignEigen(
						Eigen::Vector2d(x2, y2),
						Eigen::Vector2d(x3, y3),
						Eigen::Vector2d(std::fabs(candidate_raw.x()), std::fabs(candidate_raw.y())),
						d2_prev, d3_prev, d2, d3,
						2.5
					);

					// Fallback to previous sign if failed
					if (!sign && sign_prev)
						sign = sign_prev;

					// Update previous sign if valid
					if (sign && (*sign != 0))
						sign_prev = sign;
				}

				// Update prev ranges
				d2_prev = d2;
				d3_prev = d3;

				// 3. Select correct branch
				if (pos_dynamic.size() > 1 && sign) {
					if (*sign > 0) {
						x_dynamic = pos_dynamic[0][0];
						y_dynamic = pos_dynamic[0][1];
					} else {
						x_dynamic = pos_dynamic[1][0];
						y_dynamic = pos_dynamic[1][1];
					}
				}

				// 4. Compute yaw
				q_dynamic = this->calculateYaw(x_dynamic, y_dynamic);

				std::this_thread::sleep_for(5ms);
			}
		}

			
		void static_localization() {
				while (true) {
					std::lock_guard<std::mutex> lock(data_mutex);
					pos_static = this->calculate_3Point(Eigen::Vector2d(x1,y1), Eigen::Vector2d(x4,y4), Eigen::Vector2d(x5,y5), d1, d4, d5);
					x_static = pos_static[0];
					y_static = pos_static[1];
					q_static = this->calculateYaw(x_static, y_static);
					std::this_thread::sleep_for(5ms);
				}
			}
			
		void timer_callback() {
			std::lock_guard<std::mutex> lock(data_mutex);
			//std::vector<geometry_msgs::msg::TransformStamped> transforms;
			
			dynamic_odom_msg.header.stamp = this->get_clock()->now();
			static_odom_msg.header.stamp = dynamic_odom_msg.header.stamp;
			dynamic_tag_tf.header.stamp = dynamic_odom_msg.header.stamp;
			static_tag_tf.header.stamp = dynamic_odom_msg.header.stamp;
			base_tag_tf.header.stamp = dynamic_odom_msg.header.stamp;
			inter_frame_tf.header.stamp = dynamic_odom_msg.header.stamp;
			
			dynamic_odom_msg.header.frame_id = "dynamic_anc_link";
			dynamic_odom_msg.child_frame_id  = "tag_link";
			dynamic_tag_tf.header.frame_id = "dynamic_anc_link";
			dynamic_tag_tf.child_frame_id = "tag_link";
			static_tag_tf.header.frame_id = "static_anc_link";
			static_tag_tf.child_frame_id = "tag_link";
			//inter_frame_tf.header.frame_id = "static_anc_link";
			//inter_frame_tf.child_frame_id = "dynamic_anc_link";
			
			// Translation
			dynamic_odom_msg.pose.pose.position.x = x_dynamic;
			dynamic_odom_msg.pose.pose.position.y = y_dynamic;
			dynamic_odom_msg.pose.pose.position.z = 0.0;
			static_odom_msg.pose.pose.position.x = x_static;
			static_odom_msg.pose.pose.position.y = y_static;
			static_odom_msg.pose.pose.position.z = 0.0;
			
			dynamic_tag_tf.transform.translation.x = x_dynamic;
			dynamic_tag_tf.transform.translation.y = y_dynamic;
			dynamic_tag_tf.transform.translation.z = 0;
			static_tag_tf.transform.translation.x = x_static;
			static_tag_tf.transform.translation.y = y_static;
			static_tag_tf.transform.translation.z = 0;
			
			// Tag orientation wrt the uwb anchor positions (x1,y1),(x2,y2) and (x3,y3)
			dynamic_odom_msg.pose.pose.orientation.x = q_dynamic.x();
			dynamic_odom_msg.pose.pose.orientation.y = q_dynamic.y();
			dynamic_odom_msg.pose.pose.orientation.z = q_dynamic.z();
			dynamic_odom_msg.pose.pose.orientation.w = q_dynamic.w();
			static_odom_msg.pose.pose.orientation.x = q_static.x();
			static_odom_msg.pose.pose.orientation.y = q_static.y();
			static_odom_msg.pose.pose.orientation.z = q_static.z();
			static_odom_msg.pose.pose.orientation.w = q_static.w();
			
			dynamic_tag_tf.transform.rotation = dynamic_odom_msg.pose.pose.orientation;
			static_tag_tf.transform.rotation = static_odom_msg.pose.pose.orientation;
			
			dynamic_odom_msg.twist.twist.linear.x = 0;
			dynamic_odom_msg.twist.twist.linear.y = 0;
			dynamic_odom_msg.twist.twist.linear.z = 0;
			static_odom_msg.twist.twist.linear.x = 0;
			static_odom_msg.twist.twist.linear.y = 0;
			static_odom_msg.twist.twist.linear.z = 0;
			
			dynamic_odom_msg.pose.covariance = {
					0.04, 0, 0, 0, 0, 0,
					0, 0.04, 0, 0, 0, 0,
					0, 0, 99999, 0, 0, 0,
					0, 0, 0, 99999, 0, 0,
					0, 0, 0, 0, 99999, 0,
					0, 0, 0, 0, 0, 99999
				};
			dynamic_odom_msg.twist.covariance = {
					99999, 0, 0, 0, 0, 0,
					0, 99999, 0, 0, 0, 0,
					0, 0, 99999, 0, 0, 0,
					0, 0, 0, 99999, 0, 0,
					0, 0, 0, 0, 99999, 0,
					0, 0, 0, 0, 0, 99999
				};
		
			static_odom_msg.pose.covariance = dynamic_odom_msg.pose.covariance;
			static_odom_msg.twist.covariance = dynamic_odom_msg.twist.covariance;
			
			publisher_dynamic->publish(dynamic_odom_msg);
			publisher_static->publish(static_odom_msg);
			tf_broadcaster_->sendTransform(dynamic_tag_tf);
			tf_broadcaster_->sendTransform(static_tag_tf);
			
			try {
				auto inter_frame_lookup = buffer_.lookupTransform(
						"static_anc_link",
						"dynamic_anc_link",
						rclcpp::Time(0)
						);
				auto base_tag_lookup = buffer_.lookupTransform(
						"base_link",
						"tag_link",
						rclcpp::Time(0)
						);
						
				inter_frame_tf.transform.translation.x = inter_frame_lookup.transform.translation.x;
				inter_frame_tf.transform.translation.y = inter_frame_lookup.transform.translation.y;
				inter_frame_tf.transform.translation.z = inter_frame_lookup.transform.translation.z;
				inter_frame_tf.transform.rotation = inter_frame_lookup.transform.rotation;
				
				base_tag_tf.transform.translation.x = base_tag_lookup.transform.translation.x;
				base_tag_tf.transform.translation.y = base_tag_lookup.transform.translation.y;
				base_tag_tf.transform.translation.z = base_tag_lookup.transform.translation.z;
				base_tag_tf.transform.rotation = base_tag_lookup.transform.rotation;
				
				tf_broadcaster_->sendTransform(inter_frame_tf);
				tf_broadcaster_->sendTransform(base_tag_tf);
			}
			 catch (const tf2::TransformException& ex) {
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
