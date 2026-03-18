#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <example_interfaces/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <unordered_map>
#include <sstream>

using namespace std::chrono_literals;

#define PORT 5005
#define BUF_SIZE 512

std::unordered_map<std::string, sockaddr_in> esp_clients_;
static constexpr int ESP_CMD_PORT = 5006;

class UWBRcv : public rclcpp::Node {
public:
  UWBRcv()
  : Node("uwb_rcv")
  {
    qos_anc.reliable();
	qos_anc.durability_volatile();
    
    this->declare_parameter<std::string>("tag1");
    this->declare_parameter<std::string>("tag2");
	tag1_frame = this->get_parameter("tag1").as_string();
	tag2_frame = this->get_parameter("tag2").as_string();

    publisher_anc1_t1 = this->create_publisher<example_interfaces::msg::Float64>("uwb/"+tag1_frame+"/d_anc0", qos_anc);
    publisher_anc2_t1 = this->create_publisher<example_interfaces::msg::Float64>("uwb/"+tag1_frame+"/d_anc1", qos_anc);
    publisher_anc3_t1 = this->create_publisher<example_interfaces::msg::Float64>("uwb/"+tag1_frame+"/d_anc2", qos_anc);
    publisher_anc4_t1 = this->create_publisher<example_interfaces::msg::Float64>("uwb/"+tag1_frame+"/d_anc3", qos_anc);
    publisher_anc5_t1 = this->create_publisher<example_interfaces::msg::Float64>("uwb/"+tag1_frame+"/d_anc4", qos_anc);
    
    publisher_anc1_t2 = this->create_publisher<example_interfaces::msg::Float64>("uwb/"+tag2_frame+"/d_anc0", qos_anc);
    publisher_anc2_t2 = this->create_publisher<example_interfaces::msg::Float64>("uwb/"+tag2_frame+"/d_anc1", qos_anc);
    publisher_anc3_t2 = this->create_publisher<example_interfaces::msg::Float64>("uwb/"+tag2_frame+"/d_anc2", qos_anc);
    publisher_anc4_t2 = this->create_publisher<example_interfaces::msg::Float64>("uwb/"+tag2_frame+"/d_anc3", qos_anc);
    publisher_anc5_t2 = this->create_publisher<example_interfaces::msg::Float64>("uwb/"+tag2_frame+"/d_anc4", qos_anc);
    
    tag1_imu = this->create_publisher<sensor_msgs::msg::Imu>("uwb/"+tag1_frame+"/imu_raw", qos_anc);
    tag2_imu = this->create_publisher<sensor_msgs::msg::Imu>("uwb/"+tag2_frame+"/imu_raw", qos_anc);

    // --- Create UDP socket ---
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0)
      RCLCPP_FATAL(this->get_logger(), "Socket creation failed");

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);
    if (bind(sockfd_, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
      RCLCPP_FATAL(this->get_logger(), "Socket bind failed");

    RCLCPP_INFO(this->get_logger(), "Listening for UDP packets on port %d...", PORT);

    // Run the callback fast enough to drain UDP buffer
    timer_ = this->create_wall_timer(2.5ms, std::bind(&UWBRcv::receive_loop, this));
    timer_imu = this->create_wall_timer(1.5ms, std::bind(&UWBRcv::imu_loop, this));
  }

  ~UWBRcv() override { close(sockfd_); }

private:
  void imu_loop() {
      std::string msg;
      {
          std::lock_guard<std::mutex> lk(data_mutex);
          msg = buffer;
      }

      if (esp_clients_.size() <= 1) {
          return;
      }

      size_t first_space = msg.find(' ');
      size_t second_space = (first_space != std::string::npos) ? msg.find(' ', first_space + 1) : std::string::npos;
      size_t end = msg.find('\r');

      if (first_space == std::string::npos ||
          second_space == std::string::npos ||
          end == std::string::npos) {
          return;
      }

      std::string address_part = msg.substr(0, first_space);
      std::string acc_part = msg.substr(first_space + 1, second_space - first_space - 1);
      std::string gyro_part = msg.substr(second_space + 1, end - second_space - 1);

      try {
          size_t c1 = acc_part.find(',');
          size_t c2 = (c1 != std::string::npos) ? acc_part.find(',', c1 + 1) : std::string::npos;
          if (c1 == std::string::npos || c2 == std::string::npos) return;

          float ax = std::stof(acc_part.substr(4, c1 - 4));
          float ay = std::stof(acc_part.substr(c1 + 1, c2 - c1 - 1));
          float az = std::stof(acc_part.substr(c2 + 1));

          c1 = gyro_part.find(',');
          c2 = (c1 != std::string::npos) ? gyro_part.find(',', c1 + 1) : std::string::npos;
          if (c1 == std::string::npos || c2 == std::string::npos) return;

          float gx = std::stof(gyro_part.substr(5, c1 - 5));
          float gy = std::stof(gyro_part.substr(c1 + 1, c2 - c1 - 1));
          float gz = std::stof(gyro_part.substr(c2 + 1));
			
          sensor_msgs::msg::Imu imu_msg;
          imu_msg.header.stamp = this->get_clock()->now();
          imu_msg.angular_velocity.x = gx;
          imu_msg.angular_velocity.y = gy;
          imu_msg.angular_velocity.z = gz;
          imu_msg.linear_acceleration.x = ax;
          imu_msg.linear_acceleration.y = ay;
          imu_msg.linear_acceleration.z = az;

          if (address_part.size() >= 10 && address_part.compare(8, 2, "10") == 0) {
            imu_msg.header.frame_id = tag1_frame + "_imu_link";
            imu_msg.angular_velocity_covariance = {
			  2.48696198e-07, 1.45460038e-07, 0.0,
			  1.45460038e-07, 2.36054903e-05, -1.74169755e-07,
			  -6.45012625e-08, 0.0, 3.0022835e-07
			  };
			  
			imu_msg.linear_acceleration_covariance = {
			  0.00089055, -0.00020792, -0.00035547,
			  -0.00020792,  0.0006797,   0.00016247,
			  -0.00035547,  0.00016247,  0.0010234
			  };
            tag1_imu->publish(imu_msg);
          } else if (address_part.size() >= 10 && address_part.compare(8, 2, "20") == 0) {
            imu_msg.header.frame_id = tag2_frame + "_imu_link";
            imu_msg.angular_velocity_covariance = {
			  8.40307611e-08,  5.53873141e-08, -1.52010842e-08,
			  5.53873141e-08,  1.23388406e-06, -1.33560758e-07,
			  -1.52010842e-08, -1.33560758e-07,  1.33460380e-06
			  };
			  
			imu_msg.linear_acceleration_covariance = {
			  7.46352656e-04, -1.09487665e-04, -1.99120637e-04,
			  -1.09487665e-04,  4.36112901e-04,  3.59955636e-05,
			  -1.99120637e-04,  3.59955636e-05,  7.75708276e-04
			  };
            tag2_imu->publish(imu_msg);
          }

    } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse IMU message: %s | raw: %s", e.what(), msg.c_str());
      }
  }
  
  void receive_loop() {
	std::lock_guard<std::mutex> lk(data_mutex);
    sockaddr_in client_addr{};
    socklen_t len = sizeof(client_addr);

    // Non-blocking read: MSG_DONTWAIT prevents blocking if no data
    int n = recvfrom(sockfd_, buffer, BUF_SIZE - 1, MSG_DONTWAIT,
                     (struct sockaddr *)&client_addr, &len);

    while (n > 0) { 
      buffer[n] = '\0';
      std::string msg(buffer);
      //RCLCPP_INFO(this->get_logger(), "Received: %s", msg.c_str());
	  //RCLCPP_INFO(this->get_logger(), "Received message: %s", msg.c_str());
	  
	  remember_esp_from_msg(msg, client_addr);
	  if (esp_clients_.size() > 1) {
		  parse_and_publish(msg);
	  }

      // try reading the next waiting packet
      n = recvfrom(sockfd_, buffer, BUF_SIZE - 1, MSG_DONTWAIT,
                   (struct sockaddr *)&client_addr, &len);
    }
  }
  
  std::string sockaddr_to_string(const sockaddr_in& addr)
	{
	  char ip[INET_ADDRSTRLEN];
	  inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip));

	  std::ostringstream oss;
	  oss << ip << ":" << ntohs(addr.sin_port);
	  return oss.str();
	}
  
  void remember_esp_from_msg(const std::string &msg, const sockaddr_in &client_addr) {
	  size_t dash  = msg.find('-');
	  size_t colon = msg.find(':');
	  if (dash == std::string::npos || colon == std::string::npos || colon <= dash)
		return;

	  std::string esp_id = msg.substr(dash + 1, colon - (dash + 1));
	  esp_clients_[esp_id] = client_addr;
	}
	
  void parse_and_publish(const std::string &msg) {
    size_t dash  = msg.find('-');
    size_t colon = msg.find(':');
    if (dash != std::string::npos && colon != std::string::npos && colon > dash + 1) {
        size_t end = msg.find('\r', colon + 1);
        if (end == std::string::npos) end = msg.find('\n', colon + 1);
        if (end == std::string::npos) end = msg.size();

        esp_addr = msg.substr(dash + 1, colon - (dash + 1));
        esp_status_rcv = msg.substr(colon + 1, end - (colon + 1));

        // Trim any unwanted whitespace or newline characters
        auto trim = [](std::string &s){
            while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == '\t')) s.pop_back();
            while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.erase(s.begin());
        };
        trim(esp_addr);
        trim(esp_status_rcv);

        RCLCPP_INFO(this->get_logger(), "Parsed: ESP_ADDR=%s, ESP_STATUS=%s", esp_addr.c_str(), esp_status_rcv.c_str());

        if (esp_status_rcv == "FINISHED") {
            if (esp_addr == "10") {
				send_command_to_esp("20", "START");
				esp_status_snd = "START";
				esp_addr = "20";
			}
            else if (esp_addr == "20") {
				send_command_to_esp("10", "START");
				esp_status_snd = "START";
				esp_addr = "10";
			}
            else RCLCPP_WARN(this->get_logger(), "Unknown ESP address: %s", esp_addr.c_str());
            return;
        }

        // Handle other statuses
        if (esp_status_snd == "START" || esp_status_snd == "STOP" || esp_status_rcv == "HELLO") {
            return;  // Skip distance processing for these control messages
        }
    }

    // Handle distance messages if they don't match the control message format
    size_t colon2 = msg.find(':');
    if (colon2 == std::string::npos) return;

    std::string id = msg.substr(0, colon2);
    if (id.rfind("distance", 0) != 0) return;
    std::string val_str = msg.substr(colon2 + 1);
    try {
        double val = std::stod(val_str);
        //std::cout << val << std::endl;
        example_interfaces::msg::Float64 out;
        out.data = val;
        //std::cout << esp_addr << std::endl;
        //std::cout << esp_status_rcv << std::endl;
        //std::cout << id << std::endl;
        if (esp_addr == "10" && esp_status_snd == "START") {
			if (id.find("0") != std::string::npos)
            publisher_anc1_t1->publish(out);
			else if (id.find("1") != std::string::npos)
				publisher_anc2_t1->publish(out);
			else if (id.find("2") != std::string::npos)
				publisher_anc3_t1->publish(out);
			else if (id.find("3") != std::string::npos)
				publisher_anc4_t1->publish(out);
			else if (id.find("4") != std::string::npos)
				publisher_anc5_t1->publish(out);
		} else if (esp_addr == "20" && esp_status_snd == "START") {
			if (id.find("0") != std::string::npos)
            publisher_anc1_t2->publish(out);
			else if (id.find("1") != std::string::npos)
				publisher_anc2_t2->publish(out);
			else if (id.find("2") != std::string::npos)
				publisher_anc3_t2->publish(out);
			else if (id.find("3") != std::string::npos)
				publisher_anc4_t2->publish(out);
			else if (id.find("4") != std::string::npos)
				publisher_anc5_t2->publish(out);
		} else {
			
		}
    } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Parse error on message: %s", msg.c_str());
    }
}

  void send_command_to_esp(const std::string& target_esp, const std::string& cmd) {
	  auto it = esp_clients_.find(target_esp);
	  if (it == esp_clients_.end()) {
		RCLCPP_WARN(this->get_logger(),
					"ESP %s not known yet (no packet received from it)",
					target_esp.c_str());
		return;
	  }

	  sockaddr_in dst = it->second;
	  dst.sin_port = htons(ESP_CMD_PORT);  // ESP listen port (5006)

	  std::string payload =
		  "ADDRESS-" + target_esp + ":" + cmd + "\n";

	  int rc = sendto(sockfd_,
					  payload.c_str(),
					  payload.size(),
					  0,
					  (struct sockaddr*)&dst,
					  sizeof(dst));

	  if (rc < 0) {
		RCLCPP_WARN(this->get_logger(),
					"sendto() failed for ESP %s",
					target_esp.c_str());
	  } else {
		char ipbuf[INET_ADDRSTRLEN];
		inet_ntop(AF_INET, &dst.sin_addr, ipbuf, sizeof(ipbuf));
		RCLCPP_INFO(this->get_logger(),
					"Sent to ESP %s @ %s:%d → %s",
					target_esp.c_str(),
					ipbuf,
					ntohs(dst.sin_port),
					payload.c_str());
	  }
	}

  std::mutex data_mutex;
  int sockfd_;
  char buffer[BUF_SIZE];
  rclcpp::QoS qos_anc{rclcpp::KeepLast(3)};
  rclcpp::TimerBase::SharedPtr timer_, timer_imu;
  
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc1_t1;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc2_t1;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc3_t1;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc4_t1;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc5_t1;
  
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc1_t2;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc2_t2;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc3_t2;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc4_t2;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc5_t2;
  
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr tag1_imu;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr tag2_imu;
  
  std::string tag1_frame, tag2_frame, esp_addr, esp_addr_curr, esp_status_rcv, esp_status_snd;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UWBRcv>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
