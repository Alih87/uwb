#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/float64.hpp>
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
#define BUF_SIZE 1024

std::unordered_map<std::string, sockaddr_in> esp_clients_;
static constexpr int ESP_CMD_PORT = 5006;

class UWBRcv : public rclcpp::Node {
public:
  UWBRcv()
  : Node("uwb_rcv")
  {
    qos_anc.best_effort();
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
    timer_ = this->create_wall_timer(1ms, std::bind(&UWBRcv::receive_loop, this));
  }

  ~UWBRcv() override { close(sockfd_); }

private:
  void receive_loop()
  {
    sockaddr_in client_addr{};
    socklen_t len = sizeof(client_addr);
    char buffer[BUF_SIZE];

    // Non-blocking read: MSG_DONTWAIT prevents blocking if no data
    int n = recvfrom(sockfd_, buffer, BUF_SIZE - 1, MSG_DONTWAIT,
                     (struct sockaddr *)&client_addr, &len);

    while (n > 0) {
      buffer[n] = '\0';
      std::string msg(buffer);
      //RCLCPP_INFO(this->get_logger(), "Received: %s", msg.c_str());
	  
	  //RCLCPP_INFO(this->get_logger(), "Received message: %s", msg.c_str());
	  remember_esp_from_msg(msg, client_addr);
      parse_and_publish(msg);

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
        esp_status = msg.substr(colon + 1, end - (colon + 1));

        // Trim any unwanted whitespace or newline characters
        auto trim = [](std::string &s){
            while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == '\t')) s.pop_back();
            while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.erase(s.begin());
        };
        trim(esp_addr);
        trim(esp_status);

        RCLCPP_INFO(this->get_logger(), "Parsed: ESP_ADDR=%s, ESP_STATUS=%s", esp_addr.c_str(), esp_status.c_str());

        if (esp_status == "FINISHED") {
            if (esp_addr == "10") send_command_to_esp("20", "START");
            else if (esp_addr == "20") send_command_to_esp("10", "START");
            else RCLCPP_WARN(this->get_logger(), "Unknown ESP address: %s", esp_addr.c_str());
            return;
        }

        // Handle other statuses
        if (esp_status == "START" || esp_status == "STOP" || esp_status == "HELLO") {
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
        example_interfaces::msg::Float64 out;
        out.data = val;
        
        if (esp_addr == "10" && esp_status == "START") {
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
		} else if (esp_addr == "20" && esp_status == "START") {
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

  int sockfd_;
  rclcpp::QoS qos_anc{rclcpp::KeepLast(3)};
  rclcpp::TimerBase::SharedPtr timer_;
  
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
  
  std::string tag1_frame, tag2_frame, esp_addr, esp_status;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UWBRcv>());
  rclcpp::shutdown();
  return 0;
}
