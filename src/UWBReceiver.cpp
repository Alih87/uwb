#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/float64.hpp>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <string>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

#define PORT 5005
#define BUF_SIZE 1024

class UWBRcv : public rclcpp::Node {
public:
  UWBRcv()
  : Node("uwb_rcv")
  {
    // Define QoS
    qos_anc.best_effort();
    qos_anc.durability_volatile();

    publisher_anc1_ = this->create_publisher<example_interfaces::msg::Float64>("uwb/d_anc0", qos_anc);
    publisher_anc2_ = this->create_publisher<example_interfaces::msg::Float64>("uwb/d_anc1", qos_anc);
    publisher_anc3_ = this->create_publisher<example_interfaces::msg::Float64>("uwb/d_anc2", qos_anc);
    publisher_anc4_ = this->create_publisher<example_interfaces::msg::Float64>("uwb/d_anc3", qos_anc);
    publisher_anc5_ = this->create_publisher<example_interfaces::msg::Float64>("uwb/d_anc4", qos_anc);

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

      parse_and_publish(msg);

      // try reading the next waiting packet
      n = recvfrom(sockfd_, buffer, BUF_SIZE - 1, MSG_DONTWAIT,
                   (struct sockaddr *)&client_addr, &len);
    }
  }

  void parse_and_publish(const std::string &msg) {
    size_t dash  = msg.find('-');
    size_t colon = msg.find(':');
    if (dash != std::string::npos && colon != std::string::npos && colon > dash + 1) {

        // Find the end of the status token
        size_t end = msg.find('\r', colon + 1);
        if (end == std::string::npos) end = msg.find('\n', colon + 1);
        if (end == std::string::npos) end = msg.size();

        // Extract the address and status
        std::string esp_addr   = msg.substr(dash + 1, colon - (dash + 1));   // "10" or "20"
        std::string esp_status = msg.substr(colon + 1, end - (colon + 1));   // "START" or "FINISHED"

        RCLCPP_INFO(this->get_logger(), "Parsed: ESP_ADDR=%s, ESP_STATUS=%s", esp_addr.c_str(), esp_status.c_str());

        if (esp_status == "FINISHED") {
            if (esp_addr == "10") {
                send_command_to_esp("20", "START");
            } else if (esp_addr == "20") {
                send_command_to_esp("10", "START");
            } else {
                std::cerr << "ERROR: Unknown ESP address: " << esp_addr << "\n";
            }
            return;  // Skip the rest if the control message is processed
        }
    }

    // Handle distance messages if they don't match the control message format
    size_t colon2 = msg.find(':');
    if (colon2 == std::string::npos) return;

    std::string id = msg.substr(0, colon2);
    std::string val_str = msg.substr(colon2 + 1);
    try {
        double val = std::stod(val_str);
        example_interfaces::msg::Float64 out;
        out.data = val;

        if (id.find("0") != std::string::npos)
            publisher_anc1_->publish(out);
        else if (id.find("1") != std::string::npos)
            publisher_anc2_->publish(out);
        else if (id.find("2") != std::string::npos)
            publisher_anc3_->publish(out);
        else if (id.find("3") != std::string::npos)
            publisher_anc4_->publish(out);
        else if (id.find("4") != std::string::npos)
            publisher_anc5_->publish(out);
    } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Parse error on message: %s", msg.c_str());
    }
}


  void send_command_to_esp(const std::string& target_esp, const std::string& cmd) {
    sockaddr_in target_addr{};
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(PORT);
    inet_pton(AF_INET, "192.168.0.100", &target_addr.sin_addr);  // Assume Jetson's IP

    std::string payload = "ADDRESS-" + target_esp + ":" + cmd + "\n";  // ESP code expects '\n'
    int rc = sendto(sockfd_, payload.c_str(), payload.size(), 0, 
                    (struct sockaddr*)&target_addr, sizeof(target_addr));
    if (rc < 0) {
      RCLCPP_WARN(this->get_logger(), "sendto() failed for ESP %s", target_esp.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Sent to ESP %s: %s", target_esp.c_str(), payload.c_str());
    }
  }

  int sockfd_;
  rclcpp::QoS qos_anc{rclcpp::KeepLast(3)};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc1_;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc2_;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc3_;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc4_;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc5_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UWBRcv>());
  rclcpp::shutdown();
  return 0;
}
