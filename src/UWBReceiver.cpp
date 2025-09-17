#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <cstring>
#include <string>
#include <arpa/inet.h>
#include <unistd.h>
#include <example_interfaces/msg/float64.hpp>

using namespace std::chrono_literals;

#define PORT 5005
#define BUF_SIZE 1024

int sockfd;
char buffer[BUF_SIZE];
struct sockaddr_in server_addr, client_addr;

class UWBRcv : public rclcpp::Node {
	public:
		UWBRcv() : Node("uwb_rcv") {
			publisher_anc1 = this->create_publisher<example_interfaces::msg::Float64>("uwb/d_anc1", 10);
			publisher_anc2 = this->create_publisher<example_interfaces::msg::Float64>("uwb/d_anc2", 10);
			publisher_anc3 = this->create_publisher<example_interfaces::msg::Float64>("uwb/d_anc3", 10);
			timer_ = this->create_wall_timer(150us, std::bind(&UWBRcv::timer_callback, this));
			
			if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
				perror("Socket Creation Failed!\n");
				}
		
			memset(&server_addr, 0, sizeof(server_addr));
			memset(&client_addr, 0, sizeof(client_addr));
			
			server_addr.sin_family = AF_INET;
			server_addr.sin_addr.s_addr = INADDR_ANY;
			server_addr.sin_port = htons(PORT);
			
			if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
				perror("bind failed");
				close(sockfd);
				}
				
			std::cout << "Listening for UDP packets on port " << PORT << "...\n";
			}
			
	private:
		template <typename T>
		void print_vector(const std::vector<T>& vec) {
			for (const T& i : vec) {
				std::cout << i << "\n";
			}
		}
		
		std::vector<std::string> split(std::string s, const std::string& delimeter) {
			std::vector<std::string> tokens;
			size_t pos = 0;
			std::string token;
			
			while ((pos = s.find(delimeter)) != std::string::npos) {
				token = s.substr(0, pos);
				tokens.push_back(token);
				s.erase(0, pos + delimeter.length());
				}
				tokens.push_back(s);
				
			return tokens;
			}
		
		void timer_callback() {			
			socklen_t len = sizeof(client_addr);
			int n = recvfrom(sockfd, (char *)buffer, BUF_SIZE-1,
							MSG_WAITALL, (struct sockaddr *)&client_addr, &len);
			
			if (n < 0) {
				perror("recvfrom failed");
				}
			
			std::vector<std::string> lines = split(std::string(buffer), "\n");
			std::vector<float> float_dist;
			for (size_t i=0; i < lines.size(); i++) {
				std::vector<std::string> distances = split(lines[i], ":");
				float_dist.push_back(std::stof(distances[1].substr(0, distances[1].size() - 1)));
				if (i == 2) {
					break;
					}
				}
			//print_vector(float_dist);
			example_interfaces::msg::Float64 msg;

			msg.data = float_dist[0];
			publisher_anc1->publish(msg);

			msg.data = float_dist[1];
			publisher_anc2->publish(msg);

			msg.data = float_dist[2];
			publisher_anc3->publish(msg);
			
			buffer[n] = '\0';
			//std::cout << std::string(buffer) << std::endl;
				}
	
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc1;
		rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc2;
		rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_anc3;
	};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UWBRcv>());
	close(sockfd);
	rclcpp::shutdown();
	return 0;
}
