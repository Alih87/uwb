#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <chrono>
#include <thread>
#include <string>
#include <serial/serial.h>
#include <example_interfaces/msg/float64.hpp>

using namespace std::chrono_literals;

class UWBSerialComm : public serial::Serial {
	public:
		UWBSerialComm(const std::string& port, uint32_t baudrate, uint32_t timeout_ms)
		: serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(timeout_ms)) {}
	
	private:
		
		void _open_port(std::string& port, uint32_t baud_rate, uint32_t timeout) {
			this->setPort(port);
			this->setBaudrate(baud_rate);
			
			auto t = serial::Timeout::simpleTimeout(timeout);
			this->setTimeout(t);
			
			try {
				this->open();
				std::this_thread::sleep_for(std::chrono::microseconds(50));
				this->setDTR(false);
				this->setRTS(false);
				std::this_thread::sleep_for(std::chrono::microseconds(50));
				this->setDTR(true);
				this->setRTS(true);
				} catch (const serial::IOException& e) {
					std::cerr << "Unable to open port: " << e.what() << std::endl;
					}
			if (!this->isOpen()) {
				std::cerr << "Serial Port not opened! (" << port << ")\n"; 
				}
			}

		void print_vector(std::vector<std::string> vec) {
			for (const std::string& i : vec) {
				std::cout << i.c_str() << "\n";
				}
			}

		std::string getMessageString(std::vector<std::string> vec) {
			std::string complete = "";
			for (const std::string& i : vec) {
				complete = complete + i;
				}
			return complete;
			}
			
		std::vector<std::string> talk(std::vector<std::string> msg) {
			std::string line;
			std::vector<std::string> reply;
			if (msg.size() < 1) {
				return reply;
				}
			for (const std::string& m : msg) {
				this->write(m + "\r\n");
				std::this_thread::sleep_for(std::chrono::microseconds(5000));
				
				while(true) {
					if (this->available()) {
						std::vector<std::string> resp = this->readlines();
						if (!resp.empty()) {
							line = getMessageString(resp);
							reply.push_back(line);
							break;
							}
						}
					else {
						break;
						}
					}
				}
				return reply;
			}

		// Variable Declarations
		friend class UWB;
};

class UWB : public rclcpp::Node {
	public:
		UWB(int argc, char** argv) : Node("uwb_node"),
				serial_comm(_port, _baudrate, _timeout) {
					
			std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);	
			publisher_ = this->create_publisher<example_interfaces::msg::Float64>("Distance", 10);
			timer_ = this->create_wall_timer(150us, std::bind(&UWB::timer_callback, this));
			 
			_baudrate = static_cast<uint32_t>(std::stoul(args[1]));
			_timeout = static_cast<uint32_t>(std::stoul(args[2]));
			_port = args[3];

			serial_comm._open_port(_port, _baudrate, _timeout);
		}
		
		friend class UWBSerialComm;

	private:
		void timer_callback() {
			auto distance = example_interfaces::msg::Float64();
			//std::vector<std::string> msgs = {"AT+anchor_tag=1,2","AT+RST"};
			std::vector<std::string> msgs = {"AT"};
			
			std::vector<std::string> reply = serial_comm.talk(msgs);
			if (reply.size() > 0) {
				serial_comm.print_vector(reply);
				}
			}
		
		// Variable Declarations
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_;
		std::string _port;
		uint32_t _baudrate=0, _timeout=0;
		
		UWBSerialComm serial_comm;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UWB>(argc, argv));
	rclcpp::shutdown();
	return 0;
}
