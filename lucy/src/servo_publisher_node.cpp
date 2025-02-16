// #include <iostream>
// #include <chrono>
// #include <memory>
// #include <string>
// #include <cmath>
// #include "SCServo.h"
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "tf2_ros/transform_broadcaster.h"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "geometry_msgs/msg/point_stamped.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"

// using namespace std::chrono_literals;

// SMS_STS sm_st;
// unsigned char ID[] = {1};
// uint8_t rxPacket[4];
// int16_t Position;
// int16_t Speed;

// class ServoPublisher : public rclcpp::Node
// {
// public:
// 	ServoPublisher(std::string serial_port_servo)
// 		: Node("servo_publisher"), serial_port_servo_(serial_port_servo)
// 	{
// 		publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

// 		this->declare_parameter<std::string>("serial_port_servo", "/dev/ttyUSB0");
// 		this->get_parameter<std::string>("serial_port_servo", serial_port_servo_);
// 		this->declare_parameter<bool>("enable_navigation", false);
// 		if (!sm_st.begin(1000000, serial_port_servo_.c_str()))
// 		{
// 			RCLCPP_ERROR(this->get_logger(), "Servo init: FAILED at %s", serial_port_servo_.c_str());
// 		}
// 		else
// 		{
// 			RCLCPP_INFO(this->get_logger(), "Servo init: SUCCESS at %s", serial_port_servo_.c_str());
// 		}
// 		sm_st.syncReadBegin(sizeof(ID), sizeof(rxPacket));
// 		sm_st.WritePosEx(ID_, P3, speed, acc);
// 		usleep(764 * 1000);

// 		servo_control_thread_ = std::thread(&ServoPublisher::servoControlLoop, this);

// 		timer_ = this->create_wall_timer(
// 			40ms,
// 			std::bind(&ServoPublisher::readAndPublishServoData, this));
// 	}
// 	~ServoPublisher()
// 	{

// 		if (servo_control_thread_.joinable())
// 		{
// 			servo_control_thread_.join();
// 		}
// 		sm_st.syncReadEnd();
// 		sm_st.end();
// 	}

// private:
// 	std::string serial_port_servo_;
// 	rclcpp::TimerBase::SharedPtr timer_;
// 	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
// 	std::thread servo_control_thread_;
// 	int ID_{1};
// 	int16_t P1{511};  // 682
// 	int16_t P2{1534}; // 1364
// 	int16_t P3{1023};
// 	int16_t speed{600};
// 	int acc{10};

// 	void servoControlLoop()
// 	{
// 		bool enable_nav;
// 		this->get_parameter("enable_navigation", enable_nav);
// 		if (enable_nav)
// 		{
// 			while (rclcpp::ok())
// 			{
// 				sm_st.WritePosEx(ID_, P1, speed, acc);
// 				std::this_thread::sleep_for(std::chrono::milliseconds(2305));
// 				sm_st.WritePosEx(ID_, P2, speed, acc);
// 				std::this_thread::sleep_for(std::chrono::milliseconds(2305));
// 			}
// 		}
// 	}
// 	double mapRange(int input, int in_min, int in_max, double out_min, double out_max)
// 	{
// 		return ((input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min) + M_PI / 2;
// 	}

// 	void readAndPublishServoData()
// 	{
// 		sm_st.syncReadPacketTx(ID, sizeof(ID), SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket));

// 		for (uint8_t i = 0; i < sizeof(ID); i++)
// 		{
// 			if (!sm_st.syncReadPacketRx(ID[i], rxPacket))
// 			{
// 				RCLCPP_WARN(this->get_logger(), "ID: %d sync read error!", (int)ID[i]);
// 				continue;
// 			}

// 			Position = sm_st.syncReadRxPacketToWrod(15);
// 			Speed = sm_st.syncReadRxPacketToWrod(15);

// 			auto joint_state_msg = sensor_msgs::msg::JointState();

// 			if (Position <= 2047)
// 				joint_state_msg.position.push_back(mapRange(Position, 0, 2047, 0, -M_PI));

// 			else
// 				joint_state_msg.position.push_back(mapRange(Position, 2048, 4095, M_PI, 0));

// 			joint_state_msg.name.push_back("base_lidar_joint");
// 			joint_state_msg.header.stamp = this->get_clock()->now();

// 			publisher_->publish(joint_state_msg);
// 		}
// 	}
// };

// int main(int argc, char *argv[])
// {

// 	if (argc < 2)
// 	{
// 		std::cout << "Usage: " << argv[0] << " <serial_port_servo>" << std::endl;
// 		return 0;
// 	}

// 	std::string serial_port_servo = argv[1];
// 	std::cout << "serial:" << argv[1] << std::endl;

// 	rclcpp::init(argc, argv);

// 	try
// 	{
// 		rclcpp::spin(std::make_shared<ServoPublisher>(serial_port_servo));
// 	}
// 	catch (const std::exception &e)
// 	{
// 		std::cerr << "Error: " << e.what() << std::endl;
// 	}
// 	sm_st.WritePosEx(1, 1023, 2400, 50);
// 	rclcpp::shutdown();
// 	return 0;
// }