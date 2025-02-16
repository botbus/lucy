// #include "rclcpp/rclcpp.hpp"           // Main ROS2 C++ library
// #include "geometry_msgs/msg/twist.hpp" // For Twist messages
// #include "geometry_msgs/msg/twist_stamped.hpp"
// #include <fcntl.h>
// #include <linux/input.h>
// #include <unistd.h>
// #include <iostream>
// #include <chrono>
// #include <thread>

// class JoystickVibrationNode : public rclcpp::Node
// {
// public:
//     JoystickVibrationNode() : Node("joystick_vibration_node")
//     {

//         subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
//             "/diff_drive_controller/cmd_vel_unstamped", 10,
//             std::bind(&JoystickVibrationNode::cmd_vel_callback, this, std::placeholders::_1));
//         joystick_device_ = "/dev/input/event5"; // Replace "eventX" with your actual device (e.g., /dev/input/event3)
//         device_fd_ = open(joystick_device_.c_str(), O_RDWR);

//         if (device_fd_ < 0)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Failed to open joystick device: %s", joystick_device_.c_str());
//             return;
//         }

//         RCLCPP_INFO(this->get_logger(), "Joystick device opened: %s", joystick_device_.c_str());
//     }

//     ~JoystickVibrationNode()
//     {
//         if (device_fd_ >= 0)
//         {
//             close(device_fd_);
//         }
//     }

// private:
//     void triggerVibration(long strength, int duration)
//     {
//         setVibration(strength);
//         std::this_thread::sleep_for(std::chrono::milliseconds(duration));
//         setVibration(strength);
//     }

//     void setVibration(long strength)
//     {
//         struct ff_effect effect;
//         memset(&effect, 0, sizeof(effect));
//         RCLCPP_DEBUG(this->get_logger(), "Strength received: %li", strength);
//         effect.type = FF_RUMBLE;
//         effect.id = -1; // New effect
//         effect.u.rumble.strong_magnitude = strength;
//         effect.u.rumble.weak_magnitude = strength / 2;
//         effect.replay.length = 250; // Length in ms
//         effect.replay.delay = 0;

//         if (ioctl(device_fd_, EVIOCSFF, &effect) < 0)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Failed to set force feedback effect");
//             return;
//         }

//         struct input_event play;
//         memset(&play, 0, sizeof(play));
//         play.type = EV_FF;
//         play.code = effect.id;
//         play.value = 1;

//         if (write(device_fd_, &play, sizeof(play)) < 0)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Failed to play force feedback effect");
//         }
//     }
//     void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
//     {
//         double direction = msg->angular.z;
//         RCLCPP_INFO(this->get_logger(), "Direction received: %f", direction);
//         if (direction > 0 && abs(direction) > 0.3)
//         {
//             RCLCPP_DEBUG(this->get_logger(), "Strength sent: %li", abs(static_cast<long>(direction * 65535)));
//             triggerVibration(abs(static_cast<long>(direction * 65535)), vibeRight);
//         }
//         else if (direction < 0 && abs(direction) > 0.3)
//         {
//             RCLCPP_DEBUG(this->get_logger(), "Strength sent: %li", abs(static_cast<long>(direction * 65535)));
//             triggerVibration(abs(static_cast<long>(direction * 65535)), vibeLeft);
//         }
//         else
//         {
//             triggerVibration(0, 0);
//         }
//     }

//     std::string joystick_device_;
//     int device_fd_;
//     std::vector<std::pair<int, int>> vibration_pattern_;
//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
//     int vibeLeft = 0;
//     int vibeRight = 500;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<JoystickVibrationNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
