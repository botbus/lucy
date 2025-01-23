#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>

class JoystickVibrationNode : public rclcpp::Node
{
public:
    JoystickVibrationNode() : Node("joystick_vibration_node")
    {
        // Open joystick device
        joystick_device_ = "/dev/input/event5"; // Replace "eventX" with your actual device (e.g., /dev/input/event3)
        device_fd_ = open(joystick_device_.c_str(), O_RDWR);

        if (device_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open joystick device: %s", joystick_device_.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Joystick device opened: %s", joystick_device_.c_str());

        // Define vibration pattern
        vibration_pattern_ = {
            {20000, 100}, // Strength (0-65535), Duration (ms)
            {0, 600},
            {20000, 100}};

        triggerVibration();
    }

    ~JoystickVibrationNode()
    {
        if (device_fd_ >= 0)
        {
            close(device_fd_);
        }
    }

private:
    void triggerVibration()
    {
        for (const auto &vibration : vibration_pattern_)
        {
            int strength = vibration.first;
            int duration = vibration.second;

            setVibration(strength);
            std::this_thread::sleep_for(std::chrono::milliseconds(duration));
        }

        // Turn off vibration
        setVibration(0);
    }

    void setVibration(int strength)
    {
        struct ff_effect effect;
        memset(&effect, 0, sizeof(effect));

        effect.type = FF_RUMBLE;
        effect.id = -1; // New effect
        effect.u.rumble.strong_magnitude = strength;
        effect.u.rumble.weak_magnitude = strength / 2;
        effect.replay.length = 1000; // Length in ms
        effect.replay.delay = 0;

        if (ioctl(device_fd_, EVIOCSFF, &effect) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set force feedback effect");
            return;
        }

        struct input_event play;
        memset(&play, 0, sizeof(play));
        play.type = EV_FF;
        play.code = effect.id;
        play.value = 1;

        if (write(device_fd_, &play, sizeof(play)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to play force feedback effect");
        }
    }

    std::string joystick_device_;
    int device_fd_;
    std::vector<std::pair<int, int>> vibration_pattern_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickVibrationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
