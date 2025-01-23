#ifndef pico_hardware_interface_REAL_ROBOT_H
#define pico_hardware_interface_REAL_ROBOT_H

#include <cstring>
#include "rclcpp/rclcpp.hpp"

// #include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "hardware_interface/types/hardware_interface_status_values.hpp"
#include <rclcpp_lifecycle/state.hpp>
#include "config.h"
#include "wheel.h"
#include "arduino_comms.h"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

// class PicoHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
// {
class PicoHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  Config cfg_;
  ArduinoComms arduino_;

  Wheel l_wheel_;
  Wheel r_wheel_;
  IMUSensor imu_;
  int serial_fd_;

  std::chrono::time_point<std::chrono::system_clock> time_;
};

#endif // pico_hardware_interface_REAL_ROBOT_H