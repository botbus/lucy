#include "pico_hardware_interface/pico_hardware_interface.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

// PicoHardware::PicoHardware()
//     : PicoHardware(rclcpp::get_logger("PicoHardware"))
// {
// }
// namespace pico_hardware_interface
// {

CallbackReturn PicoHardware::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("PicoHardware"), "init...");

  l_wheel_.vel = std::numeric_limits<double>::quiet_NaN();
  l_wheel_.pos = std::numeric_limits<double>::quiet_NaN();
  r_wheel_.vel = std::numeric_limits<double>::quiet_NaN();
  r_wheel_.pos = std::numeric_limits<double>::quiet_NaN();

  imu_.angular_velocity.x = std::numeric_limits<double>::quiet_NaN();
  imu_.angular_velocity.y = std::numeric_limits<double>::quiet_NaN();
  imu_.angular_velocity.z = std::numeric_limits<double>::quiet_NaN();

  imu_.linear_acceleration.x = std::numeric_limits<double>::quiet_NaN();
  imu_.linear_acceleration.y = std::numeric_limits<double>::quiet_NaN();
  imu_.linear_acceleration.z = std::numeric_limits<double>::quiet_NaN();

  imu_.orientation.x = std::numeric_limits<double>::quiet_NaN();
  imu_.orientation.y = std::numeric_limits<double>::quiet_NaN();
  imu_.orientation.z = std::numeric_limits<double>::quiet_NaN();

  time_ = std::chrono::system_clock::now();

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  RCLCPP_DEBUG(rclcpp::get_logger("PicoHardware"), "left name: %s", cfg_.left_wheel_name.c_str());

  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  RCLCPP_DEBUG(rclcpp::get_logger("PicoHardware"), "right name: %s", cfg_.right_wheel_name.c_str());

  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  RCLCPP_DEBUG(rclcpp::get_logger("PicoHardware"), "loop: %f", cfg_.loop_rate);

  cfg_.device = info_.hardware_parameters["device"];
  RCLCPP_DEBUG(rclcpp::get_logger("PicoHardware"), "port: %s", cfg_.device.c_str());

  cfg_.enc_counts_per_rev = std::stol(info_.hardware_parameters["enc_counts_per_rev"]);
  RCLCPP_DEBUG(rclcpp::get_logger("PicoHardware"), "Finished enc");

  cfg_.baud_rate = std::stol(info_.hardware_parameters["baud_rate"]);
  RCLCPP_DEBUG(rclcpp::get_logger("PicoHardware"), "Finished baud");
  cfg_.timeout = std::stol(info_.hardware_parameters["timeout"]);
  RCLCPP_DEBUG(rclcpp::get_logger("PicoHardware"), "Finished timeout");

  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  serial_fd_ = arduino_.setupPort(cfg_.device);

  RCLCPP_DEBUG(rclcpp::get_logger("PicoHardware"), "Finished Configuration");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PicoHardware::export_state_interfaces()
{

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_.name, "angular_velocity.x", &imu_.angular_velocity.x));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_.name, "angular_velocity.y", &imu_.angular_velocity.y));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_.name, "angular_velocity.z", &imu_.angular_velocity.z));

  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_.name, "linear_acceleration.x", &imu_.linear_acceleration.x));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_.name, "linear_acceleration.y", &imu_.linear_acceleration.y));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_.name, "linear_acceleration.z", &imu_.linear_acceleration.z));

  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_.name, "orientation.w", &imu_.orientation.w));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_.name, "orientation.x", &imu_.orientation.x));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_.name, "orientation.y", &imu_.orientation.y));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_.name, "orientation.z", &imu_.orientation.z));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PicoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}

CallbackReturn PicoHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger("PicoHardware"), "Starting Controller...");

  arduino_.reset(serial_fd_);
  // arduino_.setPidValues(serial_fd_, 0.7, 0.6, 0.02);

  return CallbackReturn::SUCCESS;
}

CallbackReturn PicoHardware::on_deactivate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(rclcpp::get_logger("PicoHardware"), "stop");
  return CallbackReturn::SUCCESS;
}

return_type PicoHardware::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;

  arduino_.readEncoderValues(cfg_.device, serial_fd_,
                             l_wheel_.enc, r_wheel_.enc,
                             imu_.angular_velocity.x, imu_.angular_velocity.y, imu_.angular_velocity.z,
                             imu_.linear_acceleration.x, imu_.linear_acceleration.y, imu_.linear_acceleration.z,
                             imu_.orientation.w, imu_.orientation.x, imu_.orientation.y, imu_.orientation.z);

  double pos_prev = l_wheel_.pos;
  l_wheel_.pos = l_wheel_.calcEncAngle();
  l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = r_wheel_.pos;
  r_wheel_.pos = r_wheel_.calcEncAngle();
  r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;

  return return_type::OK;
}

return_type PicoHardware::write(
    const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  // double max_vel = 30;
  // double min_vel = -30;
  // std::vector<double> left_speeds;
  // std::vector<double> right_speeds;
  // left_speeds.push_back(l_wheel_.cmd);
  // right_speeds.push_back(r_wheel_.cmd);
  // if (left_speeds.size() > 10)
  // {
  //   left_speeds.erase(left_speeds.begin());
  // }
  // if (right_speeds.size() > 10)
  // {
  //   right_speeds.erase(right_speeds.begin());
  // }

  // double average_left = 0.0;
  // for (double input : left_speeds)
  // {
  //   average_left += input;
  // }

  // double average_right = 0.0;
  // for (double input : right_speeds)
  // {
  //   average_right += input;
  // }

  // average_left /= left_speeds.size();   // Get the average of the last 10 inputs
  // average_right /= right_speeds.size(); // Get the average of the last 10 inputs
  // if (abs(average_left) < 4 && abs(average_left) > 0.1)
  // {
  //   l_wheel_.cmd *= 10;
  // }
  // if (abs(average_right) < 4.0 && abs(average_right) > 0.1)
  // {
  //   r_wheel_.cmd *= 10;
  // }
  // if (l_wheel_.cmd < 0)
  //   l_wheel_.cmd = -max_vel;
  // else if (l_wheel_.cmd > 0)
  //   l_wheel_.cmd = max_vel;
  // if (r_wheel_.cmd < 0)
  //   r_wheel_.cmd = -max_vel;
  // else if (r_wheel_.cmd > 0)
  //   r_wheel_.cmd = max_vel;
  if (l_wheel_.cmd != 0 || r_wheel_.cmd != 0)
    RCLCPP_INFO(rclcpp::get_logger("PicoHardware"), "Left command: %f. Right command: %f", l_wheel_.cmd, r_wheel_.cmd);

  arduino_.setMotorValues(serial_fd_, l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate, r_wheel_.cmd / r_wheel_.rads_per_count / cfg_.loop_rate);

  return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(PicoHardware, hardware_interface::SystemInterface)
