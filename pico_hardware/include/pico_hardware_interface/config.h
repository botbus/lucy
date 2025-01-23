#ifndef pico_hardware_interface_CONFIG_H
#define pico_hardware_interface_CONFIG_H

#include <string>

struct Config
{
  std::string left_wheel_name = "left_wheel";
  std::string right_wheel_name = "right_wheel";
  float loop_rate = 30;
  std::string device = "/dev/ttyUSB0";
  long baud_rate = 115200;
  long timeout = 1000;
  long enc_counts_per_rev = 1920;
};

struct IMUSensorParams
{
  double w{};
  double x{};
  double y{};
  double z{};
};

struct IMUSensor
{
  std::string name = "imu_sensor";
  IMUSensorParams angular_velocity;
  IMUSensorParams linear_acceleration;
  IMUSensorParams orientation;
  // // Angular velocity
  // double angular_velocity_x = 0.0;
  // double angular_velocity_y = 0.0;
  // double angular_velocity_z = 0.0;

  // // Linear acceleration
  // double linear_acceleration_x = 0.0;
  // double linear_acceleration_y = 0.0;
  // double linear_acceleration_z = 0.0;

  // // Orientation (quaternion)
  // double orientation_x = 0.0;
  // double orientation_y = 0.0;
  // double orientation_z = 0.0;
  // double orientation_w = 1.0;
};
#endif // pico_hardware_interface_CONFIG_H