#ifndef pico_hardware_interface_ARDUINO_COMMS_H
#define pico_hardware_interface_ARDUINO_COMMS_H

#include <serial/serial.h>
#include <cstring>
#include <iostream>
// #include <fstream>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <chrono>
#include <memory>
class ArduinoComms
{

public:
  // ArduinoComms()
  // {
  // }

  int setupPort(const std::string &serial_device);
  void reset(int fd);
  // void readEncoderValues(int fd, long &val_1, long &val_2);

  void readEncoderValues(std::string serial_device, int fd,
                         long &val_1, long &val_2,
                         double &gyrx, double &gyry, double &gyrz,
                         double &accx, double &accy, double &accz,
                         double &quatw, double &quatx, double &quaty, double &quatz);
  void setMotorValues(int fd, int val_1, int val_2);
  // void setPidValues(int fd, float k_p, float k_i, float k_d);
  void sendCharToPi(int fd, char character);
  // bool connected() const { return serial_conn_.isOpen(); }
  std::string readFromSerial(int fd);
  void sendMsg(int fd, const std::string &msg_to_send);

private:
  // serial::Serial serial_conn_; ///< Underlying serial connection
};

#endif // pico_hardware_interface_ARDUINO_COMMS_H