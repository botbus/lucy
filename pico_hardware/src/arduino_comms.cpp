
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include "pico_hardware_interface/arduino_comms.h"
int ArduinoComms::setupPort(const std::string &port_name)
{
    // serial_conn_.setPort(serial_device);
    // serial_conn_.setBaudrate(baud_rate);
    // serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    // serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    // serial_conn_.open();
    // // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));
    RCLCPP_INFO(rclcpp::get_logger("PicoHardware"), "Configuring Serial Port");

    int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("Error opening serial port");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("Error getting terminal attributes");
        close(fd);
        return -1;
    }
    cfsetospeed(&tty, B115200); // Set output baud rate
    cfsetispeed(&tty, B115200);

    // Configure the terminal attributes
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;
    tty.c_cflag = B115200 | CS8 | CLOCAL | CREAD; // 115200 baud, 8 data bits, local, enable receiver
    tty.c_iflag = IGNPAR;                         // Ignore parity errors
    tty.c_oflag = 0;                              // Raw output
    tty.c_lflag = 0;                              // Raw input
    tty.c_cc[VMIN] = 1;                           // read doesn't block
    tty.c_cc[VTIME] = 1;                          // 0.1 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Error getting serial attributes");
        close(fd);
        return -1;
    }
    RCLCPP_INFO(rclcpp::get_logger("PicoHardware"), "Serial Port success");
    return fd;
}

void ArduinoComms::reset(int fd)
{
    sendMsg(fd, "r \n");
}

// void ArduinoComms::calcKalman(double &KalmanState, double &KalmanUncertainty, double KalmanMeasurement)
// {
//     constexpr double KalmanInput = 0.0;
//     constexpr double ProcessNoise = 10.0;
//     constexpr double NoiseCovariance = 20;
//     constexpr double StateTransition = 1;
//     constexpr double controlMatrix = 0.004;
//     constexpr double observationMatrix = 1;

//     // Predict phase
//     KalmanState = StateTransition * KalmanState + controlMatrix * KalmanInput;
//     KalmanUncertainty += ProcessNoise;

//     // Update phase
//     double KalmanGain = KalmanUncertainty * observationMatrix / (observationMatrix * KalmanUncertainty + NoiseCovariance);
//     KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - observationMatrix * KalmanState);
//     KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty + ProcessNoise;
//     RCLCPP_INFO(rclcpp::get_logger("kalman"), "Measurement: %.2f | State: %.2f | Uncertainty: %.2f",
//                 KalmanMeasurement, KalmanState, KalmanUncertainty);
// }

void ArduinoComms::readEncoderValues(std::string serial_device, int fd,
                                     long &val_1, long &val_2,
                                     double &gyrx, double &gyry, double &gyrz,
                                     double &accx, double &accy, double &accz,
                                     double &quatw, double &quatx, double &quaty, double &quatz)
{
    static long fullCount{};
    static long errorCount{};

    std::string inputString = readFromSerial(fd);
    const std::string delimiter = "|";
    size_t pos = 0;
    std::string target = "-inf";
    std::string replacement = "0.0";
    bool errorFound = false;
    while ((pos = inputString.find(target, pos)) != std::string::npos)
    {
        inputString.replace(pos, target.length(), replacement);
        pos += replacement.length();
        errorFound = true;
    }
    size_t start = inputString.find_first_of(delimiter, 0);
    size_t end = 0;
    RCLCPP_DEBUG(rclcpp::get_logger("ArduinoComms"), "Read from %s: %s", serial_device.c_str(), inputString.c_str());
    if (start != std::string::npos)
        start += 1;
    while (start < inputString.size())
    {
        std::string jsonString{};
        end = inputString.find_first_of(delimiter, start);

        if (end == std::string::npos)
        {
            break;
        }
        try
        {
            jsonString = inputString.substr(start, end - start);
        }
        catch (const std::exception &e)
        {
            std::cerr << "JSON substring error: " << e.what() << std::endl;
            std::cout << "\n\n\n"
                      << inputString << "\n\n\n";
            break;
        }

        start = end + delimiter.length();
        size_t len = jsonString.length();
        if (len <= 3)
            continue;
        try
        {
            json parsedJson = json::parse(jsonString);

            if (parsedJson.contains("GYR") && parsedJson["GYR"].is_array())
            {
                auto gyr = parsedJson["GYR"];
                gyrx = gyr[0];
                gyry = gyr[1];
                gyrz = gyr[2];
            }
            if (parsedJson.contains("ACC") && parsedJson["ACC"].is_array())
            {

                auto acc = parsedJson["ACC"];
                accx = acc[0];
                accy = acc[1];
                accz = acc[2];
            }
            if (parsedJson.contains("QUAT") && parsedJson["QUAT"].is_array())
            {
                auto quat = parsedJson["QUAT"];
                quatx = quat[1];
                quaty = quat[2];
                quatz = quat[3];
                if (!errorFound)
                    quatw = quat[0];
                else
                    quatw = sqrt(1.0 - ((quatx * quatx) + (quaty * quaty) + (quatz * quatz)));
            }

            if (parsedJson.contains("ENC") && parsedJson["ENC"].is_array())
            {
                auto enc = parsedJson["ENC"];
                val_1 = enc[0];
                val_2 = enc[1];
            }
            fullCount++;
        }
        catch (const json::exception &e)
        {
            RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "JSON parsing error: %s", e.what());
            RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "inputString: %s\njsonString:  %s\n", inputString.c_str(), jsonString.c_str());
            errorCount++;
        }

        start = end + 1;
    }
    if (fullCount >= 1000)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("ArduinoComms"), "Data loss: %f%", ((float)errorCount / (float)fullCount) * 100);
        fullCount = 0;
        errorCount = 0;
    }
}
std::string ArduinoComms::readFromSerial(int fd)
{

    // tcflush(fd, TCIFLUSH);
    char buffer[512];
    int bytesRead = read(fd, buffer, sizeof(buffer) - 1);
    if (bytesRead > 0)
    {
        buffer[bytesRead] = '\0';
        std::string result(buffer);

        result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());
        result.erase(std::remove(result.begin(), result.end(), '\r'), result.end());

        result.erase(result.begin(), std::find_if(result.begin(), result.end(), [](unsigned char ch)
                                                  { return !std::isspace(ch); }));

        return result;
    }
    else if (bytesRead == -1)
    {

        RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Error reading from serial port.");
    }
    return "";
}
void ArduinoComms::setMotorValues(int fd, int val_1, int val_2)
{
    std::stringstream ss;
    ss << "m " << std::clamp(val_1, -255, 255) << " " << std::clamp(val_2, -255, 255) << "\r";
    if (val_1 != 0 && val_2 != 0)
        RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Driving robot: %s", ss.str().c_str());
    sendMsg(fd, ss.str());
}

// void ArduinoComms::setPidValues(int fd, float k_p, float k_i, float k_d)
// {
//     std::stringstream ss;
//     ss << "u " << k_p << ":" << k_i << ":" << k_d << "\r";
//     sendMsg(fd, ss.str());
//     RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "PID Parameters sent");
// }

void ArduinoComms::sendMsg(int fd, const std::string &msg_to_send)
{
    for (const char &character : msg_to_send)
    {

        sendCharToPi(fd, character); // Send each character one by one
    }

    sendCharToPi(fd, '\n');
    // serial_conn_.write(msg_to_send);
    // std::string response = serial_conn_.readline();

    // if (print_output)
    // {
    //     RCLCPP_DEBUG(rclcpp::get_logger("PicoHardware"), "Sent: %s", msg_to_send.c_str());
    //     RCLCPP_DEBUG(rclcpp::get_logger("PicoHardware"), "Received: %s", response.c_str());
    // }

    // return response;
}

void ArduinoComms::sendCharToPi(int fd, char character)
{
    int bytesWritten = write(fd, &character, 1); // Write 1 byte (character) to the serial port

    if (bytesWritten == -1)
    {
        // perror("Error writing to serial port");

        RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Error writing to serial port");
    }
    else
    {
        // std::cout << "Sent character: " << character << std::endl;

        // RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "sent character: %c", character);
    }
}