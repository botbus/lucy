#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <string>
#include <memory>
#include <chrono>

using json = nlohmann::json;

class GPSPublisher : public rclcpp::Node
{
public:
    GPSPublisher()
        : Node("gps_publisher"), mqtt_client_(MQTT_SERVER_ADDRESS, CLIENT_ID)
    {
        // ROS Publisher
        gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
        dest_gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix_dest", 10);

        // MQTT setup
        mqtt::connect_options conn_opts;
        conn_opts.set_clean_session(true);

        mqtt_callback_ = std::make_shared<Callback>(shared_from_this(), gps_publisher_, dest_gps_publisher_);
        mqtt_client_.set_callback(*mqtt_callback_);

        try
        {
            RCLCPP_INFO(this->get_logger(), "Connecting to MQTT broker...");
            mqtt_client_.connect(conn_opts)->wait();
            RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker.");
            mqtt_client_.subscribe(MQTT_TOPIC, 1)->wait();
            RCLCPP_INFO(this->get_logger(), "Subscribed to MQTT topic: %s", MQTT_TOPIC.c_str());
        }
        catch (const mqtt::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to MQTT broker: %s", e.what());
        }
    }

    ~GPSPublisher()
    {
        try
        {
            mqtt_client_.disconnect()->wait();
            RCLCPP_INFO(this->get_logger(), "Disconnected from MQTT broker.");
        }
        catch (const mqtt::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to disconnect from MQTT broker: %s", e.what());
        }
    }

private:
    class Callback : public virtual mqtt::callback
    {
    public:
        Callback(std::shared_ptr<GPSPublisher> node,
                 std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> gps_pub,
                 std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> dest_gps_pub)
            : node_(node), gps_publisher_(gps_pub), dest_gps_publisher_(dest_gps_pub) {}

        void message_arrived(mqtt::const_message_ptr msg) override
        {
            try
            {
                RCLCPP_INFO(rclcpp::get_logger("MQTT"), "Received MQTT message: '%s' on topic: '%s'",
                            msg->to_string().c_str(), msg->get_topic().c_str());
                json parsedJson = json::parse(msg->to_string());
                auto source_fix = sensor_msgs::msg::NavSatFix();
                auto destination_fix = sensor_msgs::msg::NavSatFix();

                source_fix.header.stamp = node_->now();
                source_fix.header.frame_id = "gps_frame";

                destination_fix.header.stamp = node_->now();
                destination_fix.header.frame_id = "gps_frame";

                if (parsedJson.contains("src"))
                {
                    auto src = parsedJson["src"];
                    source_fix.latitude = src["lat"];
                    source_fix.longitude = src["long"];
                }

                if (parsedJson.contains("dest"))
                {
                    auto dest = parsedJson["dest"];
                    destination_fix.latitude = dest["lat"];
                    destination_fix.longitude = dest["long"];
                }

                gps_publisher_->publish(source_fix);
                dest_gps_publisher_->publish(destination_fix);
            }
            catch (const json::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MQTT"), "JSON parsing error: %s", e.what());
            }
        }

    private:
        std::shared_ptr<GPSPublisher> node_;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> gps_publisher_;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> dest_gps_publisher_;
    };

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> gps_publisher_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> dest_gps_publisher_;
    mqtt::async_client mqtt_client_;
    std::shared_ptr<Callback> mqtt_callback_;

    const std::string MQTT_TOPIC = "mqtt/ros";
    const std::string MQTT_SERVER_ADDRESS = "tcp://localhost:1883";
    const std::string CLIENT_ID = "mqtt_to_ros_bridge";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
