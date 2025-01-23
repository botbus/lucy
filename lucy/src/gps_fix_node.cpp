#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <mqtt/async_client.h>
#include <string>
#include <memory>
#include <chrono>

#include <nlohmann/json.hpp>
class GPSPublisher : public rclcpp::Node
{
public:
	GPSPublisher() : Node("gps_publisher"), mqtt_client_(MQTT_SERVER_ADDRESS, CLIENT_ID)
	{
		// ROS Publisher
		gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
		dest_gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix_dest", 10);
		// Timer for GPS publishing
		timer_ = this->create_wall_timer(
			std::chrono::seconds(1),
			std::bind(&GPSPublisher::publishGPSData, this));

		// MQTT setup
		mqtt::connect_options conn_opts;
		conn_opts.set_clean_session(true);

		mqtt_callback_ = std::make_shared<Callback>(gps_publisher_);
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
		explicit Callback(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> gps_publisher)
			: gps_publisher_(gps_publisher) {}

		void message_arrived(mqtt::const_message_ptr msg) override
		{
			RCLCPP_INFO(rclcpp::get_logger("MQTT"), "Received MQTT message: '%s' on topic: '%s'",
						msg->to_string().c_str(), msg->get_topic().c_str());
			std::string inputString = msg->to_string();

			const std::string delimiter = "|";
			size_t start = inputString.find_first_of(delimiter, 0);
			size_t end = 0;

			if (start != std::string::npos)
				start += 1;
			// Loop until we have processed the entire input string
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
					// Catch any exceptions (e.g., substr issues)
					std::cerr << "big TIME substr error: " << e.what() << std::endl;
					std::cout << "\n\n\n"
							  << inputString << "\n\n\n";
					break;
				}

				// if (end == std::string::npos)
				//     break;
				start = end + delimiter.length();
				size_t len = jsonString.length();
				if (len <= 3)
					continue;
				try
				{
					// sensor_msgs::msg::Imu imu_msg;
					// nav_msgs::msg::Odometry odom_msg;
					json parsedJson = json::parse(jsonString);
					auto source_fix = sensor_msgs::msg::NavSatFix();
					auto destination_fix = sensor_msgs::msg::NavSatFix();
					destination.header.stamp = this->now();
					source.header.frame_id = "gps_frame";
					source.header.stamp = this->now();
					destination.header.frame_id = "gps_frame";

					// String from mqtt {"src":{lat":-412.0048621,"long":2339.629339},"dest":{"lat":-234.005217,"long":31239.631857}}

					if (parsedJson.contains("src"))
					{
						auto _source = parsedJson["src"];
						source_fix.latitude = _source["lat"];	// Set gyro X value
						source_fix.longitude = _source["long"]; // Set gyro Y value
					}
					if (parsedJson.contains("dest"))
					{

						auto _destination = parsedJson["dest"];
						destination_fix.latitude = _destination["lat"];	  // Set accelerometer X value
						destination_fix.longitude = _destination["long"]; // Set accelerometer Y value
					}

					dest_gps_publisher_->publish(destination_fix);
					gps_publisher_->publish(source_fix);
				}
				catch (const json::exception &e)
				{
					std::cerr << "JSON parsing error: " << e.what() << std::endl;

					std::cout << "\n\n\n"
							  << inputString << "\n\n\n";
					std::cout << "\n\n\n"
							  << jsonString << "\n\n\n";
				}

				// Move the start position to the next part after the delimiter
				start = end + 1;
			}
		}

	private:
		std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> dest_gps_publisher_;
		std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> gps_publisher_;
	};

	// void publishGPSData()
	// {
	// 	auto message = sensor_msgs::msg::NavSatFix();
	// 	message.header.stamp = this->now();
	// 	message.header.frame_id = "gps_frame";

	// 	// Simulated GPS coordinates
	// 	message.latitude = 37.7749;
	// 	message.longitude = -122.4194;
	// 	message.altitude = 10.0;

	// 	// GPS status
	// 	message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
	// 	message.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

	// 	// Position covariance
	// 	message.position_covariance[0] = 1.0;
	// 	message.position_covariance[4] = 1.0;
	// 	message.position_covariance[8] = 5.0;
	// 	message.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

	// 	gps_publisher_->publish(message);
	// 	RCLCPP_INFO(this->get_logger(), "Published GPS fix: lat=%.6f, lon=%.6f, alt=%.2f",
	// 				message.latitude, message.longitude, message.altitude);
	// }

	std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> gps_publisher_;
	std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> dest_gps_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
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
