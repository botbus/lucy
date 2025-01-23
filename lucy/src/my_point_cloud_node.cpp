#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class Lidar3DMapper : public rclcpp::Node
{
public:
    Lidar3DMapper()
        : Node("lidar_3d_mapper"), current_angle_(0.0)
    {
        // Subscribers
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Lidar3DMapper::scanCallback, this, std::placeholders::_1));
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Lidar3DMapper::jointCallback, this, std::placeholders::_1));

        // Publisher
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_points", 10);

        // TF Buffer and Listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Assume joint[0] is the revolute joint controlling lidar
        if (!msg->position.empty())
        {

            for (size_t i = 0; i < msg->name.size(); ++i)
            {
                if (msg->name[i] == "base_lidar_joint")
                {
                    // RCLCPP_INFO(this->get_logger(), "name of joint: %s", msg->name[i].c_str());
                    // RCLCPP_INFO(this->get_logger(), "pose of joint: %f", msg->position[i]);
                    current_angle_ = msg->position[i];
                    break; // Exit loop once the joint is found
                }
            }
        }
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;

        double angle = msg->angle_min;
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max)
            {
                angle += msg->angle_increment;
                continue;
            }

            // Compute 3D points
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            double z = std::sin(current_angle_); // Use joint angle for Z-coordinate

            cloud.points.emplace_back(x, y, z);
            angle += msg->angle_increment;
        }

        // Convert PCL PointCloud to ROS PointCloud2 message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.frame_id = "laser_frame";
        cloud_msg.header.stamp = this->get_clock()->now();

        // Publish the 3D point cloud
        point_cloud_pub_->publish(cloud_msg);
    }

    // Node components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

    // TF components
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Current joint angle
    double current_angle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Lidar3DMapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
