import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import LogInfo
def generate_launch_description():
    serial_port_pico = LaunchConfiguration("serial_port_pico", default="/dev/ttyACM0")
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port_pico',
            default_value=serial_port_pico,
            description='Specifying usb port to connected servo'),
            
            Node(
            package='lucy',
            executable='imu_odom_node',
            name='imu_odom_node',
            parameters=[{"serial_port_pico": serial_port_pico}]
        ),
    ])
