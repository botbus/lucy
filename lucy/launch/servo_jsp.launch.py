# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from launch.actions import LogInfo
# def generate_launch_description():
#     serial_port_servo = LaunchConfiguration("serial_port_servo", default="/dev/ttyACM1")
#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'serial_port_servo',
#             default_value=serial_port_servo,
#             description='Specifying usb port to connected servo'),
            
#             Node(
#             package='lucy',
#             executable='servo_publisher_node',
#             name='servo_publisher_node',
#             parameters=[{"serial_port_servo": serial_port_servo}]
#         ),
#     ])
