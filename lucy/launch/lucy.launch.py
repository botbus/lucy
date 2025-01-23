#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,LogInfo, RegisterEventHandler
# from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('lucy'),'launch','joystick.launch.py'
                )])
    )
    
    lidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('lucy'),'launch','sllidar_c1_launch.py'
                )])
    )
    
    
    servo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('lucy'),'launch','servo_jsp.launch.py'
                )])
    )
    
    imu_odom = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('lucy'),'launch','imu_odom_pub.launch.py'
                )])
    )
    ekf = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('lucy'),'launch','ekf.launch.py'
                )])
    )
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('lucy'),'launch','rsp.launch.py'
                )])
    )
    # pkg_path = os.path.join(get_package_share_directory('lucy'))      
    return LaunchDescription([
        joystick,
        lidar,
        servo,
        imu_odom,
        ekf,
        rsp
    ])

