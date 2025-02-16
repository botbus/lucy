import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'lucy'    
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    

    ld = LaunchDescription()
    ld.add_action(declare_ekf_config_file_cmd)
    ld.add_action(start_ekf_node_cmd)

    return ld
