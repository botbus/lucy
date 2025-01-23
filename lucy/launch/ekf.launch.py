import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate a launch description for the EKF node.

    This function creates and returns a LaunchDescription object that will start
    the EKF node from the robot_localization package. The node is configured
    using parameters from a YAML file.

    Returns:
        LaunchDescription: A complete launch description for the EKF node
    """
    # Constants for paths to different files and folders
    package_name = 'lucy'

    # Config file paths
    ekf_config_file_path = 'config/ekf.yaml'

    # Set the path to different packages
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # Set the path to config files
    default_ekf_config_path = os.path.join(pkg_share, ekf_config_file_path)

    # Launch configuration variables
    ekf_config_file = LaunchConfiguration('ekf_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_ekf_config_file_cmd = DeclareLaunchArgument(
        name='ekf_config_file',
        default_value=default_ekf_config_path,
        description='Full path to the EKF configuration YAML file'
    )

    # Specify the actions
    start_ekf_node_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file,
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_ekf_config_file_cmd)
    ld.add_action(start_ekf_node_cmd)

    return ld
