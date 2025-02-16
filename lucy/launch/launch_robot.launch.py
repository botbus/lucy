import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction,DeclareLaunchArgument, EmitEvent, LogInfo,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, AndSubstitution, LaunchConfiguration, NotSubstitution
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.event_handlers import OnStateTransition
from launch_ros.actions import Node, LifecycleNode
from launch.conditions import IfCondition
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():

    start_time = time.time()  
    pkg_name = "lucy"
    pkg_share_path = os.path.join(get_package_share_directory("lucy"))
    xacro_file = os.path.join(pkg_share_path, "description", "lucy.urdf.xacro")
    robot_urdf = Command(["xacro ", xacro_file])
    controller_params_file = os.path.join(get_package_share_directory(pkg_name), "config", "controller_manager_params.yaml")
    joy_params_file = os.path.join(get_package_share_directory(pkg_name),'config','joy_params.yaml')
    ekf_params_file = os.path.join(get_package_share_directory(pkg_name),'config','ekf_params.yaml')
    lidar_params_file = os.path.join(get_package_share_directory(pkg_name),'config','lidar_params.yaml')
    twist_mux_params_file = os.path.join(get_package_share_directory(pkg_name), "config", "twist_mux_params.yaml")

    
    ekf_node= Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file]
    )

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params_file],
    )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params_file],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
    )

    twist_stamper_node = Node(
            package='twist_stamper',
            executable='twist_stamper',
            remappings=[('/cmd_vel_in','/cmd_vel_joy'),
                        ('/cmd_vel_out','/diff_drive_controller/cmd_vel')]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_urdf}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_urdf}, controller_params_file],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    lidar = Node(
            package=pkg_name,
            executable='lidar_node',
            name='lidar_node',
            parameters=[lidar_params_file],
            output='screen'
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params_file],
        remappings=[("/cmd_vel_out", "/diff_drive_controller/cmd_vel_unstamped")],
    )

    async_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory("lucy"), "launch", "online_async_launch.py"
                )
            ])
    )


    delayTime = 1.0
    delayed_robot_state_publisher_node = TimerAction(period=delayTime, actions=[robot_state_publisher_node])
    
    delayTime += 1.0
    delayed_controller_manager = TimerAction(period=delayTime, actions=[controller_manager])

    delayTime += 3.0
    delayed_imu_sensor_broadcaster_spawner = TimerAction(period=delayTime, actions=[imu_sensor_broadcaster_spawner])
    
    delayTime += 2.0
    delayed_diff_drive_controller_spawner = TimerAction(period=delayTime, actions=[diff_drive_controller_spawner])
    
    delayTime += 2.0
    delayed_joint_state_broadcaster_spawner = TimerAction(period=delayTime, actions=[joint_state_broadcaster_spawner])
    
    delayTime += 6.0
    delayed_lidar = TimerAction(period=delayTime, actions=[lidar])
    
    delayTime += 2.0
    delayed_ekf_node = TimerAction(period=delayTime, actions=[ekf_node])
    
    delayTime += 6.0
    delayed_twist_mux = TimerAction(period=delayTime, actions=[twist_mux])
    
    delayTime += 2.0
    delayed_twist_stamper_node = TimerAction(period=delayTime, actions=[twist_stamper_node])
    
    delayTime += 3.0
    delayed_slam_toolbox = TimerAction(period=delayTime, actions=[async_slam_toolbox])
    
    delayTime += 2.0
    delayed_teleop_node = TimerAction(period=delayTime, actions=[teleop_node])
    
    delayTime += 1.0
    delayed_joy_node = TimerAction(period=delayTime, actions=[joy_node])
    
    
    return LaunchDescription(
        [
            delayed_robot_state_publisher_node,
            delayed_controller_manager,
            delayed_imu_sensor_broadcaster_spawner,
            delayed_diff_drive_controller_spawner,
            delayed_joint_state_broadcaster_spawner,
            delayed_lidar,
            delayed_ekf_node,
            delayed_twist_mux,
            delayed_twist_stamper_node,
            delayed_slam_toolbox,
            delayed_teleop_node,
            delayed_joy_node,
        ]
    )
