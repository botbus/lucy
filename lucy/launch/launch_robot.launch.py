import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import RegisterEventHandler

from launch_ros.actions import Node

def generate_launch_description():

    package_name='lucy' 
    pkg_path = os.path.join(get_package_share_directory('lucy'))
    xacro_file = os.path.join(pkg_path,'description','lucy.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])
    # time = 0.0
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )])
    )
    
    # delayed_rsp = TimerAction(period=time, actions=[rsp])

    servo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('lucy'),'launch','servo_jsp.launch.py'
                )])
    )
    # time+=2.0
    # delayed_servo_spawner = TimerAction(period=time, actions=[servo])

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )])
    )
    # time+=1.0
    # delayed_joy_spawner = TimerAction(period=time, actions=[joystick])
 
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )    
    # time+=1.5
    # delayed_twist_mux_spawner = TimerAction(period=time, actions=[twist_mux])

    
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )
    # time+=3.5
    # delayed_controller_manager = TimerAction(period=time, actions=[controller_manager])

    imu_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster"],
    )
    # time+=3.5
    # delayed_imu_spawner = TimerAction(period=time, actions=[imu_spawner])
   
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    # time+=3.5
    # delayed_diff_drive_spawner = TimerAction(period=time, actions=[diff_drive_spawner])

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    # time+=3.5
    # delayed_joint_broad_spawner = TimerAction(period=time, actions=[joint_broad_spawner])
   
    ekf = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('lucy'),'launch','ekf.launch.py'
                )])
    )
    # time+=3.5
    # delayed_ekf_spawner = TimerAction(period=time, actions=[ekf])
   
    lidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('lucy'),'launch','sllidar_c1_launch.py'
                )])
    )
    # time+=3.5
    # delayed_lidar_spawner = TimerAction(period=time, actions=[lidar])
  
    slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','online_async_launch.py'
            )])
    )
    # time+=3.5
    # delayed_slam_spawner = TimerAction(period=time, actions=[slam])
  

    return LaunchDescription([
        # delayed_rsp,
        # delayed_servo_spawner,
        # delayed_joy_spawner,
        # delayed_twist_mux_spawner,
        # delayed_controller_manager, 
        # delayed_imu_spawner,
        # delayed_diff_drive_spawner,
        # delayed_joint_broad_spawner,
        # delayed_ekf_spawner,
        # delayed_lidar_spawner,
        # # delayed_slam_spawner
        rsp,
        servo,
        joystick,
        twist_mux,
        controller_manager,
        imu_spawner,
        diff_drive_spawner,
        joint_broad_spawner,
        ekf,
        lidar,
        slam
    ])
