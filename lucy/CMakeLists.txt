################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(lucy)
cmake_policy(SET CMP0144 NEW)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

set(SLLIDAR_SDK_PATH "./sdk/")

FILE(GLOB SLLIDAR_SDK_SRC 
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(tf2_ros REQUIRED) 
find_package(tf2 REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(PCL REQUIRED)
# find_package(pcl_conversions REQUIRED)

include_directories(
  ${SLLIDAR_SDK_PATH}/include
  ${SLLIDAR_SDK_PATH}/src
  ${Boost_INCLUDE_DIRS}
  include/servo
)
link_directories(include/servo)
link_libraries(libSCServo.a ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES})

add_executable(sllidar_client src/sllidar_client.cpp)
# add_executable(imu_odom_node src/imu_odom_node.cpp)
add_executable(sllidar_node	src/sllidar_node.cpp ${SLLIDAR_SDK_SRC})
add_executable(servo_publisher_node src/servo_publisher_node.cpp)
# add_executable(joystick_vibration src/joystick_vibration.cpp)
# add_executable(cmd_vel_relay src/cmd_vel_relay_node.cpp)
# add_executable(my_point_cloud_node src/my_point_cloud_node.cpp)

# target_include_directories(my_point_cloud_node PUBLIC ${PCL_INCLUDE_DIRS})
# target_link_libraries(my_point_cloud_node ${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})
ament_target_dependencies(sllidar_node
  rclcpp
  std_srvs
  sensor_msgs
)
# ament_target_dependencies(my_point_cloud_node
#   rclcpp
#   std_srvs
#   sensor_msgs
#   tf2_ros
#   geometry_msgs
#   PCL
#   pcl_conversions
# )
# ament_target_dependencies(
#   imu_odom_node
#   rclcpp
#   std_msgs
#   sensor_msgs
#   nav_msgs

# )

ament_target_dependencies(
  servo_publisher_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  sensor_msgs
)
ament_target_dependencies(sllidar_client
  rclcpp
  std_srvs
  sensor_msgs
)
# ament_target_dependencies(cmd_vel_relay
#   rclcpp
#   geometry_msgs
# )
install(
  TARGETS 
  # cmd_vel_relay
  # imu_odom_node
  servo_publisher_node
  sllidar_node 
  sllidar_client
  # joystick_vibration
  # my_point_cloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(include)
ament_package()