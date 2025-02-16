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
include_directories(
    ${PROJECT_SOURCE_DIR}/sdk/include
    ${PROJECT_SOURCE_DIR}/sdk/src
    ${PROJECT_SOURCE_DIR}/sdk/src/arch/linux
)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED) 
# find_package(tf2_ros REQUIRED) 
# find_package(tf2 REQUIRED) 
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(nav2_msgs REQUIRED)


# include_directories(
#   ${SLLIDAR_SDK_PATH}/include
#   ${SLLIDAR_SDK_PATH}/src
#   ${Boost_INCLUDE_DIRS}
#   include/servo
# )
# link_directories(include/servo)
# link_libraries(libSCServo.a ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES})

add_executable(lidar_client src/lidar_client.cpp)
add_executable(lidar_node	src/lidar_node.cpp ${SLLIDAR_SDK_SRC})
# add_executable(joystick_vibration src/joystick_vibration.cpp)
# add_definitions(${PCL_DEFINITIONS})


ament_target_dependencies(lidar_node
  rclcpp
  std_srvs
  sensor_msgs
)

ament_target_dependencies(lidar_client
  rclcpp
  std_srvs
  sensor_msgs
)

# ament_target_dependencies(
#   joystick_vibration
#   rclcpp
#   std_msgs
#   sensor_msgs
#   nav_msgs
#   geometry_msgs
# )


# ament_target_dependencies(
#   servo_publisher_node
#   rclcpp
#   std_msgs
#   geometry_msgs
#   tf2_ros
#   sensor_msgs
# )


install(
  TARGETS 
  lidar_node 
  lidar_client
  # joystick_vibration
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