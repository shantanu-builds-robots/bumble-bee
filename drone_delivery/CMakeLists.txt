cmake_minimum_required(VERSION 3.8)
project(drone_delivery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake_gtest)
find_package(xacro REQUIRED)
find_package(ros2_control REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(drone_plugin REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(include)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)


set(GROUP3_INCLUDE_DEPENDS
  rclcpp 
  geometry_msgs
  tf2
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
  tf2_kdl
  drone_plugin
  
)


add_executable(fly src/fly.cpp )
ament_target_dependencies(fly  ${GROUP3_INCLUDE_DEPENDS})

add_executable(deliver src/pick_and_drop.cpp )
ament_target_dependencies(deliver  ${GROUP3_INCLUDE_DEPENDS})

install(DIRECTORY
  launch
  models
  worlds
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS 

  fly
  deliver
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS
  launch/robot_description_publisher.py
  scripts/aruco_detector.py
  scripts/model_verification.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
