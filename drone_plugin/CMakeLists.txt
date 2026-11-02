cmake_minimum_required(VERSION 3.8)
project(drone_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Message used by drone_delivery's `fly` node to command rotor speeds.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorSpeed.msg"
)

# The URDF loads this as filename="libdrone.so", so the target must be `drone`.
add_library(drone SHARED src/drone_plugin_.cpp)
target_include_directories(drone PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(drone
  rclcpp gazebo_ros gazebo_dev geometry_msgs std_msgs tf2_ros)

# Link the plugin against this package's own generated message type support.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(drone "${cpp_typesupport_target}")

install(TARGETS drone
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
