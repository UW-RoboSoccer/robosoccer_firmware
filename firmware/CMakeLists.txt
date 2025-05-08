cmake_minimum_required(VERSION 3.8)
project(firmware_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(firmware_bridge_node 
  src/firmware_bridge_node.cpp
  src/firmware_bridge_utils.cpp
)

ament_target_dependencies(firmware_bridge_node 
  rclcpp 
  std_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  firmware_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(include msg)

ament_package()
