cmake_minimum_required(VERSION 3.8)
project(base_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)

add_library(
  ${PROJECT_NAME}
  src/mobile_base_hardware_interface.cpp
  src/arm_hardware_interface.cpp
)
# add_library(
#   ${PROJECT_NAME}
#   src/arm_hardware_interface.cpp
# )

ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_lifecycle hardware_interface pluginlib dynamixel_sdk)

pluginlib_export_plugin_description_file(hardware_interface base_robot_hardware_interface.xml)

install(
  TARGETS ${PROJECT_NAME} 
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
