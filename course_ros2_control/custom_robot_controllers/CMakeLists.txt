cmake_minimum_required(VERSION 3.8)
project(custom_robot_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(example_interfaces REQUIRED)


include_directories(include)

add_library(
  ${PROJECT_NAME} SHARED
  src/complimentary_filter_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_lifecycle controller_interface pluginlib example_interfaces)

pluginlib_export_plugin_description_file(controller_interface complimentary_filter_controller_plugin.xml)

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
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
