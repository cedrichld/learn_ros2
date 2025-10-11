cmake_minimum_required(VERSION 3.8)
project(lifecycle_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp rclcpp_lifecycle example_interfaces)

install(TARGETS
  number_publisher
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
