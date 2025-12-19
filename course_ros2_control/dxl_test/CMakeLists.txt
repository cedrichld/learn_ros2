cmake_minimum_required(VERSION 3.8)
project(dxl_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)

add_executable(test_motors src/test_motors.cpp)
ament_target_dependencies(test_motors rclcpp dynamixel_sdk)

install(TARGETS
  test_motors
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
