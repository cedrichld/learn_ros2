cmake_minimum_required(VERSION 3.8)
project(robot_arm_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(robot_arm_interfaces REQUIRED)

add_executable(test_moveit1 src/test_moveit1.cpp)
ament_target_dependencies(test_moveit1 rclcpp moveit_ros_planning_interface)

add_executable(test_moveit2 src/test_moveit2.cpp)
ament_target_dependencies(test_moveit2 rclcpp moveit_ros_planning_interface)

add_executable(commander src/commander_template.cpp)
ament_target_dependencies(commander rclcpp moveit_ros_planning_interface example_interfaces robot_arm_interfaces)

install(TARGETS 
  test_moveit1
  test_moveit2
  commander
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
