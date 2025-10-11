cmake_minimum_required(VERSION 3.8)
project(final_project_cpp_course1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller rclcpp my_robot_interfaces turtlesim geometry_msgs)

add_executable(turtle_manager src/turtle_manager.cpp)
ament_target_dependencies(turtle_manager rclcpp my_robot_interfaces turtlesim geometry_msgs)

install(TARGETS
  turtle_controller
  turtle_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
