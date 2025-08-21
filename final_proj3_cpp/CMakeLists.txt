cmake_minimum_required(VERSION 3.8)
project(final_proj3_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(turtlesim REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(lifecycle_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_library(turtle_spawn_kill_component SHARED src/turtle_spawn_kill.cpp)
ament_target_dependencies(turtle_spawn_kill_component rclcpp rclcpp_lifecycle rclcpp_components turtlesim example_interfaces std_srvs)
rclcpp_components_register_nodes(turtle_spawn_kill_component "final_proj3_cpp::TurtleSpawnKillClientNode")

add_library(move_turtle_component SHARED src/move_turtle.cpp)
ament_target_dependencies(move_turtle_component rclcpp rclcpp_action rclcpp_components turtlesim geometry_msgs my_robot_interfaces example_interfaces)
rclcpp_components_register_nodes(move_turtle_component "final_proj3_cpp::MoveTurtleServerNode")

add_library(move_turtlebot3_component SHARED src/move_turtlebot3.cpp)
ament_target_dependencies(move_turtlebot3_component rclcpp rclcpp_action rclcpp_components nav_msgs geometry_msgs my_robot_interfaces example_interfaces)
rclcpp_components_register_nodes(move_turtlebot3_component "final_proj3_cpp::MoveTurtlebot3ServerNode")

# add_executable(turtle_spawn_kill src/turtle_spawn_kill.cpp)
# ament_target_dependencies(turtle_spawn_kill rclcpp rclcpp_lifecycle turtlesim example_interfaces std_srvs)

# add_executable(move_turtle src/move_turtle.cpp)
# ament_target_dependencies(move_turtle rclcpp rclcpp_action turtlesim geometry_msgs my_robot_interfaces example_interfaces)

install(TARGETS
  turtle_spawn_kill_component move_turtle_component move_turtlebot3_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS
#   move_turtle
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
