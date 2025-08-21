cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "srv/ComputeRectangleArea.srv"
  "msg/LedInformation.msg"
  "srv/PowerLed.srv"
  "msg/BatteryPercentage.msg"
  "msg/TurtleSpawnInfo.msg"
  "msg/Target.msg"
  "msg/TargetArray.msg"
  "srv/KillTurtle.srv"
  "action/CountUntil.action"
  "action/MoveRobot.action"
  "action/MoveTurtle.action"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
