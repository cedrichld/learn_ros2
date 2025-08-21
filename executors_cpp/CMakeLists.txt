cmake_minimum_required(VERSION 3.8)
project(executors_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(single_threaded_executor src/single_threaded_executor.cpp)
ament_target_dependencies(single_threaded_executor rclcpp)

add_executable(multi_threaded_executor src/multi_threaded_executor.cpp)
ament_target_dependencies(multi_threaded_executor rclcpp)

install(TARGETS
  single_threaded_executor
  multi_threaded_executor
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
