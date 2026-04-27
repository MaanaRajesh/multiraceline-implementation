cmake_minimum_required(VERSION 3.5)
project(multiraceline_racing)

set(CMAKE_CXX_STANDARD 17)
                                                                     set(CMAKE_CXX_STANDARD_REQUIRED ON) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

foreach(PACKAGE
  ament_cmake
  rclcpp
  std_msgs
  sensor_msgs
  ackermann_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  raceline_msgs)
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

add_executable(tactical_racing_node
  src/tactical_racing_node.cpp
  src/frenet_frame.cpp
  src/raceline_library.cpp
  src/opponent_detector.cpp
  src/opponent_tracker.cpp
  src/tactical_state_machine.cpp
  src/pure_pursuit_controller.cpp
)

ament_target_dependencies(tactical_racing_node
  rclcpp
  std_msgs
  sensor_msgs
  ackermann_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  raceline_msgs
)

install(TARGETS tactical_racing_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  src/config
  src/launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
