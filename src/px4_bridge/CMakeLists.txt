cmake_minimum_required(VERSION 3.8)
project(px4_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(px4_bridge_node src/px4_bridge_node.cpp)
target_compile_features(px4_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(px4_bridge_node
  rclcpp
  px4_msgs
  quadrotor_msgs
  nav_msgs
  std_msgs
)

install(TARGETS px4_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/manual_goal.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME manual_goal
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
