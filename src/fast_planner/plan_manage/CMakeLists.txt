cmake_minimum_required(VERSION 3.8)
project(plan_manage)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(bspline REQUIRED)
find_package(fast_bspline_opt REQUIRED)
find_package(poly_traj REQUIRED)
find_package(fast_planner REQUIRED)
find_package(fast_path_searching REQUIRED)
find_package(fast_traj_utils REQUIRED)
find_package(fast_plan_env REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)  


add_executable(fast_planner_node  
  src/planner_manager.cpp
  src/kino_replan_fsm.cpp
  #src/topo_replan_fsm.cpp
  src/fast_planner_node.cpp
)

ament_target_dependencies(fast_planner_node 
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs
  fast_plan_env
  fast_path_searching
  bspline
  fast_bspline_opt
  poly_traj
  quadrotor_msgs
  fast_traj_utils
  fast_planner
  builtin_interfaces
)

target_link_libraries(fast_planner_node
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

 ###########################################################
add_executable(traj_server  
  src/traj_server.cpp
)

ament_target_dependencies(traj_server 
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs
  fast_plan_env
  fast_path_searching
  bspline
  fast_bspline_opt
  poly_traj
  quadrotor_msgs
  fast_traj_utils
  fast_planner
  builtin_interfaces
)

target_link_libraries(traj_server
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
)


install(TARGETS fast_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS traj_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)
install(FILES rviz2_fastplanner.rviz
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_package()

