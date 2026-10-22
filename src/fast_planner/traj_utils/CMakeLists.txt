cmake_minimum_required(VERSION 3.8)
project(fast_traj_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
find_package(bspline REQUIRED)
find_package(fast_bspline_opt REQUIRED)
find_package(poly_traj REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fast_plan_env REQUIRED)
find_package(fast_planner REQUIRED)

include_directories( 
    include 
    include/bspline
    include/poly_traj
    ${Eigen3_INCLUDE_DIRS} 
)

add_library( fast_traj_utils SHARED
    src/planning_visualization.cpp 
  )

ament_target_dependencies(fast_traj_utils
    rclcpp
    std_msgs
    visualization_msgs
    bspline
    poly_traj
    fast_plan_env
    fast_planner
  ) 

target_include_directories(fast_traj_utils PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)


install(DIRECTORY include/
  DESTINATION include)

install(TARGETS fast_traj_utils
  EXPORT export_fast_traj_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_targets(export_fast_traj_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(bspline fast_bspline_opt fast_plan_env poly_traj fast_planner Eigen3)
ament_package()
