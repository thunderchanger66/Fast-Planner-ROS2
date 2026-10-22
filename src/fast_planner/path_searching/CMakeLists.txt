cmake_minimum_required(VERSION 3.8)
project(fast_path_searching)

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
# further dependencies manually.
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fast_plan_env REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories( 
    include
    include/plan_env
    ${Eigen3_INCLUDE_DIRS} 
)

add_library(fast_path_searching
   src/kinodynamic_astar.cpp
   src/astar.cpp)
target_include_directories(fast_path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fast_path_searching 
  rclcpp
  std_msgs
  fast_plan_env
  visualization_msgs
) 

# target_link_libraries( path_searching
# rclcpp
# plan_env
# ) 

install(TARGETS fast_path_searching
EXPORT export_fast_path_searching
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)


install(DIRECTORY include/
DESTINATION include)

ament_export_targets(export_fast_path_searching HAS_LIBRARY_TARGET)
ament_export_dependencies(fast_plan_env)
ament_package()
