cmake_minimum_required(VERSION 3.8)
project(bspline)

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

find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


include_directories(include
${Eigen3_INCLUDE_DIRS} 
)  

add_library(bspline SHARED
src/non_uniform_bspline.cpp )

ament_target_dependencies(bspline 
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs
)

target_link_libraries(bspline 
  Eigen3::Eigen)

ament_export_libraries(bspline)
ament_export_include_directories(include)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS bspline
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
