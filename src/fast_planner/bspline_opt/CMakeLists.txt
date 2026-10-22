cmake_minimum_required(VERSION 3.5)
project(fast_bspline_opt)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(fast_plan_env REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories( 
    include 
    include/plan_env   
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( fast_bspline_opt 
    src/bspline_optimizer.cpp 
    )

ament_target_dependencies(fast_bspline_opt
    rclcpp
    std_msgs
    visualization_msgs
    cv_bridge
    fast_plan_env
)

target_link_libraries(fast_bspline_opt
    Eigen3::Eigen
) 

install(DIRECTORY include/
  DESTINATION include/)  

install(TARGETS fast_bspline_opt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(fast_bspline_opt)
ament_export_include_directories(include)
ament_export_dependencies(fast_plan_env Eigen3)
ament_package()
