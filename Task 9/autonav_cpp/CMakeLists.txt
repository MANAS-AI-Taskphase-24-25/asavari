cmake_minimum_required(VERSION 3.8)
project(autonav_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)

add_executable(astar_planner_node src/astar_planner_node.cpp)
add_executable(path_follower_node src/path_follower_node.cpp)

ament_target_dependencies(astar_planner_node
  rclcpp nav_msgs geometry_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs
)
ament_target_dependencies(path_follower_node
  rclcpp nav_msgs geometry_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs angles
)

install(TARGETS  
  astar_planner_node
  path_follower_node  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
