cmake_minimum_required(VERSION 3.8)
project(hydrant_finder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(hydrant_finder_node src/hydrant_finder_node.cpp)

ament_target_dependencies(hydrant_finder_node
  rclcpp sensor_msgs geometry_msgs cv_bridge image_transport
  tf2 tf2_ros tf2_geometry_msgs std_msgs nav_msgs 
)

target_link_libraries(hydrant_finder_node ${OpenCV_LIBS})

install(TARGETS hydrant_finder_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  
  set(ament_cmake_copyright_FOUND TRUE)  
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
