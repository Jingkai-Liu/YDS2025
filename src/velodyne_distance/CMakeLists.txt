cmake_minimum_required(VERSION 3.5)
project(velodyne_distance)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  add_compile_options(-march=armv8-a)
endif()
message(STATUS "Processor:${CMAKE_SYSTEM_PROCESSOR}")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(distance_node
  src/distance_node.cpp
  src/compute_distance.cpp
)

target_include_directories(distance_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(distance_node
  rclcpp
  sensor_msgs
)

install(TARGETS distance_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
