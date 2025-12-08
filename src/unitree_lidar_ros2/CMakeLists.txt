cmake_minimum_required(VERSION 3.10)
project(unitree_lidar_ros2)

# C / C++ standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(unilidar_sdk2 REQUIRED)

# Include paths
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(unitree_lidar_ros2_node
  src/unitree_lidar_ros2_node.cpp
)

target_include_directories(unitree_lidar_ros2_node
  PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
)

# PCL needs its preprocessor defs
target_compile_definitions(unitree_lidar_ros2_node
  PRIVATE
    ${PCL_DEFINITIONS}
)

# Link against PCL + UniLidar SDK vendor target
target_link_libraries(unitree_lidar_ros2_node
  PRIVATE
    ${PCL_LIBRARIES}
    unilidar_sdk2::unilidar_sdk2
)

ament_target_dependencies(unitree_lidar_ros2_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
)

# Install node
install(TARGETS unitree_lidar_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch + rviz config
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
