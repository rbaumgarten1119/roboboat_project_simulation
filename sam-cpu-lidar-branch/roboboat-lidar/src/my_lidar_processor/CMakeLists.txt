cmake_minimum_required(VERSION 3.8)
project(my_lidar_processor)

# Use C++17 (needed for std::filesystem and modern PCL)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters)

# Create executable from your node
add_executable(lidar_processor_3d_node src/lidar_processor_3d_node.cpp)
add_executable(pcd_publisher src/pcd_publisher.cpp)

# Link dependencies for lidar processor
ament_target_dependencies(lidar_processor_3d_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
)

ament_target_dependencies(pcd_publisher
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
)

# Ensure PCL include directories and libraries are linked properly
target_include_directories(lidar_processor_3d_node PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(lidar_processor_3d_node ${PCL_LIBRARIES})

target_include_directories(pcd_publisher PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_publisher ${PCL_LIBRARIES})

# Install the node binaries
install(TARGETS
  lidar_processor_3d_node
  pcd_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install launch and config directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# Export the package to the ROS 2 build system
ament_package()
