cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_utils)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

find_package(PCL REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    laser_assembler
    laser_geometry
    nodelet
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf2
)
catkin_package(
  DEPENDS PCL
  CATKIN_DEPENDS
    cmake_modules
    laser_assembler
    laser_geometry
    nodelet
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/spinning_lidar_utils/interrupt_laser_assembler.cpp
  src/spinning_lidar_utils/laser_range_filter.cpp
  src/spinning_lidar_utils/scan_to_cloud_converter.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_nodelets
  src/interrupt_laser_assembler_nodelet.cpp
  src/laser_range_filter_nodelet.cpp
  src/scan_to_cloud_converter_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelets
  ${PROJECT_NAME}
)

## Utils for handling the lidar point clouds
add_executable(scan_to_cloud_converter src/scan_to_cloud_converter_node.cpp)
target_link_libraries(scan_to_cloud_converter
  ${PROJECT_NAME}
)

add_executable(laser_range_filter src/laser_range_filter_node.cpp)
target_link_libraries(laser_range_filter
  ${PROJECT_NAME}
)

add_executable(interrupt_laser_assembler src/interrupt_laser_assembler_node.cpp)
target_link_libraries(interrupt_laser_assembler
  ${PROJECT_NAME}
)