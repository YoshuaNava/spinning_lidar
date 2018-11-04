cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_utils)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

set(BUILD_TESTS FALSE)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    gazebo_msgs
    laser_assembler
    laser_geometry
    message_filters
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
)
catkin_package(
  DEPENDS PCL
  CATKIN_DEPENDS
    cmake_modules
    gazebo_msgs
    laser_assembler
    laser_geometry
    message_filters
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


## Utils for handling the lidar point clouds
add_executable(lidar_scan_to_cloud src/lidar_scan_to_cloud.cpp)
target_link_libraries(lidar_scan_to_cloud ${catkin_LIBRARIES})

add_executable(lidar_scan_filtering src/lidar_scan_filtering.cpp)
target_link_libraries(lidar_scan_filtering ${catkin_LIBRARIES})

add_executable(interrupt_laser_assembler src/interrupt_laser_assembler.cpp)
target_link_libraries(interrupt_laser_assembler ${catkin_LIBRARIES})


if(BUILD_TESTS) 
  add_executable(test_2d_cloud_gen_laserpipeline test/laser_pipeline/test_2d_cloud_gen_laserpipeline.cpp)
  target_link_libraries(test_2d_cloud_gen_laserpipeline ${catkin_LIBRARIES})

  add_executable(test_assemblers_laserpipeline test/laser_pipeline/test_assemblers_laserpipeline.cpp)
  target_link_libraries(test_assemblers_laserpipeline ${catkin_LIBRARIES})
endif()