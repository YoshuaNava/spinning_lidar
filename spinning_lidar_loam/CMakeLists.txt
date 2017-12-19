cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_loam)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

set(BUILD_TESTS TRUE)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
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

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
    DEPENDS EIGEN3 PCL
    INCLUDE_DIRS include
    CATKIN_DEPENDS cmake_modules laser_assembler laser_geometry message_filters pcl_ros roscpp sensor_msgs std_msgs tf
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_executable(loam_feature_extractor src/feature_extractor.cpp)
target_link_libraries(loam_feature_extractor ${catkin_LIBRARIES})


if(BUILD_TESTS)
    add_executable(test_loam_feature_extraction test/test_loam_feature_extraction.cpp)
    target_link_libraries(test_loam_feature_extraction ${catkin_LIBRARIES})
endif()