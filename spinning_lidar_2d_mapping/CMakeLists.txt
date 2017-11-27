cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_2d_mapping)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    laser_geometry
    message_filters
    roscpp
    sensor_msgs
    std_msgs
    tf
)

find_package(Eigen3)

catkin_package(
    DEPENDS eigen
    INCLUDE_DIRS include
    CATKIN_DEPENDS cmake_modules laser_geometry message_filters roscpp sensor_msgs std_msgs tf
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)


add_executable(test_pcl_gen test/test_pcl_gen.cpp)
target_link_libraries(test_pcl_gen ${catkin_LIBRARIES})
