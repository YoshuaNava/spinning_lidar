cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_icpslam)

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
    # INCLUDE_DIRS include
    CATKIN_DEPENDS cmake_modules laser_assembler laser_geometry message_filters pcl_ros roscpp sensor_msgs std_msgs tf
)

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)


add_executable(lidar_scan_filtering src/lidar_scan_filtering.cpp)
target_link_libraries(lidar_scan_filtering ${catkin_LIBRARIES})

add_executable(interrupt_laser_assembler src/interrupt_laser_assembler.cpp)
target_link_libraries(interrupt_laser_assembler ${catkin_LIBRARIES})

add_executable(icpslam src/icpslam.cpp)
target_link_libraries(icpslam ${catkin_LIBRARIES})


if(BUILD_TESTS)
    add_executable(test_2d_cloud_gen_laserpipeline test/laser_pipeline/test_2d_cloud_gen_laserpipeline.cpp)
    target_link_libraries(test_2d_cloud_gen_laserpipeline ${catkin_LIBRARIES})

    add_executable(test_assemblers_laserpipeline test/laser_pipeline/test_assemblers_laserpipeline.cpp)
    target_link_libraries(test_assemblers_laserpipeline ${catkin_LIBRARIES})

    add_executable(test_pcl_ros test/test_pcl_ros.cpp)
    target_link_libraries(test_pcl_ros ${catkin_LIBRARIES})

    add_executable(test_pcl_registration test/test_pcl_registration.cpp)
    target_link_libraries(test_pcl_registration ${catkin_LIBRARIES})
endif()