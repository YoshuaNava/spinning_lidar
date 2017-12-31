cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_icpslam)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

set(BUILD_TESTS FALSE)


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

find_package(G2O REQUIRED)
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
    ${G2O_INCLUDE_DIR} 
    /usr/include/suitesparse
)

LIST(APPEND LINK_LIBS
    ${G2O_CORE_LIBRARY} 
    ${G2O_STUFF_LIBRARY} 
    ${G2O_SOLVER_CHOLMOD} 
    ${G2O_SOLVER_CSPARSE} 
    ${G2O_SOLVER_DENSE}
    ${G2O_SOLVER_PCG} 
    ${G2O_TYPES_SBA} cholmod cxsparse
  )
LIST(APPEND SOURCEFILES src/bundle_adjustment.cpp)


add_executable(lidar_scan_filtering src/utils/lidar_scan_filtering.cpp)
target_link_libraries(lidar_scan_filtering ${catkin_LIBRARIES})

add_executable(interrupt_laser_assembler src/utils/interrupt_laser_assembler.cpp)
target_link_libraries(interrupt_laser_assembler ${catkin_LIBRARIES})


add_executable(odometry src/icpslam/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})

add_executable(mapping src/icpslam/mapping.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES})


if(BUILD_TESTS)
    add_executable(test_2d_cloud_gen_laserpipeline test/laser_pipeline/test_2d_cloud_gen_laserpipeline.cpp)
    target_link_libraries(test_2d_cloud_gen_laserpipeline ${catkin_LIBRARIES})

    add_executable(test_assemblers_laserpipeline test/laser_pipeline/test_assemblers_laserpipeline.cpp)
    target_link_libraries(test_assemblers_laserpipeline ${catkin_LIBRARIES})

    add_executable(test_pcl_ros test/pcl/test_pcl_ros.cpp)
    target_link_libraries(test_pcl_ros ${catkin_LIBRARIES})

    add_executable(test_pcl_icp test/pcl/test_pcl_icp.cpp)
    target_link_libraries(test_pcl_icp ${catkin_LIBRARIES})

    # add_executable(test_pcl_registration test/pcl/test_pcl_registration.cpp)
    # target_link_libraries(test_pcl_registration ${catkin_LIBRARIES})
endif()