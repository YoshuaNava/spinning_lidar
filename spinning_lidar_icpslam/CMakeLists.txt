cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_icpslam)

add_compile_options(-std=c++11)
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

set(BUILD_TESTS FALSE)

find_package(G2O REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
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

LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})
LIST(APPEND LINK_LIBS
    ${G2O_CORE_LIBRARY} 
    ${G2O_STUFF_LIBRARY} 
    ${G2O_SOLVER_CHOLMOD} 
    ${G2O_SOLVER_CSPARSE} 
    ${G2O_SOLVER_DENSE}
    ${G2O_SOLVER_PCG} 
    ${G2O_TYPES_ICP}
    ${G2O_TYPES_SLAM3D}
    ${G2O_TYPES_SBA}
    cholmod cxsparse
)


## Build library from icpslam source files
LIST(APPEND SOURCEFILES
    src/utils/geometric_utils.cpp
    src/utils/messaging_utils.cpp
    src/icpslam/icp_odometer.cpp
    src/icpslam/pose_optimizer.cpp
    src/icpslam/octree_mapper.cpp
)
add_library(${PROJECT_NAME} ${SOURCEFILES})
target_link_libraries(${PROJECT_NAME}
  ${LINK_LIBS}
)

## Utils for handling the spinning lidar point clouds
add_executable(lidar_scan_filtering src/utils/lidar_scan_filtering.cpp)
target_link_libraries(lidar_scan_filtering ${catkin_LIBRARIES})

add_executable(interrupt_laser_assembler src/utils/interrupt_laser_assembler.cpp)
target_link_libraries(interrupt_laser_assembler ${catkin_LIBRARIES})



## ICP SLAM
add_executable(icpslam src/icpslam/icpslam.cpp)
target_link_libraries(icpslam ${catkin_LIBRARIES} ${PROJECT_NAME})


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