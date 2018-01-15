cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_icpslam)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

set(BUILD_TESTS FALSE)

find_package(GTSAM REQUIRED)
find_package(G2O REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
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
    DEPENDS EIGEN3 PCL
    INCLUDE_DIRS include
    CATKIN_DEPENDS cmake_modules gazebo_msgs laser_assembler laser_geometry message_filters pcl_ros roscpp sensor_msgs std_msgs tf
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR} 
    /usr/include/suitesparse
)

LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})
LIST(APPEND LINK_LIBS
    gtsam
    ${G2O_CORE_LIBRARY} 
    ${G2O_STUFF_LIBRARY} 
    ${G2O_SOLVER_CHOLMOD} 
    ${G2O_SOLVER_CSPARSE} 
    ${G2O_SOLVER_DENSE}
    ${G2O_SOLVER_PCG} 
    ${G2O_TYPES_ICP}
    ${G2O_TYPES_SLAM3D}
    ${G2O_TYPES_SBA}
    cholmod
    cxsparse
)


## Build library from utils source files
LIST(APPEND UTIL_SOURCEFILES
    src/utils/geometric_utils.cpp
    src/utils/messaging_utils.cpp
)
add_library(ROS_UTILS ${UTIL_SOURCEFILES})
target_link_libraries(ROS_UTILS
  ${LINK_LIBS}
)

## Build library from icpslam source files
LIST(APPEND ICPSLAM_SOURCEFILES
    src/icpslam/icp_odometer.cpp
    src/icpslam/pose_optimizer_g2o.cpp
    src/icpslam/pose_optimizer_gtsam.cpp
    src/icpslam/octree_mapper.cpp
)
add_library(${PROJECT_NAME} ${ICPSLAM_SOURCEFILES})
target_link_libraries(${PROJECT_NAME}
  ${LINK_LIBS}
)


## Utils for handling the spinning lidar point clouds
add_executable(lidar_scan_filtering src/laser_processing/lidar_scan_filtering.cpp)
target_link_libraries(lidar_scan_filtering ${catkin_LIBRARIES})

add_executable(interrupt_laser_assembler src/laser_processing/interrupt_laser_assembler.cpp)
target_link_libraries(interrupt_laser_assembler ${catkin_LIBRARIES})

add_executable(gazebo_ref_frame_pub src/utils/gazebo_ref_frame_publisher.cpp)
target_link_libraries(gazebo_ref_frame_pub ${catkin_LIBRARIES} ROS_UTILS)



## ICP SLAM
add_executable(icpslam src/icpslam/icpslam.cpp)
target_link_libraries(icpslam ${catkin_LIBRARIES} ${PROJECT_NAME} ROS_UTILS)


if(BUILD_TESTS) 
    add_executable(test_2d_cloud_gen_laserpipeline test/laser_pipeline/test_2d_cloud_gen_laserpipeline.cpp) 
    target_link_libraries(test_2d_cloud_gen_laserpipeline ${catkin_LIBRARIES}) 

    add_executable(test_assemblers_laserpipeline test/laser_pipeline/test_assemblers_laserpipeline.cpp) 
    target_link_libraries(test_assemblers_laserpipeline ${catkin_LIBRARIES}) 

endif()