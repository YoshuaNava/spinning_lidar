cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_gazebo)

add_compile_options(-std=c++11)
# set(CMAKE_CXX_FLAGS "-O3 -std=c++11")
set(CMAKE_BUILD_TYPE Release)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(ir_interrupt_plugin SHARED src/ir_interrupt_plugin.cpp)
target_link_libraries(ir_interrupt_plugin ${catkin_LIBRARIES})