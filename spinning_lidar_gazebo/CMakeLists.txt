cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  spinning_lidar_motor_control
)

catkin_package(
  CATKIN_DEPENDS  spinning_lidar_motor_control
)

