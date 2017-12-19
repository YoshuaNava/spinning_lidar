cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_launch)


find_package(catkin REQUIRED COMPONENTS
  spinning_lidar_motor_control
  urg_node
)

catkin_package(
  CATKIN_DEPENDS  spinning_lidar_motor_control urg_node
)

