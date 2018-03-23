cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_imu)


find_package(catkin REQUIRED COMPONENTS
  xsens_driver
)

catkin_package(
  CATKIN_DEPENDS  xsens_driver
)

