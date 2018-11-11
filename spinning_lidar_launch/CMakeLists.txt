cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_launch)

find_package(catkin REQUIRED
  COMPONENTS
    roslaunch
)

catkin_package(
  CATKIN_DEPENDS
    roslaunch
)

# Sensor drivers
roslaunch_add_file_check(launch/hokuyo_urg.launch)
roslaunch_add_file_check(launch/ueye_cam.launch)
roslaunch_add_file_check(launch/sensor_drivers.launch)

# Spinning lidar
roslaunch_add_file_check(launch/live_sensor.launch)
roslaunch_add_file_check(launch/replay_rosbag.launch)
roslaunch_add_file_check(launch/simulator.launch)
roslaunch_add_file_check(launch/visualization.launch)

