cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_motor_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs tf)

add_message_files(
  FILES
  MotorState.msg
)

add_service_files(
  FILES
  TurnMotorOnOff.srv
  ChangeTargetVelocity.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs 
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES spinning_lidar_motor_control
  #  CATKIN_DEPENDS
    DEPENDS tf
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(lidar_tf_publisher src/lidar_tf_publisher.cpp)
target_link_libraries(lidar_tf_publisher ${catkin_LIBRARIES})