cmake_minimum_required(VERSION 2.8.3)
project(spinning_lidar_motor_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
    tf
)

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


catkin_package(
    CATKIN_DEPENDS
      tf
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)

