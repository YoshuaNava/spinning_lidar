# spinning_lidar
ROS stack for spinning 2D LIDAR

For more information, check our [wiki](https://github.com/YoshuaNava/spinning_lidar/wiki)


## TODO
- [X] URDF Description
  - [X] Macro for Hokuyo UST-20LX.
  - [X] Macro for Rotating mount V1.
- [X] Gazebo
  - [X] Custom scene that resembles RPL, floor 7.
  - [X] Plugin to emulate the IR sensor interrupts.
- [X] Launch files
  - [X] For visualizing the URDF model.
  - [X] For visualizing sequences stored in rosbags.
  - [X] For handling a sensor hooked to external computer. (Jetson TX1 use-case)
- [X] Motor control
  - [X] Add teensy udev rules.
  - [X] Arduino code for handling ROS comm and PID control of the motor.
- [X] Utils
  - [X] ROS laser assembler integration.
  - [X] IR interrupt handler.
- [ ] Wiki
  - [ ] Setup
    - [X] Installation.
    - [X] Motor control.
    - [X] Network setup.
    - [ ] Calibration
  - [ ] Using the sensor
    - [ ] Integration into other platforms.
    - [ ] Simulator.
    - [ ] Live sensor.
    - [ ] Time synchronization.
    - [ ] Cloud stitching.
