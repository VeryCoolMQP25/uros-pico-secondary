# Tourbot Microcontroller - Pi Pico

The low level control system for the TourBot, implemented in micro-ROS and the Pi Pico C SDK.

## Functional Overview

This system is responsible for the following tasks:
* Direct control of actuators
* Drivetrain and Lift PID control
* Sampling and publishing of IMU and odometry data to ROS [Not yet implemented]
* Safety monitoring with proximity sensors [Not yet implemented]

This software is designed with saftey at its core. The micro-controller may initiate a halt in multiple cases including but not limited to:
- Loss of communication with ROS serial agent 
- Short range proximity sensor tripped [Not yet implemented]
- stale actuator control data
- IMU tipping detected [Not yet implemented]
- IMU impact detected [Not yet implemented]

## Software structure

This implementation does not make use of an RTOS, but directly on bare metal. Core 0 of the processor is responsable for
ROS communications handling, sensor sampling, and other miscelaneous tasks. Core 1 is responsible for actuator control, PID and pose calculations, etc.

## Debugging
The pico outputs a log over UART0 (pins 1 and 2) for debugging and troubleshooting at a baud rate of 115200.
This project includes some unmodified source files from the [Pi Pico Examples](https://github.com/raspberrypi/pico-examples) under BSD 3-clause.
