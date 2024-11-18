# Tourbot Microcontroller - Pi Pico

The low level control system for the TourBot, implemented in micro-ROS and the Pi Pico C SDK.

## Usage

To use the Pico robot controller, you will need the micro-ROS agent to interface it with the rest of the ROS2 environment.
I reccomend using the docker version of the agent to prevent issues arrising from your specific environment.
To activate, ensure that docker is installed and running, and that your user account is a member of `docker` and `dialout` (or `uucp` if on Arch).
Then, you can run the command 
```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -v6
```
Note that you may have to replace `ttyACM0` with a different serial port depending on your local configuration. 
Also note that this instruction will only work on Linux. If running Windows: switch to a better OS then try again.

### Teleop
To use the robot in teleop, you can use any package that publishes `Twist` messages to the `/cmd_vel` topic. The reccomended method is using an xbox controller and the following command:
```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```
The `teleop_twist_joy` package should be included with your installation of ROS2. 

Note that this will only allow control of the drivetrain, not the lift. To control the lift, use the [Lift Teleop Node](https://github.com/VeryCoolMQP25/teleop-lift-pkg) and follow the instructions
in that repository's README.

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
ROS communications handling, sensor sampling, and other miscelaneous tasks. Core 1 is responsible for drivetrain control and PID only.

The pico accepts `Twist` messages on the `/cmd_vel` topic, which it executes with the drivetrain. It publishes `TwistStamped` messages on `/twist_observed` for use in odometry calculations.

## Debugging
The pico outputs a log over UART0 (pins 1 and 2) for debugging and troubleshooting at a baud rate of 115200.
This project includes some unmodified source files from the [Pi Pico Examples](https://github.com/raspberrypi/pico-examples) under BSD 3-clause.
