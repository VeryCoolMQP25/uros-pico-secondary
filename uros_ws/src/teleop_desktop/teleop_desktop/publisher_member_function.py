# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from inputs import get_gamepad
import threading

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'drivetrain_powers', 10)
        self.xboxthread = threading.Thread(target=self.update_powers)
        self.xboxthread.daemon = True
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.powers = [0, 0]
        self.xboxthread.start()

    def update_powers(self):
        x = 0
        y = 0
        while True:
            fresh = False
            events = get_gamepad()
            for event in events:
                if event.ev_type == "Absolute": 
                    if event.code == "ABS_RX":
                        #right stick X
                        x = event.state/(-327.68)
                        fresh = True
                    elif event.code == "ABS_RY":
                        y = event.state/(-327.68)
                        fresh = True
            if fresh:
                leftpower = (y + x)
                rightpower = (y - x)
                if abs(leftpower) > 100:
                    leftpower = 100*(leftpower/abs(leftpower))
                if abs(rightpower) > 100:
                    rightpower = 100*(rightpower/abs(rightpower))
                self.powers = [int(leftpower), int(rightpower)]
                    

    def timer_callback(self):
        msg = Int32MultiArray()
        msg.data = self.powers
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
