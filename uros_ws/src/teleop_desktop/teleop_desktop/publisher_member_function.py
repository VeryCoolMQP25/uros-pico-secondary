import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from inputs import get_gamepad

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'drivetrain_powers', 10) 
        self.update_powers()

    def update_powers(self):
        x = 0
        y = 0
        z = 0
        while True:
            fresh = False
            events = get_gamepad()
            for event in events:
                if event.ev_type == "Absolute": 
                    match event.code:
                        case "ABS_RX":
                            x = event.state/(-327.68)
                            fresh = True                        
                        case "ABS_RY":
                            y = event.state/(-327.68)
                            fresh = True
                        case "ABS_Y":
                            z = event.state/(-327.68)
                            fresh = True
                    
            if fresh:
                leftpower = (y + x)
                rightpower = (y - x)
                if abs(leftpower) > 100:
                    leftpower = 100*(leftpower/abs(leftpower))
                if abs(rightpower) > 100:
                    rightpower = 100*(rightpower/abs(rightpower))
                msg = Int32MultiArray()
                msg.data = [int(leftpower), int(rightpower), int(100*z)]
                self.publisher_.publish(msg)
                self.get_logger().info(f'Pufblishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
