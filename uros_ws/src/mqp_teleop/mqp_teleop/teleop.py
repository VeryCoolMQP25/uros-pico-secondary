import rclpy
import rclpy.executors
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from rclpy.publisher import Publisher

LIFT_ENABLE_BTN = 5 # right bumper
LIFT_AXIS = 4 # right stick Y

power: float = 0.0

class JoyListener(Node):
    def __init__(self):
        super().__init__('joy_listener')
        # Subscriber to the /joy topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10  # Queue size
        )

    def joy_callback(self, msg):
        global power
        # Print out the button and axis values
        buttons = msg.buttons  # List of button states (0 or 1)
        axes = msg.axes  # List of axis states (e.g., for analog sticks)
        command = 0.0
        if (msg.buttons[LIFT_ENABLE_BTN]):
            command = msg.axes[LIFT_AXIS]
        power = command
        self.get_logger().debug(f'States: {buttons[LIFT_ENABLE_BTN]}, {axes[LIFT_AXIS]}')
        # self.get_logger().info(f'Buttons: {buttons}')
        # self.get_logger().info(f'Axes: {axes}')

class LiftPublisher(Node):
    def __init__(self):
        super().__init__('lift_raw_publisher')
        self.msg = Float32()
        self.publisher: Publisher = self.create_publisher(Float32, "/lift_raw", 1)
        self.timer = self.create_timer(0.05, self.callback)

    def callback(self):
        self.msg.data = float(power)
        self.publisher.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    joy_listener = JoyListener()
    lift_publish = LiftPublisher()
    executor.add_node(joy_listener)
    executor.add_node(lift_publish)
    executor.spin()
    joy_listener.destroy_node()
    lift_publish.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
