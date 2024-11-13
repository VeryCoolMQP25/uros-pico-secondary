import pygame
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
import os
os.environ['SDL_RENDERER'] = 'software'  # Forces SDL to use the software renderer

cur_angular = 0.0
cur_linear = 0.0

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
    def timer_callback(self):
        msg = Twist()
        msg.angular.z = cur_angular
        msg.linear.x = cur_linear
        self.publisher_.publish(msg)

class PygameRunner(Node):
    def __init__(self):
        super().__init__('teleop_pygame_runner')
        self.timer = self.create_timer(0.05, self.timer_callback)
        print("Init pygame...")
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("No xbox controller connected!")
            pygame.quit()
            exit()
        self.controller = pygame.joystick.Joystick(0)
        self.font = pygame.font.SysFont(None, 24)
        self.screen = pygame.display.set_mode((800, 400), pygame.RESIZABLE)
        pygame.display.set_caption("Teleop Client")
        self.screen.fill((0, 0, 0))


    def timer_callback(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Get joystick inputs
        button_states = []
        for i in range(self.controller.get_numbuttons()):
            button_states.append(self.controller.get_button(i))

        # Get joystick axes
        axes = [self.controller.get_axis(i) for i in range(self.controller.get_numaxes())]

        # Display button states
        for i, state in enumerate(button_states):
            text = self.font.render(f"Button {i}: {'Pressed' if state else 'Released'}", True, (255, 255, 255))
            self.screen.blit(text, (10, 30 + 20 * i))

        # Display joystick axes (left and right analog sticks)
        text = self.font.render(f"Left Stick: ({axes[0]:.2f}, {axes[1]:.2f})", True, (255, 255, 255))
        self.screen.blit(text, (10, 30 + 20 * len(button_states)))
        text = self.font.render(f"Right Stick: ({axes[3]:.2f}, {axes[4]:.2f})", True, (255, 255, 255))
        self.screen.blit(text, (10, 30 + 20 * (len(button_states) + 1)))
        pygame.display.flip()


def main(args=None):
    print("Init RCLpy...")
    rclpy.init(args=args)


    twist_publisher = TwistPublisher()
    display = PygameRunner()
    executor = MultiThreadedExecutor()
    executor.add_node(twist_publisher)
    executor.add_node(display)

    executor.spin()

    twist_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
