import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from picrawler_interfaces.msg import DigitalPinState, OnboardButtonState

class PicrawlerKeyboard(Node):

    def __init__(self):
        super().__init__('picrawler_keyboard')

        self.publisher_ = self.create_publisher(Bool, 'led_state', 10)
        self.timer_ = self.create_timer(1, self.timer_callback)
        self.current_state_ = False

        self.button_subscriber_ = self.create_subscription(OnboardButtonState, 'onboard_button_state', self.log_buttons, 10)

    def timer_callback(self):
        self.get_logger().info('led is: ' + ('on' if self.current_state_ else 'off'))
        msg = Bool()
        msg.data = self.current_state_
        self.publisher_.publish(msg)
        self.current_state_ = not self.current_state_

    def log_buttons(self, msg: OnboardButtonState):
        self.get_logger().info('rst button: ' + ('pressed' if msg.rst_button else 'released'))
        self.get_logger().info('usr button: ' + ('pressed' if msg.usr_button else 'released'))

def main(args=None):
    rclpy.init(args=args)
    node = PicrawlerKeyboard()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()