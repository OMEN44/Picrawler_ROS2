import rclpy
from rclpy.node import Node

from picrawler_interfaces.msg import DigitalPinState

class PicrawlerKeyboard(Node):

    def __init__(self):
        super().__init__('picrawler_keyboard')

        self.publisher_ = self.create_publisher(DigitalPinState, 'write_pin_state', 10)
        self.timer_ = self.create_timer(2, self.timer_callback)
        self.current_state_ = False

    def timer_callback(self):
        self.get_logger().info('laser is: ' + ('on' if self.current_state_ else 'off'))
        msg = DigitalPinState()
        msg.pins = [False, False, False, self.current_state_]
        self.current_state_ = not self.current_state_
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PicrawlerKeyboard()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()