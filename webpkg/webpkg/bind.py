import serial
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('serial_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.port = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = self.port.readline().decode('utf-8').strip()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


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

