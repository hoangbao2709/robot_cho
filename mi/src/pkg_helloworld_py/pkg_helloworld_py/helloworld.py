#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class HelloWorldNode(Node):
    def __init__(self):
        super().__init__('helloworld')
        # Gọi mỗi 0.5 giây
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        self.get_logger().info('Hello World')

def main(args=None):
        rclpy.init(args=args)
        node = HelloWorldNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

