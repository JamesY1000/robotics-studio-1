#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ExampleNodePy(Node):
    def __init__(self):
        super().__init__("example_node_py")
        self.publisher = self.create_publisher(String, "example_topic_py", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period_sec=timer_period, callback=self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Publishing the {self.count}st message"
        self.publisher.publish(msg=msg)
        self.get_logger().info(f"Publishing the {self.count}st message...")
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNodePy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()