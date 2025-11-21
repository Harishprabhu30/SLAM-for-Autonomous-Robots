import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

