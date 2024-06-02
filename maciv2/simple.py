import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node("simple_node")
    logger = node.get_logger()
    logger.info("node here, going to sleep")
    rclpy.spin(node)

if __name__ == '__main__':
    main()