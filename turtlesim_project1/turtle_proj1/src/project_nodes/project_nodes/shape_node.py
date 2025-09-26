#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class ShapeNode(Node):
    def __init__(self):
        super().__init__("shape_node")
        self.shape_pub= self.create_publisher(String, "/shape_input", 10)
        self.get_logger().info("Choose a shape from:\n1. Star\n2. Flower\n3. Butterfly\n4. Infinity\n5. stop\n6. clear")

        # background thread for user input
        thread = threading.Thread(target=self.read_input)
        thread.daemon = True
        thread.start()

    def read_input(self):
        while rclpy.ok():
            try:
                user_input = input("> ")  # waits for user
                msg = String()
                msg.data = user_input.strip()
                self.shape_pub.publish(msg)
                self.get_logger().info("Recieved input from cmd: "+ msg.data +"\nChoose next shape from:\n1. Star\n2. Flower\n3. Butterfly\n4. Infinity\n5. stop\n6. clear")
                
            except EOFError:
                break

def main(args=None):
    rclpy.init(args=args)
    node = ShapeNode()
    rclpy.spin(node)
    rclpy.shutdown()
