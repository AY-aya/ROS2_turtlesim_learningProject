#!/usr/bin/env python3
import rclpy
import math
import time
import threading
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute, SetPen

class TurtleCommander(Node):
    def __init__(self):
        super().__init__("turtle_commander")
        self.shape_sub_ = self.create_subscription(String, "/shape_input", self.shape_reciever, 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.linear_speed = 1.5   
        self.angular_speed = 1.5   
        self.is_drawing = False
        self.get_logger().info("Turtle Commander Node has been started")

    def shape_reciever(self, msg:String):
        self.shape = msg.data
        #self.get_logger().info("Received Shape: "+ self.shape)
        
        if self.shape == "clear":
            self.clear()
        elif self.shape == "stop":
            self.is_drawing = False
            msg = Twist()
            self.cmd_vel_pub_.publish(msg)
        elif not self.is_drawing:
            self.is_drawing = True
            # start shape in a background thread
            threading.Thread(target=self.start_drawing, daemon=True).start()
    
    def start_drawing(self):
        self.get_logger().info("Now Drawing: "+ self.shape)
        if self.shape == "Star":
            self.draw_star()
        elif self.shape == "Leaf":
            self.draw_leaf()
        elif self.shape == "Flower":
            self.draw_flower()
        elif self.shape == "Butterfly":
            self.draw_butterfly()
        self.is_drawing = False  # Drawing completed

    def clear(self):
        client = self.create_client(Empty, '/clear')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /clear service...')
        req = Empty.Request()
        client.call_async(req)

        # return to center
        self.is_drawing = True
        self.set_pen(off=1)
        self.move_to(5.5, 5.5, 0.0)
        self.set_pen(off=0)   
        self.is_drawing = False
        
        self.get_logger().info("Cleared screen")
    
    # 3 Shapes Drawer Functions:
    def draw_star(self, side_length=2.0):
        # move side_length then turn 144° 5 times
        angle = math.radians(144.0)
        for _ in range(5):
            if not self.is_drawing: return
            self.move_distance(side_length)
            if not self.is_drawing: return
            self.turn_angle(angle)

    def draw_infinity(self, a=2.0, step=0.02):
        if not self.is_drawing: return
        
        center_x, center_y = 5.5, 5.5
        t = 0.0

        # disable pen, teleport to start point
        self.set_pen(off=1)
        x = center_x + a * math.cos(t)
        y = center_y + a * math.sin(t) * math.cos(t)
        self.move_to(x, y)

        # enable pen again
        self.set_pen(off=0)

        t += step
        while t <= 2 * math.pi and self.is_drawing:
            x = center_x + a * math.cos(t)
            y = center_y + a * math.sin(t) * math.cos(t)
            self.move_to(x, y)
            t += step

    def draw_flower(self, petals=6, radius=2.0, step=0.05):
        #using rose curve
        if not self.is_drawing: return
        
        center_x, center_y = 5.5, 5.5
        t = 0.0
        k = petals
        
        # disable pen, teleport to start point
        self.set_pen(off=1)
        r = radius * math.cos(k * t)
        x = center_x + r * math.cos(t)
        y = center_y + r * math.sin(t)
        self.move_to(x, y)

        # enable pen again
        self.set_pen(off=0)

        while t < 2 * math.pi and self.is_drawing:
            r = radius * math.cos(k * t)
            x = center_x + r * math.cos(t)
            y = center_y + r * math.sin(t)
            self.move_to(x, y)
            t += step

    def draw_butterfly(self, step=0.05, t_max=12 * math.pi, scale=0.8):
        center_x, center_y = 5.5, 5.5
        t = 0.0

        while t < t_max:
            x = math.sin(t) * (
                math.e**math.cos(t)
                - 2 * math.cos(4 * t)
                - (math.sin(t / 12) ** 5)
            )
            y = math.cos(t) * (
                math.e**math.cos(t)
                - 2 * math.cos(4 * t)
                - (math.sin(t / 12) ** 5)
            )
            self.move_to(center_x + scale * x, center_y + scale * y)
            t += step

    def draw_leaf(self, step=0.05, t_max=20 * math.pi, scale=0.1):
        center_x, center_y = 5.5, 5.5
        t = 0.0

        while t < t_max:
            r = math.sin(2 * t) * math.exp(-0.01 * t)  # damped sinusoidal radius
            x = r * math.cos(t)
            y = r * math.sin(t)
            self.move_to(center_x + scale * x * 40, center_y + scale * y * 40)
            t += step

    # Helper Functions
    def _publish_for(self, twist: Twist, duration: float):
        """Publish twist repeatedly for duration seconds, staying responsive."""
        end = time.time() + duration
        while time.time() < end and self.is_drawing:
            self.cmd_vel_pub_.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.is_drawing:
            self.cmd_vel_pub_.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

    def move_distance(self, distance: float, speed: float = None):
        if not self.is_drawing or distance == 0:
            return
        if speed is None:
            speed = self.linear_speed
        duration = abs(distance) / speed
        twist = Twist()
        twist.linear.x = speed if distance > 0 else -speed
        self._publish_for(twist, duration)

    def turn_angle(self, angle_rad: float, angular_speed: float = None):
        if not self.is_drawing or angle_rad == 0:
            return
        if angular_speed is None:
            angular_speed = self.angular_speed
        duration = abs(angle_rad) / angular_speed
        twist = Twist()
        twist.angular.z = angular_speed if angle_rad > 0 else -angular_speed
        self._publish_for(twist, duration)

    def move_to(self, x, y, theta=0.0):
        #Teleport to position (non-blocking)
        if not self.is_drawing:
            return
        client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not client.wait_for_service(timeout_sec=1.0) and self.is_drawing:
            self.get_logger().info('waiting for teleport service...')
        
        if self.is_drawing:
            req = TeleportAbsolute.Request()
            req.x = x
            req.y = y
            req.theta = theta
            future = client.call_async(req)
            # Wait briefly for completion but stay responsive
            start_time = time.time()
            while time.time() - start_time < 1.0 and self.is_drawing:
                if future.done():
                    break

    def set_pen(self, r=255, g=255, b=255, width=2, off=0):
        # used to enable/disable drawing when the turtle moves
        if not self.is_drawing:
            return
        client = self.create_client(SetPen, '/turtle1/set_pen')
        while not client.wait_for_service(timeout_sec=1.0) and self.is_drawing:
            self.get_logger().info('waiting for service...')
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.is_drawing:
            req = SetPen.Request()
            req.r, req.g, req.b, req.width, req.off = r, g, b, width, off
            client.call_async(req)
            rclpy.spin_once(self, timeout_sec=0.1)
    
    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()