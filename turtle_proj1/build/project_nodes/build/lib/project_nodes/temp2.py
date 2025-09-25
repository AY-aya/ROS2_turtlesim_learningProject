#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import SetPen

class ShapeDrawer(Node):
    def __init__(self):
        super().__init__('shape_drawer')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        time.sleep(0.5)  # let publisher register

        # tuning (change if you want)
        self.linear_speed = 1.5    # m/s used for forward moves
        self.angular_speed = 1.5   # rad/s used for turns

    

    def move_to(self, x, y, theta=0.0):
        client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /turtle1/teleport_absolute service...')
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def set_pen(self, r=255, g=255, b=255, width=2, off=0):
        client = self.create_client(SetPen, '/turtle1/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            pass
        req = SetPen.Request()
        req.r, req.g, req.b, req.width, req.off = r, g, b, width, off
        client.call_async(req)
        
    def _publish_for(self, twist: Twist, duration: float):
        """Publish twist repeatedly for duration seconds, staying responsive."""
        end = time.time() + duration
        while time.time() < end:
            self.pub.publish(twist)
            # allow callbacks to run (stop/clear etc.)
            rclpy.spin_once(self, timeout_sec=0.05)
        # stop
        self.pub.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.05)

    def move_distance(self, distance: float, speed: float = None):
        """Move forward 'distance' meters. Positive distance moves forward."""
        if speed is None:
            speed = self.linear_speed
        if distance == 0:
            return
        duration = abs(distance) / speed
        twist = Twist()
        twist.linear.x = speed if distance > 0 else -speed
        self._publish_for(twist, duration)

    def turn_angle(self, angle_rad: float, angular_speed: float = None):
        """
        Turn by angle_rad (radians). Positive = left (ccw), negative = right (cw).
        Uses angular_speed (rad/s) to compute duration.
        """
        if angular_speed is None:
            angular_speed = self.angular_speed
        if angle_rad == 0:
            return
        duration = abs(angle_rad) / angular_speed
        twist = Twist()
        twist.angular.z = angular_speed if angle_rad > 0 else -angular_speed
        self._publish_for(twist, duration)

    # ---------- shapes using distance/angle semantics ----------
    
    def draw_star(self, side_length=2.0):
        # 5-point star: move side_length then turn 144° (2.513 rad) 5 times
        angle = math.radians(144.0)
        for _ in range(5):
            self.move_distance(side_length)
            self.turn_angle(angle)

    def draw_circle(self, linear=1.0, angular=1.0, revolutions=1.0):
        # drive forward while turning: run for revolutions * 2pi/omega seconds
        duration = revolutions * 2 * math.pi / abs(angular)
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._publish_for(twist, duration)

    def draw_infinity(self, a=3.0, step=0.05):
        center_x, center_y = 5.5, 5.5
        t = 0.0

        # disable pen, teleport to start point
        self.set_pen(off=1)
        x = center_x + a * math.cos(t)
        y = center_y + a * math.sin(t) * math.cos(t)
        self.move_to(x, y)

        # enable pen again
        self.set_pen(off=0)

        # now draw smoothly
        t += step
        while t <= 2 * math.pi:
            x = center_x + a * math.cos(t)
            y = center_y + a * math.sin(t) * math.cos(t)
            self.move_to(x, y)
            t += step

    def draw_flower(self, petals=6, radius=3.0, step=0.05):
        """
        Draws a flower shape using a rose curve: r = cos(kθ).
        petals: number of petals
        radius: scaling factor (smaller if it hits the walls)
        """
        center_x, center_y = 5.5, 5.5
        t = 0.0
        k = petals
        while t < 2 * math.pi:
            r = radius * math.cos(k * t)
            x = center_x + r * math.cos(t)
            y = center_y + r * math.sin(t)
            self.move_to(x, y)
            t += step
    

def main():
    rclpy.init()
    node = ShapeDrawer()

    # Examples:
    print("Drawing a square (side 2.0)...")
    #node.draw_square(side_length=2.0)

    #rint("Drawing a star (side 1.4)...")
    node.draw_infinity()

    # you can try other shapes:
    # node.draw_circle(linear=1.2, angular=1.0)
    # node.draw_flower(petals=6, petal_radius=1.0)
    # node.draw_infinity(loop_radius=0.9)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
