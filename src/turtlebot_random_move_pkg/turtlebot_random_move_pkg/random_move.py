#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class RandomMove(Node):
    def __init__(self):
        super().__init__('random_move_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = random.uniform(0.0, 0.8)
        msg.angular.z = random.uniform(-0.1, -0.1)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Moving: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = RandomMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
