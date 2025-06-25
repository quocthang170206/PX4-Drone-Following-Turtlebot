import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        self.publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.timer = self.create_timer(0.1, self.publish_setpoint)
        self.setpoint = PoseStamped()
        self.setpoint.pose.position.x = 0.0
        self.setpoint.pose.position.y = 0.0
        self.setpoint.pose.position.z = 4.0  # Hover at 2m
        self.get_logger().info('Takeoff node initialized — sending position setpoints')

    def publish_setpoint(self):
        self.setpoint.header = Header()
        self.setpoint.header.stamp = self.get_clock().now().to_msg()
        self.setpoint.header.frame_id = "map"
        self.publisher.publish(self.setpoint)

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
