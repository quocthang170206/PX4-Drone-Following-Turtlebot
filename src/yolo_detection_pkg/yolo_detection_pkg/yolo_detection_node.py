#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import threading
import time


class PID:
    def __init__(self, kp, ki, kd, out_min=-2.0, out_max=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def compute(self, error, now):
        if self.prev_time is None:
            dt = 0.05
        else:
            dt = max(now - self.prev_time, 1e-3)
        self.prev_time = now

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(self.out_min, min(self.out_max, out))


class YOLOPIDNode(Node):
    def __init__(self):
        super().__init__("yolo_pid_tracker")

        self.model = YOLO("/home/quocthang/ros2_ws/src/yolo_detection_pkg/weights.pt")

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            "/iris/downward_camera/down_cam/image_raw",
            self.image_cb,
            10,
        )
        self.img_pub = self.create_publisher(Image, "/yolo/annotated_image", 10)
        self.cmd_pub = self.create_publisher(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", 10
        )

        self.frame = None
        self.lock = threading.Lock()

        self.alpha = 0.8
        self.prev_x = 0.0
        self.prev_y = 0.0

        self.pid_x = PID(2.0, 0.0, 0.8)
        self.pid_y = PID(2.0, 0.0, 0.8)
        self.last_time = time.time()

        self.hold_thr = 0.15

        self.running = True
        threading.Thread(target=self.loop, daemon=True).start()
        self.get_logger().info("YOLO PID + Hold Window tracker started")

    def image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.frame = cv_img.copy()
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")

    def loop(self):
        while self.running:
            with self.lock:
                frame = None if self.frame is None else self.frame.copy()

            if frame is None:
                time.sleep(0.05)
                continue

            frame = cv2.resize(frame, (640, 480))
            det = self.model(frame)
            boxes = det[0].boxes.xywh
            self.img_pub.publish(
                self.bridge.cv2_to_imgmsg(det[0].plot(), encoding="bgr8")
            )

            twist = Twist()
            if len(boxes) > 0:
                x_c, y_c, *_ = boxes[0].tolist()
                w, h = frame.shape[1], frame.shape[0]

                nx = -(x_c - w / 2) / (w / 2)
                ny = -(y_c - h / 2) / (h / 2)

                x_off = self.alpha * nx + (1 - self.alpha) * self.prev_x
                y_off = self.alpha * ny + (1 - self.alpha) * self.prev_y
                self.prev_x, self.prev_y = x_off, y_off

                now = time.time()

                if abs(x_off) < self.hold_thr:
                    vel_y = 0.0
                else:
                    vel_y = self.pid_y.compute(x_off, now)

                if abs(y_off) < self.hold_thr:
                    vel_x = 0.0
                else:
                    vel_x = self.pid_x.compute(y_off, now)

                twist.linear.x = vel_x
                twist.linear.y = vel_y
                self.cmd_pub.publish(twist)

                if vel_x or vel_y:
                    self.get_logger().info(
                        f"MOVE | vx={vel_x:.2f}, vy={vel_y:.2f}, offset=({x_off:.2f}, {y_off:.2f})"
                    )
                else:
                    self.get_logger().info("HOLDING POSITION (within precision window)")
            else:
                self.cmd_pub.publish(twist)
                self.get_logger().warn("No target detected – hovering")

            time.sleep(0.05)

    def destroy(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
