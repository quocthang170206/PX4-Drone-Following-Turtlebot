#!/usr/bin/env python3
# yolo_pid_track_only.py – Logs offset x/y every 1 s; no blind-search motion

import time, threading, cv2, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO

# ─────────── PID & Kalman helpers ─────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, *, out_min=-1.2, out_max=1.2, dt_default=0.05):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.dt_default = dt_default
        self.reset()

    def reset(self):
        self._prev_error = 0.0
        self._integral   = 0.0
        self._prev_time  = None

    def compute(self, err, now):
        dt = self.dt_default if self._prev_time is None else max(now - self._prev_time, 1e-3)
        self._prev_time = now
        self._integral += err * dt
        deriv = (err - self._prev_error) / dt
        self._prev_error = err
        u = self.kp * err + self.ki * self._integral + self.kd * deriv
        return max(self.out_min, min(self.out_max, u))

class SimpleKalman:
    def __init__(self, q=0.01, r=0.4):
        self.x, self.P, self.Q, self.R = 0.0, 1.0, q, r
    def update(self, z):
        self.P += self.Q
        k = self.P / (self.P + self.R)
        self.x += k * (z - self.x)
        self.P *= 1 - k
        return self.x

# ─────────── Main node ────────────────────────────────────────────────
class YOLOPIDNode(Node):
    def __init__(self):
        super().__init__('yolo_pid_track_only')

        self.model  = YOLO("/home/quocthang/ros2_ws/src/yolo_detection_pkg/weights.pt")
        self.bridge = CvBridge()

        self.tb_id = next((i for i, n in self.model.names.items()
                           if n.lower() == "turtlebot"), None)
        if self.tb_id is None:
            raise RuntimeError("TurtleBot class not found in YOLO model")

        self.create_subscription(Image,
                                 "/iris/downward_camera/down_cam/image_raw",
                                 self.image_cb, 10)
        self.pub_img = self.create_publisher(Image,
                                             "/yolo/annotated_image", 10)
        self.pub_cmd = self.create_publisher(Twist,
                                             "/mavros/setpoint_velocity/cmd_vel_unstamped", 10)

        self._frame, self._lock = None, threading.Lock()
        self.pid_x = PID(1.0, 0.0, 0.6)
        self.pid_y = PID(1.0, 0.0, 0.6)
        self.kx, self.ky = SimpleKalman(), SimpleKalman()
        self.dead = 0.10

        self.last_seen  = time.time() - 10.0
        self.lost_tmo   = 2.0     # seconds before we stop commanding motion
        self.last_log   = 0.0
        self._run       = True
        threading.Thread(target=self.loop, daemon=True).start()
        self.get_logger().info("YOLO PID tracker (no blind search) running")

    # ── Image callback ────────────────────────────────────────────────
    def image_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self._frame = img.copy()
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")

    # ── Main control loop ─────────────────────────────────────────────
    def loop(self):
        dt = 0.05
        while self._run:
            with self._lock:
                frame = None if self._frame is None else self._frame.copy()
            if frame is None:
                time.sleep(dt)
                continue

            frame = cv2.resize(frame, (640, 480))
            res   = self.model(frame)
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(res[0].plot(), "bgr8"))
            boxes = res[0].boxes
            tb    = boxes[boxes.cls.int() == self.tb_id]

            cmd = Twist()
            now = time.time()

            if len(tb):
                self.last_seen = now
                w, h = frame.shape[1], frame.shape[0]
                cx, cy = w / 2, h / 2
                xywh = tb.xywh.cpu().numpy()
                i = int(np.argmin(np.abs(xywh[:, 0] - cx) + np.abs(xywh[:, 1] - cy)))
                x_c, y_c = xywh[i, 0], xywh[i, 1]
                nx = -(x_c - cx) / cx
                ny = -(y_c - cy) / cy

                vx = self.pid_x.compute(self.ky.update(ny), now) if abs(ny) > self.dead else 0.0
                vy = self.pid_y.compute(self.kx.update(nx), now) if abs(nx) > self.dead else 0.0
                cmd.linear.x, cmd.linear.y = vx, vy
            else:
                # TurtleBot lost: hover (zero velocity)
                if now - self.last_seen > self.lost_tmo:
                    cmd.linear.x = cmd.linear.y = 0.0

            if now - self.last_log >= 1.0:
                self.get_logger().info(f"OffsetX: {nx:.4f}, OffsetY: {ny:.4f}")
                self.last_log = now

            self.pub_cmd.publish(cmd)
            time.sleep(dt)

    # ── Shutdown cleanly ──────────────────────────────────────────────
    def destroy(self):
        self._run = False
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
