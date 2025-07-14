#!/usr/bin/env python3
# obstacle_avoider_fsm.py  – TurtleBot3 Burger obstacle avoidance + 1 Hz position print
# ──────────────────────────────────────────────────────────────────────────────
import math, random, time
import rclpy
from rclpy.node        import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg   import LaserScan
from nav_msgs.msg      import Odometry

# ───── Tunable parameters ────────────────────────────────────────────────────
SAFE_DIST     = 0.60
CRIT_DIST     = 0.30
FWD_TARGET    = 0.70
TURN_SPEED    = 1.20
BACKUP_SPEED  = -0.20
BACKUP_TIME   = 1.0
FRONT_ARC_DEG = 90
TICK_DT       = 0.10
MAX_LIN_ACC   = 0.40
MAX_ANG_ACC   = 1.20
# ──────────────────────────────────────────────────────────────────────────────

class Modes:
    FORWARD = 0
    BACKUP  = 1
    TURN    = 2

class AvoidFSM(Node):
    def __init__(self):
        super().__init__('tb3_avoid_fsm')

        # ROS I/O
        self.pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub   = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.sub_o = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(TICK_DT, self.tick)

        # Laser bookkeeping
        self.front_idx, self.left_idx, self.right_idx = [], [], []
        self.min_front = float('inf')
        self.min_left  = float('inf')
        self.min_right = float('inf')

        # FSM state
        self.mode            = Modes.FORWARD
        self.backup_counter  = 0
        self.turn_dir        = 1

        # Velocity state
        self.v_lin = 0.0
        self.v_ang = 0.0

        # Print timing
        self.start_time   = time.time()
        self.last_log_sec = -1

    # ───────── Laser callback ────────────────────────────────────────────────
    def scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if not self.front_idx:
            ctr  = int((0.0 - msg.angle_min) / msg.angle_increment)
            half = int(math.radians(FRONT_ARC_DEG/2) / msg.angle_increment)
            self.front_idx = [(ctr+i)            % n for i in range(-half, half+1)]
            self.left_idx  = [(ctr+half+1+i)     % n for i in range(n//4)]
            self.right_idx = [(ctr-half-1-i)     % n for i in range(n//4)]

        def nearest(idxs):
            return min((r for r in (msg.ranges[i] for i in idxs) if 0.05 < r < float('inf')),
                       default=float('inf'))

        self.min_front = nearest(self.front_idx)
        self.min_left  = nearest(self.left_idx)
        self.min_right = nearest(self.right_idx)
        self.turn_dir  = -1 if self.min_left < self.min_right else 1

    # ───────── Odometry callback: print every 1 s ────────────────────────────
    def odom_cb(self, msg: Odometry):
        now = time.time()
        secs = int(now - self.start_time)
        if secs != self.last_log_sec:                  # new full second
            p = msg.pose.pose.position
            print(f"[TurtleBot][{secs:>4d} s] x = {p.x:+.2f}, y = {p.y:+.2f}")
            self.last_log_sec = secs

    # ───────── Utility: 1st-order limiter ────────────────────────────────────
    @staticmethod
    def ramp(cur, tgt, step):
        delta = max(min(tgt - cur, step), -step)
        return cur + delta

    # ───────── Main loop ─────────────────────────────────────────────────────
    def tick(self):
        # FSM transitions
        if self.mode == Modes.FORWARD:
            if self.min_front < CRIT_DIST:
                self.mode = Modes.BACKUP
                self.backup_counter = int(BACKUP_TIME / TICK_DT)
            elif self.min_front < SAFE_DIST:
                self.mode = Modes.TURN
        elif self.mode == Modes.BACKUP:
            self.backup_counter -= 1
            if self.backup_counter <= 0:
                self.mode = Modes.TURN
        elif self.mode == Modes.TURN and self.min_front >= SAFE_DIST:
            self.mode = Modes.FORWARD

        # Set velocities per mode
        cmd = Twist()
        if self.mode == Modes.FORWARD:
            tgt_lin = FWD_TARGET
            tgt_ang = random.uniform(-0.5, 0.5)
            lin_step = MAX_LIN_ACC * TICK_DT
            ang_step = MAX_ANG_ACC * TICK_DT
            self.v_lin = self.ramp(self.v_lin, tgt_lin, lin_step)
            self.v_ang = self.ramp(self.v_ang, tgt_ang, ang_step)
        elif self.mode == Modes.BACKUP:
            self.v_lin = BACKUP_SPEED
            self.v_ang = 0.0
        else:  # TURN
            self.v_lin = 0.0
            self.v_ang = self.turn_dir * TURN_SPEED

        cmd.linear.x  = self.v_lin
        cmd.angular.z = self.v_ang
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AvoidFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
