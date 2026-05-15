#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_deg(dx, dy):
    return math.degrees(math.atan2(dy, dx))


def wrap_deg(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


class CompareOdomGps(Node):
    def __init__(self):
        super().__init__("compare_odom_gps_realtime")

        self.local_start = None
        self.gps_start = None
        self.map_start = None

        self.local_now = None
        self.gps_now = None
        self.map_now = None

        self.create_subscription(
            Odometry,
            "/scout/odom_filtered",
            self.local_cb,
            20
        )

        self.create_subscription(
            Odometry,
            "/odometry/gps",
            self.gps_cb,
            20
        )

        self.create_subscription(
            Odometry,
            "/scout/map",
            self.map_cb,
            20
        )

        self.timer = self.create_timer(1.0, self.print_status)

    def extract(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        return x, y, yaw, msg.header.frame_id, msg.child_frame_id

    def local_cb(self, msg):
        self.local_now = self.extract(msg)
        if self.local_start is None:
            self.local_start = self.local_now

    def gps_cb(self, msg):
        self.gps_now = self.extract(msg)
        if self.gps_start is None:
            self.gps_start = self.gps_now

    def map_cb(self, msg):
        self.map_now = self.extract(msg)
        if self.map_start is None:
            self.map_start = self.map_now

    def delta(self, start, now):
        if start is None or now is None:
            return None

        dx = now[0] - start[0]
        dy = now[1] - start[1]
        dist = math.sqrt(dx * dx + dy * dy)
        direction = angle_deg(dx, dy) if dist > 1e-6 else 0.0
        yaw = math.degrees(now[2])

        return dx, dy, dist, direction, yaw, now[3], now[4]

    def print_one(self, name, start, now):
        d = self.delta(start, now)
        if d is None:
            print(f"{name}: waiting...")
            return None

        dx, dy, dist, direction, yaw, frame_id, child_frame_id = d

        print(
            f"{name:22s} frame={frame_id:8s} child={child_frame_id:10s} "
            f"dx={dx:+8.3f} dy={dy:+8.3f} "
            f"dist={dist:7.3f} m dir={direction:+8.2f} deg yaw={yaw:+8.2f} deg"
        )

        return d

    def print_status(self):
        print("\n================ REALTIME DELTA FROM START ================")

        d_local = self.print_one("/scout/odom_filtered", self.local_start, self.local_now)
        d_gps = self.print_one("/odometry/gps", self.gps_start, self.gps_now)
        d_map = self.print_one("/scout/map", self.map_start, self.map_now)

        if d_local is not None and d_gps is not None:
            local_dist = d_local[2]
            gps_dist = d_gps[2]

        if local_dist < 1.0 or gps_dist < 1.0:
            print(
            f"STATUS: WAITING FOR MOTION. "
            f"local_dist={local_dist:.3f} m, gps_dist={gps_dist:.3f} m"
            )
        else:
            diff = wrap_deg(d_gps[3] - d_local[3])
            print(f"GPS direction - local direction = {diff:+.2f} deg")

            if abs(diff) > 30.0:
                print("STATUS: BAD. /odometry/gps and /scout/odom_filtered are not aligned.")
            else:
                print("STATUS: GOOD. Directions are roughly aligned.")

            print("============================================================")


def main():
    rclpy.init()
    node = CompareOdomGps()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
