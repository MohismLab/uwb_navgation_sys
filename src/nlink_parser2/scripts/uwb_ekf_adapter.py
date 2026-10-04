#!/usr/bin/env python3
"""Glue between the raw UWB pose and a robot_localization ekf_node.

  /uwb/<robot>/pose (PoseStamped)
      -> /uwb_ekf/<robot>/uwb_pose_cov (PoseWithCovarianceStamped)  -> ekf_node
  ekf_node -> /uwb_ekf/<robot>/odometry/filtered (Odometry)
      -> /uwb_ekf/<robot>/pose (PoseStamped)
      -> /uwb_ekf/<robot>/path (Path, last --history seconds, for RViz)
      -> /uwb_ekf/<robot>/label (Marker, robot name above it, for RViz)
      -> /uwb_ekf/<robot>/heading_valid (Bool, latched): false, pose.orientation is identity
      -> /uwb_ekf/<robot>/pose_valid (Bool, latched): false while no raw pose for --timeout s;
         pose / path / label are not published then
  /uwb_ekf/<robot>/set_pose: the EKF is reset onto the raw pose when the tag comes back after an
  outage or the raw pose stays --reset-distance away (tag restarted, robot carried)

The UWB pose is stamped by the remote machine's clock, so it is re-stamped with
the local clock before it reaches the filter. The EKF runs in 2D and the UWB z is
too noisy to be useful, so the published pose is put on the floor (--floor-z,
= floor_z of linktrack.ekf.launch.py) and RViz draws it on the floor grid.
"""

import argparse
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


class UwbEkfAdapter(Node):
    def __init__(self, args):
        super().__init__(f'uwb_ekf_adapter_{args.robot}')
        ns = f'/uwb_ekf/{args.robot}'
        self.var = args.std ** 2
        self.floor_z = args.floor_z
        self.path = Path()
        self.path_len = int(args.history * 10)
        self.last_path_t = 0.0
        self.cov_pub = self.create_publisher(PoseWithCovarianceStamped, f'{ns}/uwb_pose_cov', 10)
        self.pose_pub = self.create_publisher(PoseStamped, f'{ns}/pose', 10)
        self.path_pub = self.create_publisher(Path, f'{ns}/path', 10)
        self.label_pub = self.create_publisher(Marker, f'{ns}/label', 10)
        # latched: true only once pose.orientation carries a real heading in the UWB frame
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.heading_valid_pub = self.create_publisher(Bool, f'{ns}/heading_valid', latched)
        self.heading_valid_pub.publish(Bool(data=False))
        # latched: false while the tag is silent (powered off, out of range); nothing is published then
        self.pose_valid_pub = self.create_publisher(Bool, f'{ns}/pose_valid', latched)
        self.set_pose_pub = self.create_publisher(PoseWithCovarianceStamped, f'{ns}/set_pose', 10)
        self.robot = args.robot
        self.timeout = args.timeout
        self.reset_distance = args.reset_distance
        self.last_uwb = None         # monotonic time of the last raw pose
        self.pose_valid = None       # last published pose_valid
        self.ekf_xy = None           # last EKF position
        self.jumps = 0               # consecutive raw poses far from the EKF
        self.reset_xy = None         # after a reset: drop EKF output until it is at this position
        self.create_subscription(PoseStamped, f'/uwb/{args.robot}/pose', self.on_uwb, qos_profile_sensor_data)
        self.create_subscription(Odometry, f'{ns}/odometry/filtered', self.on_odom, 10)
        self.create_timer(0.1, self.check_fresh)
        self.get_logger().info(f'/uwb/{args.robot}/pose -> ekf -> {ns}/pose  (UWB std {args.std} m)')

    def fresh(self):
        return self.last_uwb is not None and time.monotonic() - self.last_uwb <= self.timeout

    def check_fresh(self):
        valid = self.fresh()
        if valid != self.pose_valid:
            self.pose_valid = valid
            self.pose_valid_pub.publish(Bool(data=valid))
            if not valid and self.last_uwb is not None:
                self.get_logger().warn(f'no UWB pose for {self.timeout:g} s, {self.robot} pose invalid')

    def reset_ekf(self, msg, why):
        """Put the EKF straight onto the raw pose (velocities zeroed) instead of letting it slide."""
        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.pose.pose.position = msg.pose.position
        out.pose.pose.orientation.w = 1.0
        for i in range(6):
            out.pose.covariance[i * 7] = self.var if i < 3 else 1e3
        self.set_pose_pub.publish(out)
        self.reset_xy = (msg.pose.position.x, msg.pose.position.y)
        self.path.poses.clear()
        self.get_logger().info(f'EKF reset to ({self.reset_xy[0]:.2f}, {self.reset_xy[1]:.2f}): {why}')

    def on_uwb(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        if not self.fresh():
            # first pose, or the tag is back after an outage: do not drift over from the old state
            self.reset_ekf(msg, 'first pose' if self.last_uwb is None else 'UWB back after an outage')
            self.jumps = 0
        elif self.ekf_xy is not None and self.reset_xy is None:
            # a persistent jump (tag restarted, robot carried); single outliers stay with the EKF gate
            far = math.hypot(x - self.ekf_xy[0], y - self.ekf_xy[1]) > self.reset_distance
            self.jumps = self.jumps + 1 if far else 0
            if self.jumps >= 5:
                self.reset_ekf(msg, f'raw pose {self.reset_distance:g}+ m away from the EKF')
                self.jumps = 0
        self.last_uwb = time.monotonic()

        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.pose.pose = msg.pose
        # only x/y are fused (see uwb_ekf.yaml), the remaining diagonal just has to be valid
        for i in range(6):
            out.pose.covariance[i * 7] = self.var if i < 3 else 1e3
        self.cov_pub.publish(out)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.ekf_xy = (p.x, p.y)
        if self.reset_xy is not None:
            # the EKF may still emit its old state once or twice after set_pose
            if math.hypot(p.x - self.reset_xy[0], p.y - self.reset_xy[1]) > self.reset_distance:
                return
            self.reset_xy = None
        if not self.fresh():
            # the EKF keeps extrapolating without measurements: publish nothing while the tag is silent
            return
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose.pose
        out.pose.position.z = self.floor_z
        # UWB gives no heading: the EKF yaw only drifts through numerical coupling (seen ~13 deg),
        # so never pass it on; heading_valid (false) tells consumers there is none
        out.pose.orientation.x = out.pose.orientation.y = out.pose.orientation.z = 0.0
        out.pose.orientation.w = 1.0
        self.pose_pub.publish(out)

        # 10 Hz trail
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t - self.last_path_t >= 0.1:
            self.last_path_t = t
            self.path.header = msg.header
            self.path.poses.append(out)
            del self.path.poses[:-self.path_len]
            self.path_pub.publish(self.path)

            label = Marker()
            label.header = msg.header
            label.ns, label.id = 'uwb_ekf_label', 0
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x = out.pose.position.x
            label.pose.position.y = out.pose.position.y
            label.pose.position.z = self.floor_z + 0.55
            label.pose.orientation.w = 1.0
            label.scale.z = 0.25
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = self.robot
            # disappears when the tag stops reporting
            label.lifetime.sec = 1
            self.label_pub.publish(label)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--robot', default='rm_0')
    p.add_argument('--std', type=float, default=0.05, help='UWB position std fed to the EKF [m]')
    p.add_argument('--history', type=float, default=30.0, help='trail length [s]')
    p.add_argument('--floor-z', type=float, default=-1.75, help='floor height in the UWB frame [m]')
    p.add_argument('--timeout', type=float, default=0.5,
                   help='no raw UWB pose for this long -> pose_valid false, nothing published [s]')
    p.add_argument('--reset-distance', type=float, default=1.0,
                   help='reset the EKF when the raw pose stays this far from it [m]')
    args = p.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = UwbEkfAdapter(args)
    # a raising handler can fire inside rclpy's C code and surface as RuntimeError,
    # so only set a flag and leave the loop at the next check
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(True))
    try:
        # spin_once with a timeout: this node has no timers, so a plain spin() would block
        # forever (and never see the flag) once the UWB input stops
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
