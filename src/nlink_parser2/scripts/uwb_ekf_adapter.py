#!/usr/bin/env python3
"""Kalman filter of one robot's UWB position, optionally aided by the robot's own velocity.

  /uwb/<robot>/pose (PoseStamped, raw UWB)
      -> /uwb_ekf/<robot>/pose (PoseStamped, filtered, on the floor)
      -> /uwb_ekf/<robot>/odometry/filtered (Odometry, position + velocity in the UWB frame)
      -> /uwb_ekf/<robot>/path (Path, last --history seconds, for RViz)
      -> /uwb_ekf/<robot>/label (Marker, robot name above it, for RViz)
      -> /uwb_ekf/<robot>/heading_valid (Bool, latched): false, pose.orientation is identity
      -> /uwb_ekf/<robot>/pose_valid (Bool, latched): false while no raw pose for --timeout s;
         pose / path / label are not published then
  optional, --vel-topic and --heading-topic:
  /<robot>/odom/twist (TwistStamped, body frame: x forward, y left) + /<robot>/odometry/filtered
  (Odometry, IMU heading, ENU from magnetic east) -> fused velocity measurement
      -> /uwb_ekf/<robot>/heading_offset (Float32, deg, latched): the offset in use

Filter: x y vx vy ax ay, constant acceleration, process noise Q * dt, x/y measurements with a
Mahalanobis gate. Without a velocity it behaves like the former robot_localization setup
(uwb_ekf.yaml). Body velocities are turned into the UWB frame (mirrored) as
    v_uwb = Rot(heading_offset - yaw_imu) * diag(1, -1) * v_body
With them the UWB is trusted less (a blocked anchor drags the raw position for seconds,
the odometry does not follow it) and the gate is tighter. heading_offset comes from
config/uwb_velocity.yaml; without it, it is estimated once from straight drives (UWB
displacement vs integrated odometry). It is not refined online: the IMU heading of dog_4 drifted
while its magnetometer was rejected and an online estimate followed the drift (2026-10-05);
the fix belongs to the IMU (gyro bias removal in ybimu_driver).

The filter is reset onto the raw pose when the tag comes back after an outage or the raw pose
stays --reset-distance away for --reset-time (tag restarted, robot carried).
"""

import argparse
import collections
import math
import os
import signal
import sys
import time

# tiny matrices only: a multi-threaded BLAS spins every one of its threads (one core each, 100 %)
for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')
import numpy as np  # noqa: E402

# filter settings, tuned on dog_4 recordings against mocap (2026-10-05)
POS_ONLY = dict(std=0.05, q_vel=2e-2, gate=5.0)      # = the former robot_localization config
WITH_VEL = dict(std=0.3, q_vel=1e-3, gate=3.0)
Q_POS = 1e-3
Q_ACC = 5e-2
# the robot's own velocity says it stands still: the position must not move at all, the UWB
# readings are only averaged (a parked robot otherwise wanders with the UWB multipath)
STILL = dict(std=0.3, q_vel=1e-6, gate=3.0, q_pos=1e-6, q_acc=1e-6)
STILL_TIME = 0.5          # s below STILL_SPEED before STILL applies
VEL_STD = 0.08            # body velocity measurement std [m/s]
VEL_TIMEOUT = 1.0         # s without velocity -> back to POS_ONLY (WiFi drops them for up to ~1 s)
HEADING_TIMEOUT = 0.5     # s without IMU heading -> only standstill is used
STILL_SPEED = 0.02        # m/s, below it the robot stands still (fused without heading)

MIRROR = np.diag([1.0, -1.0])


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


class UwbFilter:
    """Position Kalman filter, plain numpy (also used offline by the replay tools)."""

    def __init__(self):
        self.x = None
        self.P = None
        self.t = None

    def reset(self, t, xy, std):
        self.x = np.zeros(6)
        self.x[:2] = xy
        self.P = np.diag([std ** 2, std ** 2, 1.0, 1.0, 1.0, 1.0])
        self.t = t

    def predict(self, t, q_vel, q_pos=Q_POS, q_acc=Q_ACC):
        dt = t - self.t
        if dt <= 0:
            return
        self.t = t
        F = np.eye(6)
        F[0, 2] = F[1, 3] = dt
        F[0, 4] = F[1, 5] = 0.5 * dt * dt
        F[2, 4] = F[3, 5] = dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + np.diag([q_pos, q_pos, q_vel, q_vel, q_acc, q_acc]) * dt

    def _update(self, idx, z, var, gate=0.0):
        H = np.zeros((2, 6))
        H[0, idx] = H[1, idx + 1] = 1.0
        y = z - self.x[idx:idx + 2]
        S = H @ self.P @ H.T + np.eye(2) * var
        if gate > 0 and math.sqrt(y @ np.linalg.solve(S, y)) > gate:
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
        return True

    def update_pos(self, xy, std, gate):
        return self._update(0, xy, std ** 2, gate)

    def update_vel(self, v):
        self._update(2, v, VEL_STD ** 2)

    def hold(self):
        """standing still: velocity and acceleration are zero and certain, the position only averages"""
        self.x[2:6] = 0.0
        self.P[2:6, :] = 0.0
        self.P[:, 2:6] = 0.0
        self.P[2:6, 2:6] = np.eye(4) * 1e-6

    def release(self):
        """moving again: velocity and acceleration are free to follow the measurements at once"""
        self.P[2, 2] = self.P[3, 3] = VEL_STD ** 2
        self.P[4, 4] = self.P[5, 5] = 1.0


class HeadingOffset:
    """heading_offset from straight drives: UWB displacement vs the integrated body velocity
    rotated by the IMU heading only (v_uwb = Rot(offset) w, w = Rot(-yaw) diag(1, -1) v_body)."""

    WINDOW = 3.0          # s
    MIN_DIST = 0.5        # m travelled in the window
    STRAIGHT = 0.95       # |displacement| / path length of the odometry
    SAMPLES = 60
    MIN_SAMPLES = 15

    def __init__(self, offset_deg=None):
        self.offset = None if offset_deg is None else math.radians(offset_deg)
        self.uwb = collections.deque()        # (t, x, y)
        self.odo = collections.deque()        # (t, Wx, Wy, path)  integrated w
        self.W = np.zeros(2)
        self.path = 0.0
        self.t_odo = None
        self.samples = collections.deque(maxlen=self.SAMPLES)
        self.last_eval = 0.0

    def add_uwb(self, t, xy):
        self.uwb.append((t, xy[0], xy[1]))
        while self.uwb and self.uwb[0][0] < t - 2 * self.WINDOW:
            self.uwb.popleft()

    def add_velocity(self, t, w):
        if self.t_odo is not None and 0 < t - self.t_odo < 0.5:
            dt = t - self.t_odo
            self.W = self.W + w * dt
            self.path += float(np.hypot(*w)) * dt
        self.t_odo = t
        self.odo.append((t, self.W[0], self.W[1], self.path))
        while self.odo and self.odo[0][0] < t - 2 * self.WINDOW:
            self.odo.popleft()

    def _uwb_at(self, t):
        """median raw position over 0.3 s around t (the raw pose is noisy)"""
        pts = [(x, y) for tt, x, y in self.uwb if abs(tt - t) <= 0.15]
        return np.median(np.array(pts), axis=0) if len(pts) >= 5 else None

    def _odo_at(self, t):
        best = min(self.odo, key=lambda o: abs(o[0] - t), default=None)
        return None if best is None or abs(best[0] - t) > 0.05 else best

    def evaluate(self, t):
        """call periodically; returns True when the offset changed"""
        if t - self.last_eval < 0.5 or not self.uwb or not self.odo:
            return False
        self.last_eval = t
        t1 = min(self.uwb[-1][0], self.odo[-1][0]) - 0.15
        t0 = t1 - self.WINDOW
        u0, u1, o0, o1 = self._uwb_at(t0), self._uwb_at(t1), self._odo_at(t0), self._odo_at(t1)
        if u0 is None or u1 is None or o0 is None or o1 is None:
            return False
        U = u1 - u0
        Wd = np.array([o1[1] - o0[1], o1[2] - o0[2]])
        path = o1[3] - o0[3]
        dw = float(np.hypot(*Wd))
        if dw < self.MIN_DIST or dw / max(path, 1e-6) < self.STRAIGHT:
            return False
        ratio = float(np.hypot(*U)) / dw
        if not 0.7 < ratio < 1.3:
            return False
        self.samples.append(math.atan2(U[1], U[0]) - math.atan2(Wd[1], Wd[0]))
        if len(self.samples) < self.MIN_SAMPLES:
            return False
        # circular median, then the mean of the samples near it
        s = np.array(self.samples)
        med = min(s, key=lambda a: np.sum(np.abs(np.angle(np.exp(1j * (s - a))))))
        near = s[np.abs(np.angle(np.exp(1j * (s - med)))) < math.radians(15)]
        est = float(np.angle(np.mean(np.exp(1j * near))))
        if self.offset is None or abs(math.remainder(est - self.offset, 2 * math.pi)) > math.radians(1.0):
            self.offset = est
            return True
        return False


class Tracker:
    """Filter + reset logic for one robot, fed with timestamped events (ROS node and replay)."""

    def __init__(self, timeout=0.5, reset_distance=1.0, reset_time=0.1, reset_time_vel=1.0,
                 heading_offset=None):
        self.f = UwbFilter()
        self.timeout = timeout
        self.reset_distance = reset_distance
        self.reset_time = reset_time
        self.reset_time_vel = reset_time_vel
        # configured offset; without one it is taken from the straight-drive estimate once. The
        # estimate keeps running as a check of the configured value (it is not applied)
        self.offset0 = None if heading_offset is None else math.radians(heading_offset)
        self.estimate = HeadingOffset()
        self.last_uwb = None
        self.last_vel = None
        self.yaw = None                # IMU heading (ENU) and its time
        self.t_yaw = None
        self.still_since = None        # since when the robot's velocity says standstill
        self.holding = False           # filter held in STILL
        self.far_since = None
        self.resets = []

    def fresh(self, t):
        return self.last_uwb is not None and t - self.last_uwb <= self.timeout

    def vel_mode(self, t):
        return self.last_vel is not None and t - self.last_vel <= VEL_TIMEOUT

    def mode(self, t):
        if not self.vel_mode(t):
            return POS_ONLY
        if self.still_since is not None and t - self.still_since >= STILL_TIME:
            return STILL
        return WITH_VEL

    def predict(self, t):
        m = self.mode(t)
        if (m is STILL) != self.holding:
            self.holding = m is STILL
            self.f.hold() if self.holding else self.f.release()
        self.f.predict(t, m['q_vel'], m.get('q_pos', Q_POS), m.get('q_acc', Q_ACC))
        return m

    def heading_offset(self):
        """offset in use [rad], None until configured / bootstrapped"""
        return self.offset0

    def on_heading(self, t, yaw):
        self.yaw, self.t_yaw = yaw, t

    def on_velocity(self, t, v_body):
        """body velocity (x forward, y left); returns False when it could not be used"""
        v_body = np.asarray(v_body, float)
        if np.hypot(*v_body) < STILL_SPEED:
            self.still_since = t if self.still_since is None else self.still_since
        else:
            self.still_since = None
        v_uwb = None
        if self.yaw is not None and t - self.t_yaw <= HEADING_TIMEOUT:
            w = rot(-self.yaw) @ MIRROR @ v_body
            self.estimate.add_velocity(t, w)
            self.estimate.evaluate(t)
            if self.offset0 is None:
                self.offset0 = self.estimate.offset
            if self.offset0 is not None:
                v_uwb = rot(self.offset0) @ w
        if v_uwb is None and np.hypot(*v_body) < STILL_SPEED:
            # standing still needs neither the heading nor its offset: this alone stops the UWB
            # noise from moving a parked robot
            v_uwb = np.zeros(2)
        # also during a UWB outage: the odometry carries the state over it
        if v_uwb is None or self.f.x is None:
            return False
        self.last_vel = t
        self.predict(t)
        self.f.update_vel(v_uwb)
        return True

    def on_uwb(self, t, xy):
        """returns a reset reason or None"""
        xy = np.asarray(xy, float)
        self.estimate.add_uwb(t, xy)
        mode = self.mode(t)
        reason = None
        if self.last_uwb is None or not (self.fresh(t) or self.vel_mode(t)):
            # first pose, or the tag is back after an outage without odometry: do not drift over
            # from the old state (with odometry the state is still good, the far check below applies)
            reason = 'first pose' if self.last_uwb is None else 'UWB back after an outage'
        else:
            self.predict(t)
            if np.hypot(*(xy - self.f.x[:2])) > self.reset_distance:
                self.far_since = t if self.far_since is None else self.far_since
                hold = self.reset_time if mode is POS_ONLY else self.reset_time_vel
                if t - self.far_since >= hold:
                    reason = f'raw pose {self.reset_distance:g}+ m away for {hold:g} s'
            else:
                self.far_since = None
        self.last_uwb = t
        if reason:
            self.f.reset(t, xy, mode['std'])
            self.holding = False
            self.far_since = None
            self.resets.append(t)
        else:
            self.f.update_pos(xy, mode['std'], mode['gate'])
        return reason


def main():
    import rclpy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav_msgs.msg import Odometry, Path
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Bool, Float32
    from visualization_msgs.msg import Marker

    class UwbEkfAdapter(Node):
        def __init__(self, args):
            super().__init__(f'uwb_ekf_adapter_{args.robot}')
            ns = f'/uwb_ekf/{args.robot}'
            self.robot = args.robot
            self.floor_z = args.floor_z
            self.frame = 'world'
            self.path = Path()
            self.path_len = int(args.history * 10)
            self.last_path_t = 0.0
            offset = self.args_offset = args.heading_offset
            self.tr = Tracker(args.timeout, args.reset_distance, args.reset_time, args.reset_time_vel, offset)
            self.published_offset = None
            POS_ONLY['std'] = args.std
            self.pose_pub = self.create_publisher(PoseStamped, f'{ns}/pose', 10)
            self.odom_pub = self.create_publisher(Odometry, f'{ns}/odometry/filtered', 10)
            self.path_pub = self.create_publisher(Path, f'{ns}/path', 10)
            self.label_pub = self.create_publisher(Marker, f'{ns}/label', 10)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            # latched: true only once pose.orientation carries a real heading in the UWB frame
            self.heading_valid_pub = self.create_publisher(Bool, f'{ns}/heading_valid', latched)
            self.heading_valid_pub.publish(Bool(data=False))
            # latched: false while the tag is silent (powered off, out of range); nothing is published then
            self.pose_valid_pub = self.create_publisher(Bool, f'{ns}/pose_valid', latched)
            self.pose_valid = None
            self.create_subscription(PoseStamped, f'/uwb/{args.robot}/pose', self.on_uwb, qos_profile_sensor_data)
            if args.vel_topic and args.heading_topic:
                self.offset_pub = self.create_publisher(Float32, f'{ns}/heading_offset', latched)
                self.create_subscription(TwistStamped, args.vel_topic, self.on_vel, qos_profile_sensor_data)
                self.create_subscription(Odometry, args.heading_topic, self.on_heading, qos_profile_sensor_data)
                self.vel_used = None
                self.create_timer(0.5, self.check_heading_offset)
                self.get_logger().info(f'{args.vel_topic} + {args.heading_topic} fused, heading offset '
                                       f'{"unknown (estimated while driving)" if offset is None else f"{offset:.1f} deg"}')
            self.create_timer(0.1, self.check_fresh)
            self.get_logger().info(f'/uwb/{args.robot}/pose -> filter -> {ns}/pose  (UWB std {args.std} m)')

        def now(self):
            return time.monotonic()

        def check_heading_offset(self):
            off = self.tr.heading_offset()
            if off is None:
                return
            deg = math.degrees(math.remainder(off, 2 * math.pi))
            if self.published_offset is None or abs(deg - self.published_offset) >= 0.5:
                if self.published_offset is None and self.args_offset is None:
                    self.get_logger().info(f'heading offset estimated from straight drives: {deg:.1f} deg '
                                           '(put it into config/uwb_velocity.yaml)')
                self.published_offset = deg
                self.offset_pub.publish(Float32(data=deg))
            est = self.tr.estimate.offset
            if est is not None and abs(math.remainder(est - off, 2 * math.pi)) > math.radians(10.0):
                self.get_logger().warn(f'straight drives say heading offset {math.degrees(est):.1f} deg, '
                                       f'{deg:.1f} deg in use: IMU remounted / recalibrated? fix config/uwb_velocity.yaml',
                                       throttle_duration_sec=60.0)

        def check_fresh(self):
            valid = self.tr.fresh(self.now())
            if valid != self.pose_valid:
                self.pose_valid = valid
                self.pose_valid_pub.publish(Bool(data=valid))
                if not valid and self.tr.last_uwb is not None:
                    self.get_logger().warn(f'no UWB pose for {self.tr.timeout:g} s, {self.robot} pose invalid')

        def on_heading(self, msg):
            q = msg.pose.pose.orientation
            self.tr.on_heading(self.now(), math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

        def on_vel(self, msg):
            used = self.tr.on_velocity(self.now(), (msg.twist.linear.x, msg.twist.linear.y))
            if used != self.vel_used:
                self.vel_used = used
                self.get_logger().info('velocity fused' if used else 'velocity not used (no IMU heading / offset yet)')

        def on_uwb(self, msg):
            self.frame = msg.header.frame_id or self.frame
            reason = self.tr.on_uwb(self.now(), (msg.pose.position.x, msg.pose.position.y))
            if reason:
                self.path.poses.clear()
                x = self.tr.f.x
                self.get_logger().info(f'filter reset to ({x[0]:.2f}, {x[1]:.2f}): {reason}')
            self.publish()

        def publish(self):
            x = self.tr.f.x
            stamp = self.get_clock().now().to_msg()
            out = PoseStamped()
            out.header.stamp = stamp
            out.header.frame_id = self.frame
            out.pose.position.x, out.pose.position.y = float(x[0]), float(x[1])
            out.pose.position.z = self.floor_z
            # UWB gives no heading: identity orientation, heading_valid (false) says so
            out.pose.orientation.w = 1.0
            self.pose_pub.publish(out)

            odom = Odometry()
            odom.header = out.header
            odom.child_frame_id = f'{self.robot}/uwb'
            odom.pose.pose = out.pose
            odom.twist.twist.linear.x, odom.twist.twist.linear.y = float(x[2]), float(x[3])
            P = self.tr.f.P
            for i, j in ((0, 0), (0, 1), (1, 0), (1, 1)):
                odom.pose.covariance[i * 6 + j] = float(P[i, j])
                odom.twist.covariance[i * 6 + j] = float(P[i + 2, j + 2])
            self.odom_pub.publish(odom)

            # 10 Hz trail
            t = stamp.sec + stamp.nanosec * 1e-9
            if t - self.last_path_t >= 0.1:
                self.last_path_t = t
                self.path.header = out.header
                self.path.poses.append(out)
                del self.path.poses[:-self.path_len]
                self.path_pub.publish(self.path)

                label = Marker()
                label.header = out.header
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

    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--robot', default='rm_0')
    p.add_argument('--std', type=float, default=0.05, help='UWB position std without velocity [m]')
    p.add_argument('--history', type=float, default=30.0, help='trail length [s]')
    p.add_argument('--floor-z', type=float, default=-1.75, help='floor height in the UWB frame [m]')
    p.add_argument('--timeout', type=float, default=0.5,
                   help='no raw UWB pose for this long -> pose_valid false, nothing published [s]')
    p.add_argument('--reset-distance', type=float, default=1.0,
                   help='reset the filter when the raw pose stays this far from it [m] ...')
    p.add_argument('--reset-time', type=float, default=0.1, help='... for this long [s]')
    p.add_argument('--reset-time-vel', type=float, default=1.0,
                   help='... for this long while the velocity is fused [s]')
    p.add_argument('--vel-topic', default='', help='body velocity (TwistStamped), e.g. /dog_4/odom/twist')
    p.add_argument('--heading-topic', default='', help='IMU heading (Odometry), e.g. /dog_4/odometry/filtered')
    p.add_argument('--heading-offset', type=float, default=None,
                   help='start value of the heading offset [deg] (~/.ros/uwb_heading_offset_<robot>.yaml wins)')
    args = p.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = UwbEkfAdapter(args)
    # a raising handler can fire inside rclpy's C code and surface as RuntimeError,
    # so only set a flag and leave the loop at the next check
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(True))
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
