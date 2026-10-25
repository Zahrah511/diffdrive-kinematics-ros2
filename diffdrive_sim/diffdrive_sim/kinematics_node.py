"""
kinematics_node — odometry computed from the MEASURED wheel speeds.

Pipeline:  VR, VL  ->  V, w  ->  ICC, R  ->  new pose (x, y, theta)
  * input     : /joint_states   (wheel spin rates from Gazebo, rad/s; times r -> m/s)
  * the pose is computed TWO ways at once: exact (rotation about the ICC) and Euler
  * reference : /ground_truth   (true pose from Gazebo)
  * output    : /path_exact, /path_euler, /path_truth (for RViz) and a CSV file

Example:
  ros2 run diffdrive_sim kinematics_node --ros-args -p use_sim_time:=true -p csv:=results/v1_straight.csv
"""
import csv
import math
import signal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

from diffdrive_sim import diffdrive_kin as kin


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class KinematicsNode(Node):
    def __init__(self):
        super().__init__("kinematics_node")
        self.declare_parameter("wheel_radius", 0.1)        # MUST match the SDF model
        self.declare_parameter("wheel_separation", 0.5)    # MUST match the SDF model
        self.declare_parameter("left_joint", "left_wheel_joint")
        self.declare_parameter("right_joint", "right_wheel_joint")
        self.declare_parameter("csv", "run.csv")
        g = lambda k: self.get_parameter(k).value
        self.r, self.L = g("wheel_radius"), g("wheel_separation")
        self.jl, self.jr = g("left_joint"), g("right_joint")

        self.pose_exact = (0.0, 0.0, 0.0)
        self.pose_euler = (0.0, 0.0, 0.0)
        self.truth = None
        self.t_truth = 0.0
        self.truth0 = (0.0, 0.0, 0.0)
        self.t_prev = None
        self.t_start = None
        self.last_path_t = -1.0
        self.moved, self.still_since, self.reported = False, None, False

        self.paths = {k: Path() for k in ("exact", "euler", "truth")}
        for p in self.paths.values():
            p.header.frame_id = "odom"
        self.pubs = {k: self.create_publisher(Path, f"/path_{k}", 10) for k in self.paths}

        self.create_subscription(JointState, "/joint_states", self.on_joints, 50)
        self.create_subscription(Odometry, "/ground_truth", self.on_truth, 50)

        self.f = open(g("csv"), "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "vr", "vl", "x_exact", "y_exact", "th_exact",
                         "x_euler", "y_euler", "th_euler", "x_true", "y_true", "th_true",
                         "t_true"])
        self.get_logger().info(f"L={self.L} m, r={self.r} m. Waiting for /joint_states ...")

    def on_truth(self, msg):
        p = msg.pose.pose
        self.truth = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        # When the true pose was sampled. It arrives at 50 Hz while wheel speeds arrive at
        # ~1000 Hz, so a fair comparison needs to know WHEN this pose was taken.
        self.t_truth = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_joints(self, msg):
        try:
            i_l, i_r = msg.name.index(self.jl), msg.name.index(self.jr)
        except ValueError:
            self.get_logger().warn(f"Joint names not found. Available: {list(msg.name)}",
                                   throttle_duration_sec=5.0)
            return
        if len(msg.velocity) <= max(i_l, i_r):
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        vl = msg.velocity[i_l] * self.r          # rad/s -> m/s (what an encoder would give)
        vr = msg.velocity[i_r] * self.r

        if self.t_prev is None:
            self.t_prev = self.t_start = t
            return
        dt = t - self.t_prev
        self.t_prev = t
        if dt <= 0.0:
            return

        # Print the result automatically 1 s (sim time) after the robot stops — no Ctrl+C needed.
        if max(abs(vr), abs(vl)) > 1e-3:
            if not self.moved and self.truth:
                self.truth0 = self.truth      # Gazebo pose when the robot STARTS moving
            self.moved, self.still_since, self.reported = True, None, False
        elif self.moved and not self.reported:
            if self.still_since is None:
                self.still_since = t
            elif t - self.still_since > 1.0:
                self.f.flush()
                self.report()
                self.reported = True

        self.pose_exact = kin.step_exact(self.pose_exact, vr, vl, self.L, dt)
        self.pose_euler = kin.step_euler(self.pose_euler, vr, vl, self.L, dt)

        tr = self.truth if self.truth else (float("nan"),) * 3
        t_tr = (self.t_truth - self.t_start) if self.truth else float("nan")
        self.w.writerow([f"{t - self.t_start:.4f}", f"{vr:.5f}", f"{vl:.5f}",
                         *[f"{v:.5f}" for v in (*self.pose_exact, *self.pose_euler, *tr)],
                         f"{t_tr:.4f}"])

        if t - self.last_path_t >= 0.05:              # add a Path point every 0.05 s
            self.last_path_t = t
            self.add_point("exact", self.pose_exact, msg.header.stamp)
            self.add_point("euler", self.pose_euler, msg.header.stamp)
            if self.truth:
                self.add_point("truth", self.relative(self.truth), msg.header.stamp)
            for k, p in self.paths.items():
                p.header.stamp = msg.header.stamp
                self.pubs[k].publish(p)
            self.f.flush()                            # CSV stays safe even if the node dies

    def add_point(self, key, pose, stamp):
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.header.stamp = stamp
        ps.pose.position.x, ps.pose.position.y = pose[0], pose[1]
        ps.pose.orientation.z = math.sin(pose[2] / 2)
        ps.pose.orientation.w = math.cos(pose[2] / 2)
        self.paths[key].poses.append(ps)

    def relative(self, p):
        """Gazebo pose expressed relative to the pose where the robot started moving."""
        x0, y0, th0 = self.truth0
        dx, dy = p[0] - x0, p[1] - y0
        th = math.atan2(math.sin(p[2] - th0), math.cos(p[2] - th0))
        return (math.cos(th0) * dx + math.sin(th0) * dy,
                -math.sin(th0) * dx + math.cos(th0) * dy, th)

    def report(self):
        # Plain print: on Ctrl+C the ROS logging system is already shut down.
        fmt = lambda p: (f"x={p[0]:.3f} y={p[1]:.3f} "
                         f"th={math.degrees(math.atan2(math.sin(p[2]), math.cos(p[2]))):.1f} deg")
        lines = ["", "===== RESULT =====",
                 "Final pose, exact : " + fmt(self.pose_exact),
                 "Final pose, Euler : " + fmt(self.pose_euler)]
        if self.truth:
            tr = self.relative(self.truth)
            lines.append("True pose (Gazebo): " + fmt(tr) + "  (relative to start)")
            d = math.dist(self.pose_exact[:2], tr[:2])
            lines.append(f"Position gap, exact vs true: {d * 100:.1f} cm")
        lines.append(f"CSV saved: {self.f.name}  (safe to press Ctrl+C now)")
        print("\n".join(lines), flush=True)


def main():
    rclpy.init()
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # ignore the second Ctrl+C from ros2 run
        try:
            node.f.close()          # close the CSV FIRST so the data is safe
        except BaseException:
            pass
        if not node.reported:
            try:
                node.report()
            except BaseException:
                pass
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except BaseException:
            pass
