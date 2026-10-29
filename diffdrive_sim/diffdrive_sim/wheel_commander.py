"""
wheel_commander — sends a speed to EACH WHEEL in Gazebo, following a variation scenario.

Every VR/VL [m/s] is computed by diffdrive_kin (our equations), then converted to rad/s:
    wheel spin rate = wheel speed / r

Examples:
  ros2 run diffdrive_sim wheel_commander --ros-args -p use_sim_time:=true -p scenario:=v1_straight
  ros2 run diffdrive_sim wheel_commander --ros-args -p use_sim_time:=true -p scenario:=v2_circle -p radius:=2.0 -p v:=0.7
  ros2 run diffdrive_sim wheel_commander --ros-args -p use_sim_time:=true -p scenario:=v5_arc -p goal_x:=2.0 -p goal_y:=1.0

Scenarios: v1_straight, v1_spin, v1_turn, v2_circle, v3_square, v3_figure8, v5_arc, v5_spin_straight
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from diffdrive_sim import diffdrive_kin as kin

def build_segments(p):
    """Turn a scenario name + parameters into a list of segments (VR, VL, duration)."""
    L, v, R, w_spin = p["L"], p["v"], p["radius"], p["w_spin"]
    s = p["scenario"]
    if s == "v1_straight":                      # VR = VL  ->  R infinite
        return [(v, v, 6.0)]
    if s == "v1_spin":                          # VR = -VL ->  R = 0, one full turn
        vr, vl = kin.inverse_spin(w_spin, L)
        return [(vr, vl, 2 * math.pi / w_spin)]
    if s == "v1_turn":                          # VR != VL ->  circle, one full turn
        vr, vl = 0.6, 0.4
        _, w = kin.forward(vr, vl, L)
        return [(vr, vl, 2 * math.pi / abs(w))]
    if s == "v2_circle":                        # inverse kinematics from a wanted R
        vr, vl = kin.inverse(v, R, L)
        return [(vr, vl, 2 * math.pi * abs(R) / v)]
    if s == "v3_square":
        return kin.plan_square(p["side"], v, w_spin, L)
    if s == "v3_figure8":
        return kin.plan_figure8(R, v, L)
    if s == "v5_arc":
        a = kin.plan_arc_to_goal(p["goal_x"], p["goal_y"], v, L)
        return [(a["VR"], a["VL"], a["t"])]
    if s == "v5_spin_straight":
        return kin.plan_spin_then_straight(p["goal_x"], p["goal_y"], v, w_spin, L)
    raise ValueError(f"Unknown scenario: {s}")


class WheelCommander(Node):
    def __init__(self):
        super().__init__("wheel_commander")
        defaults = dict(scenario="v1_straight", wheel_radius=0.1, wheel_separation=0.5,
                        v=0.5, radius=2.0, w_spin=1.2, side=2.0, goal_x=2.0, goal_y=1.0)
        for k, val in defaults.items():
            self.declare_parameter(k, val)
        g = lambda k: self.get_parameter(k).value
        self.r = g("wheel_radius")
        p = dict(scenario=g("scenario"), L=g("wheel_separation"), v=g("v"), radius=g("radius"),
                 w_spin=g("w_spin"), side=g("side"), goal_x=g("goal_x"), goal_y=g("goal_y"))

        self.segments = build_segments(p)
        name = p["scenario"]
        self.get_logger().info(f"Scenario {name}: {len(self.segments)} segment(s)")
        for i, (vr, vl, d) in enumerate(self.segments):
            V, w = kin.forward(vr, vl, p["L"])
            self.get_logger().info(
                f"  segment {i+1}: VR={vr:.4f} VL={vl:.4f} m/s  (V={V:.3f} m/s, w={w:.3f} rad/s)  "
                f"duration {d:.3f} s")

        self.pub_l = self.create_publisher(Float64, "/left_wheel/cmd", 10)
        self.pub_r = self.create_publisher(Float64, "/right_wheel/cmd", 10)
        self.t0 = None
        self.done = False
        self.stop_ticks = 0
        self.timer = self.create_timer(0.02, self.tick)    # 50 Hz, in simulation time

    def send(self, vr, vl):
        self.pub_l.publish(Float64(data=vl / self.r))       # m/s -> rad/s
        self.pub_r.publish(Float64(data=vr / self.r))

    def tick(self):
        if self.done:
            self.send(0.0, 0.0)
            self.stop_ticks += 1
            if self.stop_ticks >= 25:                        # send zero for 0.5 s, then exit
                raise SystemExit
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now == 0.0:                                       # /clock not received yet
            return
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info("Moving.")
        t = now - self.t0
        if t < 0.0:
            # Simulation clock went backwards = Gazebo restarted. Stop the wheels.
            self.get_logger().warn("Simulation clock went backwards (Gazebo restarted?). Wheels stopped.")
            self.finish()
            return
        for vr, vl, d in self.segments:
            if t < d:
                self.send(vr, vl)
                return
            t -= d
        self.get_logger().info("All segments done, wheels stopped. Node exits by itself.")
        self.finish()

    def finish(self):
        self.send(0.0, 0.0)
        self.done = True
        self.stop_ticks = 0


def main():
    rclpy.init()
    node = WheelCommander()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, Exception):
        pass
    finally:
        try:
            node.send(0.0, 0.0)
        except BaseException:
            pass
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except BaseException:
            pass
        print("wheel_commander finished.", flush=True)
