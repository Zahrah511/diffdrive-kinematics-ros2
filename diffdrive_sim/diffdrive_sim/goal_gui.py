"""
goal_gui — Variation 5, interactive map: click a goal point, see the plan, press GO,
watch the real robot path (Gazebo ground truth) being drawn on top of the plan.

  * click on the map            -> goal point (world frame)
  * world -> robot frame         x = cosθ·Δx + sinθ·Δy,  y = −sinθ·Δx + cosθ·Δy
  * Strategy A (single arc)      R = (x² + y²)/(2y),  Δθ = 2·atan2(y, x),  t = R·Δθ/V
  * Strategy B (spin, straight)  α = atan2(y, x),  d = √(x² + y²)
  * segments are timed in SIMULATION time (/clock)
  * scroll on the map to zoom, "Centre" to recentre on the robot

  ros2 run diffdrive_sim goal_gui
Run kinematics_node at the same time to record the run in a CSV.
"""
import math
import tkinter as tk
import tkinter.font as tkfont

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

from diffdrive_sim import diffdrive_kin as kin

L, r = 0.5, 0.1          # MUST match the robot model in the SDF
W_SPIN = 0.5             # spin rate for strategy B [rad/s]; slow = less overshoot
PAUSE = 0.5              # stop between spin and straight [s]: both wheels restart from 0
BG, INK, MUTED = "#F4F6F7", "#18222E", "#4A5868"
GRN, ORA, PUR, RED, GRID = "#137A5C", "#C2541F", "#5B4BC4", "#D1352B", "#E3E8EC"
BASE_W, BASE_H = 940, 480


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RosIO(Node):
    def __init__(self):
        super().__init__("goal_gui",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pl = self.create_publisher(Float64, "/left_wheel/cmd", 10)
        self.pr = self.create_publisher(Float64, "/right_wheel/cmd", 10)
        self.pose = None
        self.create_subscription(Odometry, "/ground_truth", self.on_truth, 10)

    def on_truth(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def send(self, vr, vl):
        self.pl.publish(Float64(data=vl / r))
        self.pr.publish(Float64(data=vr / r))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


class App:
    def __init__(self, ros):
        self.ros = ros
        self.goal = None            # world frame (xg, yg)
        self.preview = None         # dict: segs, pts, icc, lines
        self.segments, self.t0 = [], None
        self.trail = []
        self.view = None            # world point at the canvas centre
        self.span = 8.0             # metres across the shorter canvas side
        self.arrived = False

        self.root = tk.Tk()
        self.root.title("Variation 5: click a goal point")
        self.root.geometry(f"{BASE_W}x{BASE_H}")
        self.root.minsize(560, 300)
        self.root.configure(bg=BG)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.fH = tkfont.Font(family="DejaVu Sans", size=13, weight="bold")
        self.fT = tkfont.Font(family="DejaVu Sans", size=11)
        self.fM = tkfont.Font(family="DejaVu Sans Mono", size=11)
        self.fS = tkfont.Font(family="DejaVu Sans", size=10)

        self.root.columnconfigure(0, weight=1, uniform="c")
        self.root.columnconfigure(1, weight=1, uniform="c")
        self.root.rowconfigure(0, weight=1)

        self.cv = tk.Canvas(self.root, bg="white", highlightthickness=1,
                            highlightbackground="#D5DCE2", cursor="crosshair")
        self.cv.grid(row=0, column=0, sticky="nsew", padx=(10, 6), pady=10)
        self.cv.bind("<Button-1>", self.on_click)
        self.cv.bind("<MouseWheel>", lambda e: self.zoom(-1 if e.delta > 0 else 1))
        self.cv.bind("<Button-4>", lambda e: self.zoom(-1))
        self.cv.bind("<Button-5>", lambda e: self.zoom(1))

        right = tk.Frame(self.root, bg=BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(4, 10), pady=10)

        row1 = tk.Frame(right, bg=BG)
        row1.pack(anchor="w", fill="x")
        self.strat = tk.StringVar(value="A")
        tk.Radiobutton(row1, text="A: single arc", value="A", variable=self.strat, font=self.fT,
                       bg=BG, command=self.refresh_plan).pack(side="left")
        tk.Radiobutton(row1, text="B: spin + straight", value="B", variable=self.strat, font=self.fT,
                       bg=BG, command=self.refresh_plan).pack(side="left", padx=(8, 0))

        row2 = tk.Frame(right, bg=BG)
        row2.pack(anchor="w", fill="x")
        tk.Label(row2, text="V [m/s]", font=self.fT, bg=BG, fg=MUTED).pack(side="left")
        self.V = tk.DoubleVar(value=0.5)
        self.sV = tk.Scale(row2, from_=0.1, to=1.0, resolution=0.05, orient="horizontal", width=12,
                           variable=self.V, font=self.fS, bg=BG, highlightthickness=0,
                           troughcolor="#DCD7F7", showvalue=True, command=lambda _: self.refresh_plan())
        self.sV.pack(side="left", fill="x", expand=True, padx=(6, 0))

        row3 = tk.Frame(right, bg=BG)
        row3.pack(anchor="w", fill="x", pady=(2, 6))
        self.btn = tk.Button(row3, text="▶ GO", font=self.fH, bg=GRN, fg="white", activebackground=GRN,
                             command=self.go, width=7)
        self.btn.pack(side="left")
        tk.Button(row3, text="Clear trail", font=self.fT, command=self.clear).pack(side="left", padx=6)
        tk.Button(row3, text="Centre", font=self.fT, command=self.centre).pack(side="left")
        row4 = tk.Frame(right, bg=BG)
        row4.pack(anchor="w", fill="x", pady=(0, 6))
        tk.Button(row4, text="★ Goal: 2 m ahead, 1 m left (report point)", font=self.fT,
                  command=self.preset_goal).pack(side="left", fill="x", expand=True)

        box = tk.Frame(right, bg="white", highlightbackground="#D5DCE2", highlightthickness=1)
        box.pack(fill="both", expand=True)
        self.eq = tk.Label(box, text="", font=self.fM, bg="white", fg=INK, justify="left", anchor="nw")
        self.eq.pack(anchor="nw", fill="both", expand=True, padx=10, pady=(8, 2))
        self.status = tk.Label(box, text="", font=self.fH, bg="white", fg=ORA, anchor="w")
        self.status.pack(anchor="w", fill="x", padx=10, pady=(0, 8))

        self._last = None
        self.root.bind("<Configure>", self.on_resize)
        self.set_text(["Click the map to set a goal point."])
        self.root.after(20, self.tick)

    # ------------------------------------------------------------- layout
    def on_resize(self, event):
        if event.widget is not self.root:
            return
        size = (event.width, event.height)
        if size == self._last:
            return
        self._last = size
        s = max(0.55, min(event.width / BASE_W, event.height / BASE_H))
        self.fH.configure(size=max(8, round(13 * s)))
        self.fT.configure(size=max(7, round(11 * s)))
        self.fM.configure(size=max(7, round(11 * s)))
        self.fS.configure(size=max(7, round(10 * s)))

    def set_text(self, lines):
        self.eq.configure(text="\n".join(lines))

    # ------------------------------------------------------------- map maths
    def mapping(self):
        w, h = max(self.cv.winfo_width(), 50), max(self.cv.winfo_height(), 50)
        k = min(w, h) / self.span
        vx, vy = self.view if self.view else (0.0, 0.0)
        return w / 2, h / 2, k, vx, vy

    def w2c(self, X, Y):
        cx, cy, k, vx, vy = self.mapping()
        return cx + (X - vx) * k, cy - (Y - vy) * k

    def c2w(self, px, py):
        cx, cy, k, vx, vy = self.mapping()
        return vx + (px - cx) / k, vy - (py - cy) / k

    def zoom(self, d):
        self.span = min(30.0, max(2.0, self.span * (1.25 if d > 0 else 0.8)))

    def centre(self):
        if self.ros.pose:
            self.view = self.ros.pose[:2]

    def clear(self):
        self.trail = []

    # ------------------------------------------------------------- planning
    def on_click(self, e):
        if self.segments:
            return
        self.goal = self.c2w(e.x, e.y)
        self.arrived = False
        self.refresh_plan()

    def preset_goal(self):
        """Exact goal used in the report: 2 m ahead and 1 m to the left of the robot."""
        if self.segments or not self.ros.pose:
            return
        x0, y0, th = self.ros.pose
        self.goal = (x0 + 2.0 * math.cos(th) - 1.0 * math.sin(th),
                     y0 + 2.0 * math.sin(th) + 1.0 * math.cos(th))
        self.arrived = False
        self.refresh_plan()

    def make_plan(self, pose):
        xg, yg = self.goal
        x0, y0, th = pose
        dx, dy = xg - x0, yg - y0
        x, y = kin.world_to_robot(self.goal, pose)
        V = self.V.get()
        head = [f"Goal (world)  xg={xg:+.2f}  yg={yg:+.2f}",
                f"Robot (world) x={x0:+.2f} y={y0:+.2f} θ={math.degrees(th):+.0f}°",
                "World → robot frame:",
                f" x = cosθ·Δx + sinθ·Δy = {x:+.2f} m",
                f" y = −sinθ·Δx + cosθ·Δy = {y:+.2f} m", ""]
        if self.strat.get() == "A":
            a = kin.plan_arc_to_goal(x, y, V, L)
            segs = [(a["VR"], a["VL"], a["t"])]
            if math.isinf(a["R"]):
                icc, body = None, ["A: y = 0 → straight, R = ∞",
                                   f"t = x/V = {a['t']:.2f} s   VR = VL = {V:.2f}"]
                arrive = th
            else:
                icc = (x0 - a["R"] * math.sin(th), y0 + a["R"] * math.cos(th))
                body = [f"A: R = (x²+y²)/(2y) = {a['R']:+.3f} m",
                        f"   Δθ = 2·atan(y/x) = {math.degrees(a['dtheta']):+.1f}°",
                        f"   t = R·Δθ/V = {a['t']:.2f} s",
                        f"   VR = ω(R+L/2) = {a['VR']:+.3f} m/s",
                        f"   VL = ω(R−L/2) = {a['VL']:+.3f} m/s"]
                arrive = th + a["dtheta"]
            body.append(f"Arrival heading: {math.degrees(a['dtheta']):+.1f}° (turned)")
        else:
            spin, straight = kin.plan_spin_then_straight(x, y, V, W_SPIN, L)
            segs = [spin, (0.0, 0.0, PAUSE), straight]
            alpha, d = math.atan2(y, x), math.hypot(x, y)
            icc = None
            body = [f"B: α = atan(y/x) = {math.degrees(alpha):+.1f}°",
                    f"   1) spin VR={spin[0]:+.3f} VL={spin[1]:+.3f}, {spin[2]:.2f} s",
                    f"   2) stop {PAUSE:.1f} s (both wheels restart from 0)",
                    f"   d = √(x²+y²) = {d:.3f} m",
                    f"   3) straight V={V:.2f}, {straight[2]:.2f} s",
                    f"Arrival heading: {math.degrees(alpha):+.1f}° (turned)"]
            arrive = th + alpha
        pts = kin.run_segments(pose, segs, L, 0.02)
        return dict(segs=segs, pts=[p[:2] for p in pts], icc=icc, lines=head + body, arrive=arrive)

    def refresh_plan(self):
        if self.segments or not self.goal:
            return
        if not self.ros.pose:
            self.set_text(["Waiting for /ground_truth …", "(is the bridge running?)"])
            return
        try:
            self.preview = self.make_plan(self.ros.pose)
            self.set_text(self.preview["lines"])
        except ValueError as e:
            self.preview = None
            self.set_text([str(e)])

    def go(self):
        if self.segments or not self.goal or not self.ros.pose:
            return
        self.refresh_plan()
        if not self.preview:
            return
        self.segments, self.t0 = list(self.preview["segs"]), None
        self.btn.configure(state="disabled")

    # ------------------------------------------------------------- loop
    def tick(self):
        rclpy.spin_once(self.ros, timeout_sec=0)
        pose = self.ros.pose
        if pose:
            if not self.trail or math.dist(self.trail[-1], pose[:2]) > 0.01:
                self.trail.append(pose[:2])
                self.trail = self.trail[-6000:]
            if self.view is None:
                self.view = pose[:2]
        if self.segments:
            now = self.ros.now()
            if now > 0.0 and self.t0 is None:
                self.t0 = now
            if self.t0 is not None:
                t, total = now - self.t0, sum(d for _, _, d in self.segments)
                for i, (vr, vl, d) in enumerate(self.segments):
                    if t < d:
                        self.ros.send(vr, vl)
                        self.status.configure(text=f"● MOVING  segment {i + 1}/{len(self.segments)}"
                                                   f"   {now - self.t0:.1f} / {total:.1f} s", fg=GRN)
                        break
                    t -= d
                else:
                    self.ros.send(0.0, 0.0)
                    self.segments, self.arrived = [], True
                    self.btn.configure(state="normal")
        else:
            self.ros.send(0.0, 0.0)
            if not pose:
                self.status.configure(text="… waiting for /ground_truth", fg=ORA)
            elif self.arrived and self.goal:
                miss = math.dist(pose[:2], self.goal) * 100
                self.status.configure(text=f"✔ ARRIVED — {miss:.1f} cm from the goal", fg=PUR)
            elif self.goal:
                self.status.configure(text="■ READY — press GO", fg=ORA)
            else:
                self.status.configure(text="■ Click the map", fg=ORA)
        self.draw()
        self.root.after(20, self.tick)

    # ------------------------------------------------------------- drawing
    def draw(self):
        cv = self.cv
        cv.delete("all")
        w, h = cv.winfo_width(), cv.winfo_height()
        cx, cy, k, vx, vy = self.mapping()
        # grid, 1 m
        x0w, y1w = self.c2w(0, 0)
        x1w, y0w = self.c2w(w, h)
        for gx in range(math.floor(x0w), math.ceil(x1w) + 1):
            px, _ = self.w2c(gx, 0)
            cv.create_line(px, 0, px, h, fill=GRID)
        for gy in range(math.floor(y0w), math.ceil(y1w) + 1):
            _, py = self.w2c(0, gy)
            cv.create_line(0, py, w, py, fill=GRID)
        cv.create_text(8, h - 8, anchor="sw", text="grid 1 m · scroll = zoom", font=self.fS, fill=MUTED)
        # plan
        if self.preview:
            pts = [c for p in self.preview["pts"] for c in self.w2c(*p)]
            if len(pts) >= 4:
                cv.create_line(*pts, fill=PUR, width=3, dash=(8, 6))
            if self.preview["icc"]:
                ix, iy = self.w2c(*self.preview["icc"])
                cv.create_line(ix - 7, iy - 7, ix + 7, iy + 7, fill=ORA, width=3)
                cv.create_line(ix - 7, iy + 7, ix + 7, iy - 7, fill=ORA, width=3)
                cv.create_text(ix + 10, iy, anchor="w", text="ICC", font=self.fS, fill=ORA)
            ex, ey = self.preview["pts"][-1]
            a = self.preview["arrive"]
            p1, p2 = self.w2c(ex, ey), self.w2c(ex + 0.6 * math.cos(a), ey + 0.6 * math.sin(a))
            cv.create_line(*p1, *p2, fill=PUR, width=3, arrow="last", arrowshape=(12, 14, 5))
        # trail (true path)
        if len(self.trail) >= 2:
            pts = [c for p in self.trail for c in self.w2c(*p)]
            cv.create_line(*pts, fill=GRN, width=3)
        # goal (drawn under the robot)
        if self.goal:
            gx, gy = self.w2c(*self.goal)
            star = []
            for i in range(10):
                rr = 11 if i % 2 == 0 else 5
                ang = -math.pi / 2 + i * math.pi / 5
                star += [gx + rr * math.cos(ang), gy + rr * math.sin(ang)]
            cv.create_polygon(*star, fill=RED, outline="")
        # robot
        if self.ros.pose:
            x, y, th = self.ros.pose
            c, s = math.cos(th), math.sin(th)
            corners = [(0.2, 0.15), (0.2, -0.15), (-0.2, -0.15), (-0.2, 0.15)]
            poly = [v for dx, dy in corners for v in self.w2c(x + c * dx - s * dy, y + s * dx + c * dy)]
            cv.create_polygon(*poly, fill="#8F84E0", outline=PUR, width=2)
            p1, p2 = self.w2c(x, y), self.w2c(x + 0.35 * c, y + 0.35 * s)
            cv.create_line(*p1, *p2, fill=ORA, width=3, arrow="last")
        # legend
        lx, ly = 10, 10
        for col, txt, dash in ((GRN, "true path (Gazebo)", None), (PUR, "planned path", (8, 6))):
            cv.create_line(lx, ly + 7, lx + 26, ly + 7, fill=col, width=3, dash=dash)
            cv.create_text(lx + 32, ly + 7, anchor="w", text=txt, font=self.fS, fill=INK)
            ly += 18
        cv.create_text(lx + 13, ly + 7, text="★", font=self.fT, fill=RED)
        cv.create_text(lx + 32, ly + 7, anchor="w", text="goal   × ICC   → arrival heading",
                       font=self.fS, fill=INK)

    def close(self):
        for _ in range(5):
            self.ros.send(0.0, 0.0)
        self.root.destroy()


def main():
    rclpy.init()
    ros = RosIO()
    app = App(ros)
    try:
        app.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for _ in range(5):
                ros.send(0.0, 0.0)
        except BaseException:
            pass
        try:
            ros.destroy_node()
            rclpy.try_shutdown()
        except BaseException:
            pass
