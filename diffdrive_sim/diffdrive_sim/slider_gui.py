"""
slider_gui — slider window that drives the robot in Gazebo, with a live equation panel.

Move R and V -> inverse kinematics is recomputed -> VR, VL are sent to each wheel.
The right panel shows the substituted equations (the numbers change, the equations do not).
Text and sliders rescale with the window size, so it can be fitted into any corner of the screen.

  ros2 run diffdrive_sim slider_gui
Needs tkinter:  sudo apt install python3-tk -y
"""
import math
import tkinter as tk
import tkinter.font as tkfont

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from diffdrive_sim import diffdrive_kin as kin

L, r = 0.5, 0.1          # MUST match the robot model in the SDF
BG, INK, MUTED, GRN, ORA = "#F4F6F7", "#18222E", "#4A5868", "#137A5C", "#C2541F"
BASE_W, BASE_H = 940, 480   # design size; everything scales from here


class WheelPub(Node):
    def __init__(self):
        super().__init__("slider_gui")
        self.pl = self.create_publisher(Float64, "/left_wheel/cmd", 10)
        self.pr = self.create_publisher(Float64, "/right_wheel/cmd", 10)

    def send(self, vr, vl):
        self.pl.publish(Float64(data=vl / r))
        self.pr.publish(Float64(data=vr / r))


class App:
    def __init__(self, node):
        self.node = node
        self.vr = self.vl = 0.0
        self.aktif = False
        self.root = tk.Tk()
        self.root.title("Differential drive: live inverse kinematics")
        self.root.geometry(f"{BASE_W}x{BASE_H}")
        self.root.minsize(420, 240)
        self.root.configure(bg=BG)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        # fonts that are resized together when the window changes size
        self.fH = tkfont.Font(family="DejaVu Sans", size=15, weight="bold")
        self.fT = tkfont.Font(family="DejaVu Sans", size=12)
        self.fM = tkfont.Font(family="DejaVu Sans Mono", size=13)

        self.root.columnconfigure(0, weight=4, uniform="c")
        self.root.columnconfigure(1, weight=6, uniform="c")
        self.root.rowconfigure(0, weight=1)

        left = tk.Frame(self.root, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(14, 8), pady=12)
        right = tk.Frame(self.root, bg="white", highlightbackground="#D5DCE2", highlightthickness=1)
        right.grid(row=0, column=1, sticky="nsew", padx=(0, 14), pady=12)

        tk.Label(left, text="Radius R [m]", font=self.fH, bg=BG, fg=INK).pack(anchor="w")
        tk.Label(left, text="+ left   0 spin   − right", font=self.fT, bg=BG, fg=MUTED).pack(anchor="w")
        self.R = tk.DoubleVar(value=1.5)
        self.sR = tk.Scale(left, from_=-5, to=5, resolution=0.05, orient="horizontal", width=18,
                           variable=self.R, font=self.fT, bg=BG, highlightthickness=0,
                           troughcolor="#DCD7F7", command=lambda _: self.update())
        self.sR.pack(anchor="w", fill="x", pady=(2, 10))

        tk.Label(left, text="Forward speed V [m/s]", font=self.fH, bg=BG, fg=INK).pack(anchor="w")
        self.V = tk.DoubleVar(value=0.4)
        self.sV = tk.Scale(left, from_=0, to=1, resolution=0.05, orient="horizontal", width=18,
                           variable=self.V, font=self.fT, bg=BG, highlightthickness=0,
                           troughcolor="#DCD7F7", command=lambda _: self.update())
        self.sV.pack(anchor="w", fill="x", pady=(2, 10))

        self.straight = tk.BooleanVar(value=False)
        tk.Checkbutton(left, text="Straight (R = ∞)", variable=self.straight, font=self.fT, bg=BG,
                       command=self.update).pack(anchor="w", pady=(0, 10))

        self.btn = tk.Button(left, text="▶  START", font=self.fH, bg=GRN, fg="white",
                             activebackground=GRN, command=self.toggle)
        self.btn.pack(anchor="w", fill="x", ipady=6)

        self.eq = tk.Label(right, text="", font=self.fM, bg="white", fg=INK, justify="left", anchor="nw")
        self.eq.pack(anchor="nw", fill="both", expand=True, padx=14, pady=(10, 4))
        self.status = tk.Label(right, text="", font=self.fH, bg="white", fg=ORA, anchor="w")
        self.status.pack(anchor="w", fill="x", padx=14, pady=(0, 10))

        self._last = None
        self.root.bind("<Configure>", self.on_resize)
        self.update()
        self.root.after(20, self.tick)

    def on_resize(self, event):
        if event.widget is not self.root:
            return
        size = (event.width, event.height)
        if size == self._last:
            return
        self._last = size
        s = max(0.45, min(event.width / BASE_W, event.height / BASE_H))
        self.fH.configure(size=max(8, round(15 * s)))
        self.fT.configure(size=max(7, round(12 * s)))
        self.fM.configure(size=max(7, round(13 * s)))
        self.sR.configure(width=max(10, round(18 * s)))
        self.sV.configure(width=max(10, round(18 * s)))

    def toggle(self):
        self.aktif = not self.aktif
        self.btn.configure(text="■  STOP" if self.aktif else "▶  START",
                           bg=ORA if self.aktif else GRN, activebackground=ORA if self.aktif else GRN)
        self.update()

    def update(self):
        R, V = self.R.get(), self.V.get()
        lines = [f"L = {L} m,  r = {r} m"]
        if self.straight.get():
            vr, vl = kin.inverse(V, math.inf, L)
            lines += ["", "Straight: R = ∞  →  ω = 0",
                      f"VR = VL = V = {V:.2f} m/s", "", "ICC at infinity"]
        elif abs(R) < 0.025:
            w = V / (L / 2) if V > 0 else 0.0
            vr, vl = kin.inverse_spin(w, L)
            lines += ["", "Spin: R = 0  →  V_centre = 0",
                      f"ω  = V/(L/2) = {V:.2f}/{L/2:.2f} = {w:.3f} rad/s",
                      f"VR = +ω·L/2 = {vr:+.4f} m/s",
                      f"VL = −ω·L/2 = {vl:+.4f} m/s", "", "ICC at the robot centre"]
        else:
            vr, vl = kin.inverse(V, R, L)
            w = V / R
            lines += ["",
                      f"ω  = V / R = {V:.2f} / {R:.2f}",
                      f"   = {w:+.3f} rad/s", "",
                      f"VR = ω(R + L/2) = {vr:+.4f} m/s",
                      f"VL = ω(R − L/2) = {vl:+.4f} m/s", "",
                      f"ICC {abs(R):.2f} m to the {'LEFT' if R > 0 else 'RIGHT'}"]
        lines += [f"Wheels: L {vl / r:+.2f}, R {vr / r:+.2f} rad/s"]
        self.eq.configure(text="\n".join(lines))
        if self.aktif:
            self.vr, self.vl = vr, vl
            self.status.configure(text="● MOVING", fg=GRN)
        else:
            self.vr = self.vl = 0.0
            self.status.configure(text="■ STOPPED — press START", fg=ORA)

    def tick(self):
        self.node.send(self.vr, self.vl)                 # send 50 times per second
        rclpy.spin_once(self.node, timeout_sec=0)
        self.root.after(20, self.tick)

    def close(self):
        for _ in range(5):
            self.node.send(0.0, 0.0)                     # brake before closing
        self.root.destroy()


def main():
    rclpy.init()
    node = WheelPub()
    app = App(node)
    try:
        app.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for _ in range(5):
                node.send(0.0, 0.0)
        except BaseException:
            pass
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except BaseException:
            pass
