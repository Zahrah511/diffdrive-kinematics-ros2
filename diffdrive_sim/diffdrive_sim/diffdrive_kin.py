"""
diffdrive_kin.py — the core mathematics of a differential drive robot.

Pure Python (only the math module), no ROS. Every variation uses the functions here.
The ROS 2 nodes call these same functions, so each equation is written ONCE.

Conventions:
  VR, VL : linear speed of the right / left wheel  [m/s]
  L      : wheel separation                        [m]
  V      : forward speed of the robot centre       [m/s]
  w      : turn rate (omega)                       [rad/s], positive = turning left
  R      : distance from the ICC to the centre     [m], positive = ICC on the left
  pose   : (x, y, theta)  [m, m, rad], theta measured from the world x axis
"""
import math

EPS = 1e-9  # "treated as zero" threshold, avoids division by zero


# ---------------------------------------------------------------- Variation 1
def forward(vr, vl, L):
    """Forward kinematics: wheel speeds -> (V, w)."""
    V = (vr + vl) / 2.0
    w = (vr - vl) / L          # divided by L, NOT by 2 (transcription error in the lecture notes)
    return V, w


def icc_radius(vr, vl, L):
    """Radius R = V/w. Straight -> inf, spin in place -> 0."""
    V, w = forward(vr, vl, L)
    if abs(w) < EPS:
        return math.inf        # VR = VL: ICC at infinity
    return V / w               # VR = -VL gives V = 0, so R = 0


def icc_point(pose, vr, vl, L):
    """World coordinates of the ICC: (x - R sin th, y + R cos th). None when straight."""
    x, y, th = pose
    R = icc_radius(vr, vl, L)
    if math.isinf(R):
        return None
    return (x - R * math.sin(th), y + R * math.cos(th))


# ---------------------------------------------------------------- Variation 4
def step_exact(pose, vr, vl, L, dt):
    """Exact pose update: rotate the robot about the ICC by w*dt.

    The rotation-matrix form from the 17 Sep lecture, written without the matrix:
      x' = x + R [sin(th + w dt) - sin th]
      y' = y - R [cos(th + w dt) - cos th]
    The straight case (w = 0) is handled separately because R is infinite.
    """
    x, y, th = pose
    V, w = forward(vr, vl, L)
    if abs(w) < EPS:                       # straight
        return (x + V * math.cos(th) * dt,
                y + V * math.sin(th) * dt,
                th)
    R = V / w                              # R = 0 is also safe: x, y stay the same
    th2 = th + w * dt
    return (x + R * (math.sin(th2) - math.sin(th)),
            y - R * (math.cos(th2) - math.cos(th)),
            th2)


def step_euler(pose, vr, vl, L, dt):
    """Euler pose update: the heading is held fixed during dt."""
    x, y, th = pose
    V, w = forward(vr, vl, L)
    return (x + V * math.cos(th) * dt,
            y + V * math.sin(th) * dt,
            th + w * dt)


def simulate(pose0, vr, vl, L, dt, T, method="exact"):
    """Run constant wheel speeds for T seconds. Returns the list of poses."""
    step = step_exact if method == "exact" else step_euler
    n = int(round(T / dt))
    poses = [pose0]
    for _ in range(n):
        poses.append(step(poses[-1], vr, vl, L, dt))
    return poses


# ---------------------------------------------------------------- Variation 2
def inverse(V, R, L):
    """Inverse kinematics: wanted (V, R) -> (VR, VL).

    R = inf means straight. For spinning in place (R = 0) use inverse_spin.
    """
    if math.isinf(R):
        return V, V
    w = V / R
    return w * (R + L / 2.0), w * (R - L / 2.0)


def inverse_spin(w, L):
    """Spin in place at rate w: VR = w L/2, VL = -w L/2."""
    return w * L / 2.0, -w * L / 2.0


# ---------------------------------------------------------------- Variation 3
def plan_square(side, V, w_spin, L, n_sides=4):
    """Square: [straight, spin 90 degrees] x n. Returns segments (VR, VL, duration)."""
    seg = []
    vr_s, vl_s = inverse_spin(w_spin, L)
    for _ in range(n_sides):
        seg.append((V, V, side / V))
        seg.append((vr_s, vl_s, (math.pi / 2) / abs(w_spin)))
    return seg


def plan_figure8(R, V, L):
    """Figure 8: one circle turning left (R > 0), then one turning right (R < 0)."""
    T = 2 * math.pi * abs(R) / V
    vr1, vl1 = inverse(V, abs(R), L)
    vr2, vl2 = inverse(V, -abs(R), L)
    return [(vr1, vl1, T), (vr2, vl2, T)]


def run_segments(pose0, segments, L, dt, method="exact"):
    """Run a sequence of segments (VR, VL, duration). Returns all poses."""
    poses = [pose0]
    for vr, vl, dur in segments:
        poses += simulate(poses[-1], vr, vl, L, dt, dur, method)[1:]
    return poses


# ---------------------------------------------------------------- Variation 5
def world_to_robot(goal, pose):
    """World point (xg, yg) -> robot frame (x forward, y to the left)."""
    xg, yg = goal
    x, y, th = pose
    dx, dy = xg - x, yg - y
    return (math.cos(th) * dx + math.sin(th) * dy,
            -math.sin(th) * dx + math.cos(th) * dy)


def plan_arc_to_goal(xr, yr, V, L):
    """Strategy A — a single arc to the point (xr, yr) in the robot frame.

    R = (x^2 + y^2) / (2y),  dtheta = 2 atan2(y, x),  t = R dtheta / V
    Returns a dict with R, dtheta, t, VR, VL and the arc length s.
    """
    if abs(yr) < EPS:                      # point straight ahead: just drive straight
        if xr <= 0:
            raise ValueError("Point is behind the robot or at its position: one forward arc cannot reach it.")
        return dict(R=math.inf, dtheta=0.0, t=xr / V, VR=V, VL=V, s=xr)
    R = (xr ** 2 + yr ** 2) / (2 * yr)
    dtheta = 2 * math.atan2(yr, xr)        # same sign as y
    s = R * dtheta                          # arc length (always positive)
    vr, vl = inverse(V, R, L)
    return dict(R=R, dtheta=dtheta, t=s / V, VR=vr, VL=vl, s=s)


def plan_spin_then_straight(xr, yr, V, w_spin, L):
    """Strategy B — spin in place until facing the point, then drive straight."""
    alpha = math.atan2(yr, xr)
    d = math.hypot(xr, yr)
    w = math.copysign(abs(w_spin), alpha) if abs(alpha) > EPS else abs(w_spin)
    vr_s, vl_s = inverse_spin(w, L)
    return [(vr_s, vl_s, abs(alpha) / abs(w_spin)), (V, V, d / V)]
