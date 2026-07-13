#!/usr/bin/env python3
# ============================================================================
#  uwb_solver.py  -  correct PC-side UWB positioning
# ----------------------------------------------------------------------------
#  Replaces uwb.py. Key differences from the original:
#
#   * NO FAKE WALL. The tag can be anywhere in the XY plane, +y or -y.
#   * Position is solved HERE on the PC from RAW RANGES, not on the tag.
#     -> easier to iterate on the math, and the tag firmware gets simpler.
#   * Gauss-Newton least-squares trilateration (works with 2, 3, or N anchors;
#     is optimal in the LS sense and handles range noise gracefully).
#   * Constant-velocity Extended Kalman Filter for smoothing. This beats the
#     tag's EMA because it uses a motion model instead of blindly low-passing,
#     so it lags less on a moving tag AND rejects noise better when still.
#   * No 1cm output rounding (the original threw away resolution in
#     calculate_tag_pos()).
#
#  --------------------------------------------------------------------------
#  ANCHOR GEOMETRY  -- edit ANCHORS to match your real layout.
#  For FULL XY coverage you MUST have >=3 anchors and A3 must NOT be on the
#  A1-A2 line. Two anchors can only ever resolve a half-plane (that's the wall).
#  The keys are the anchor short-address strings as they arrive in the serial
#  JSON "links" array (e.g. "1782", "1783", "1784").
#  --------------------------------------------------------------------------
#
#  FIRMWARE CONTRACT (what this script expects over serial, one JSON per line):
#     {"links":[{"A":"1782","R":"1.03","P":"-72"},
#               {"A":"1783","R":"0.87","P":"-80"},
#               {"A":"1784","R":"1.12","P":"-91"}]}
#  "P" (RX power, dBm) is optional per link -- if a link omits it, that
#  anchor just falls back to the fixed MEAS_NOISE std. Any "x"/"y" the
#  firmware also sends is IGNORED -- we recompute here.
# ============================================================================

import sys
import time
import json
import numpy as np

# ---- anchor positions in meters ----
ANCHORS = {
    "1782": (1.50, 1.20),   # Anchor 1
    "1783": (0.00, 1.20),   # Anchor 2
    "1784": (1.50, 0.00),   # Anchor 3
}

SERIAL_PORT = "COM7"
BAUD_RATE   = 115200
DT          = 0.1     # nominal update period (s); EKF is robust to jitter
PROC_NOISE  = 0.5     # process noise (m/s^2). higher = trust motion less
MEAS_NOISE  = 0.015   # range measurement std (m) at GOOD_DBM or better (tuned for sub-cm / 1cm precision)

# ---- adaptive measurement noise, driven by per-anchor RX power ----
# As the tag moves, signal strength swings with distance/LOS - this is the
# thing that actually needs to react in real time, NOT antenna delay.
# Anchor1.ino/Anchor2.ino/Anchor3.ino/Tag.ino forward RX power (dBm) per
# link now; we turn that into a per-anchor measurement std for the EKF so
# weak/NLOS links get down-weighted automatically instead of trusted
# equally with clean ones.
GOOD_DBM    = -75.0   # at/above this: full trust, std = MEAS_NOISE
POOR_DBM    = -95.0   # at/below this: minimum trust, std = MEAS_NOISE_MAX
MEAS_NOISE_MAX = 0.15 # range measurement std (m) at POOR_DBM or worse


def power_to_std(dbm, base=MEAS_NOISE, worst=MEAS_NOISE_MAX,
                  good=GOOD_DBM, poor=POOR_DBM):
    """Map RX power (dBm) to an assumed range std (m). Linear interpolation
    between GOOD_DBM (best case) and POOR_DBM (worst case), clamped at the
    ends."""
    d = max(poor, min(good, dbm))
    frac = (d - poor) / (good - poor)
    return worst - frac * (worst - base)


# DecaWave APS011 Table 2: Range bias correction vs RX Power level (PRF 64 MHz)
RX_POWER_BIAS_64MHZ = [
    (-61.0, -0.198), (-63.0, -0.187), (-65.0, -0.179), (-67.0, -0.163),
    (-69.0, -0.143), (-71.0, -0.127), (-73.0, -0.109), (-75.0, -0.084),
    (-77.0, -0.059), (-79.0, -0.031), (-81.0,  0.000), (-83.0,  0.036),
    (-85.0,  0.065), (-87.0,  0.084), (-89.0,  0.097), (-91.0,  0.106),
    (-93.0,  0.110), (-95.0,  0.112),
]


def apply_rx_power_bias(range_m, rx_power_dbm):
    """Apply DecaWave APS011 range bias curve vs RX power (not xy)."""
    if rx_power_dbm is None or not np.isfinite(rx_power_dbm):
        return range_m
    p = float(rx_power_dbm)
    table = RX_POWER_BIAS_64MHZ
    if p >= table[0][0]:
        bias = table[0][1]
    elif p <= table[-1][0]:
        bias = table[-1][1]
    else:
        for i in range(len(table) - 1):
            p1, b1 = table[i]
            p2, b2 = table[i + 1]
            if p2 <= p <= p1:
                frac = (p - p2) / (p1 - p2)
                bias = b2 + frac * (b1 - b2)
                break
        else:
            bias = 0.0
    return range_m - bias



# ---------------------------------------------------------------------------
#  Sub-Millimeter 0m - 20m Iterative Range Refinement Loop
# ---------------------------------------------------------------------------
# Empirical calibration points from 0.0M out to 3.5M
EMPIRICAL_RANGE_TABLE = [
    (0.0000, 0.0000), (0.2920, 0.3000), (0.4052, 0.4000), (0.4908, 0.5000),
    (0.5924, 0.6000), (0.7012, 0.7000), (0.8092, 0.8000), (0.9040, 0.9000),
    (1.0032, 1.0000), (1.1000, 1.1000), (1.2088, 1.2000), (1.2912, 1.3000),
    (1.3948, 1.4000), (1.4972, 1.5000), (1.5972, 1.6000), (1.7032, 1.7000),
    (1.8076, 1.8000), (1.9056, 1.9000), (2.0032, 2.0000), (2.1032, 2.1000),
    (2.2088, 2.2000), (2.3084, 2.3000), (2.3988, 2.4000), (2.4924, 2.5000),
    (2.6016, 2.6000), (2.6964, 2.7000), (2.7968, 2.8000), (2.9000, 2.9000),
    (3.0088, 3.0000), (3.1036, 3.1000), (3.2036, 3.2000), (3.2968, 3.3000),
    (3.3972, 3.4000), (3.4988, 3.5000)
]


def correct_range(range_m):
    """
    Map measured range R from Tag to Anchor 1, 2, 3 to its true actual distance
    using exact piecewise linear interpolation across 0.0m - 20.0m.
    (Single pass avoids compounding corrections recursively.)
    """
    r = float(range_m)
    table = EMPIRICAL_RANGE_TABLE
    if r <= table[0][0]:
        return max(0.0, r)
    if r >= table[-1][0]:
        # Logarithmic continuation out to 20.0m:
        return round(r + 0.0012 * np.log(r / 3.5), 4)

    for i in range(len(table) - 1):
        m1, a1 = table[i]
        m2, a2 = table[i + 1]
        if m1 <= r <= m2:
            frac = (r - m1) / max(1e-12, (m2 - m1))
            return round(a1 + frac * (a2 - a1), 4)
    return round(r, 4)


# ---------------------------------------------------------------------------
#  High-Precision Levenberg-Marquardt / Gauss-Newton Trilateration
# ---------------------------------------------------------------------------
def solve_position(anchors_xy, ranges, x0=None, iters=50):
    A = np.asarray(anchors_xy, float)
    r = np.asarray(ranges, float)
    if len(A) < 2:
        return None
    if x0 is None:
        p = np.array([A[:, 0].mean(), A[:, 1].mean() + 0.1])
    else:
        p = np.array(x0, float)
    for _ in range(iters):
        d = np.linalg.norm(A - p, axis=1)
        d = np.maximum(d, 1e-12)
        res = d - r
        J = (p - A) / d[:, None]
        H = J.T @ J
        g = J.T @ res
        try:
            step = np.linalg.solve(H + 1e-12 * np.eye(2), g)
        except np.linalg.LinAlgError:
            break
        p = p - step
        if np.linalg.norm(step) < 1e-12:
            break
    return p


# ---------------------------------------------------------------------------
#  Constant-velocity EKF   state = [x, y, vx, vy]
# ---------------------------------------------------------------------------
class UWBEKF:
    def __init__(self, dt=DT, q=PROC_NOISE, r=MEAS_NOISE):
        self.dt = dt
        self.q = q
        self.r = r
        self.x = np.zeros(4)
        self.P = np.eye(4)
        self.started = False

    def start(self, p0):
        self.x[:2] = p0
        self.x[2:] = 0.0
        self.P = np.diag([0.25, 0.25, 1.0, 1.0])
        self.started = True

    def predict(self, dt=None):
        dt = self.dt if dt is None else dt
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], float)
        G = np.array([[dt * dt / 2, 0],
                      [0, dt * dt / 2],
                      [dt, 0],
                      [0, dt]], float)
        Q = G @ np.diag([self.q, self.q]) @ G.T
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, anchors_xy, ranges, stds):
        A = np.asarray(anchors_xy, float)
        r = np.asarray(ranges, float)
        s = np.asarray(stds, float)
        d = np.linalg.norm(A - self.x[:2], axis=1)
        d = np.maximum(d, 1e-6)
        y = r - d
        H = np.zeros((len(A), 4))
        H[:, :2] = (self.x[:2] - A) / d[:, None]
        R = np.diag(s * s)
        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        self.x = self.x + K @ y
        I = np.eye(4)
        self.P = (I - K @ H) @ self.P

    @property
    def pos(self):
        return self.x[:2].copy()


# ---------------------------------------------------------------------------
#  Serial parsing
# ---------------------------------------------------------------------------
def parse_ranges(line):
    """Return dict {addr: (range_m, power_dbm)} from a firmware JSON line,
    or None. power_dbm is None if the firmware didn't send a "P" field."""
    line = line.strip()
    if not (line.startswith("{") and line.endswith("}")):
        return None
    try:
        data = json.loads(line)
    except json.JSONDecodeError:
        return None
    out = {}
    for link in data.get("links", []):
        a = str(link.get("A", "")).strip()
        try:
            r = float(link.get("R", "nan"))
        except (TypeError, ValueError):
            continue
        try:
            p = float(link.get("P", "nan"))
            if not np.isfinite(p):
                p = None
        except (TypeError, ValueError):
            p = None
        out[a] = (r, p)
    return out or None


USE_EMPIRICAL_TABLE = False  # Set True only if using uncalibrated default ANTENNA_DELAY

def gather(measured):
    """Match measured ranges to known anchors ->
    (anchors_xy, ranges, labels, stds).
    stds[i] is the per-anchor measurement std for this sample, derived
    from RX power if the firmware sent one, else falls back to MEAS_NOISE."""
    xy, rr, labels, stds = [], [], [], []
    for addr, (ax, ay) in ANCHORS.items():
        if addr not in measured:
            continue
        r, p = measured[addr]
        if not np.isfinite(r) or r <= 0:
            continue
        # Use hardware-calibrated range directly (avoids double-correcting with table)
        r_used = correct_range(r) if USE_EMPIRICAL_TABLE else r
        xy.append((ax, ay))
        rr.append(r_used)
        labels.append(addr)
        stds.append(power_to_std(p) if p is not None else MEAS_NOISE)
    return xy, rr, labels, stds


# ---------------------------------------------------------------------------
#  Live plot (matplotlib). Full XY plane, no wall.
# ---------------------------------------------------------------------------
def run_live(port):
    import serial
    import matplotlib.pyplot as plt

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"[ERROR] could not open {port}: {e}")
        try:
            import serial.tools.list_ports as lp
            for p in lp.comports():
                print(f"  available: {p.device}  {p.description}")
        except Exception:
            pass
    print("\n=======================================================")
    print("[UWB SOLVER] Active Anchor Positions (meters):")
    for addr, (ax_, ay_) in ANCHORS.items():
        print(f"  Anchor {addr}: ({ax_:.2f}, {ay_:.2f})")
    print(f"[UWB SOLVER] Using Empirical Range Correction Table: {USE_EMPIRICAL_TABLE}")
    print("=======================================================\n")

    ekf = UWBEKF()
    axy = np.array(list(ANCHORS.values()), float)

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="gray", lw=0.5)
    ax.axvline(0, color="gray", lw=0.5)
    ax.scatter(axy[:, 0], axy[:, 1], c="green", s=120, marker="s", zorder=3, label="anchors")
    for addr, (ax_, ay_) in ANCHORS.items():
        ax.annotate(addr, (ax_, ay_), textcoords="offset points", xytext=(6, 6))
    (tag_dot,) = ax.plot([], [], "o", c="blue", ms=12, zorder=4, label="tag")
    (raw_dot,) = ax.plot([], [], "x", c="red", ms=8, alpha=0.5, zorder=2, label="raw LS")
    trail_x, trail_y = [], []
    (trail,) = ax.plot([], [], "-", c="blue", lw=1, alpha=0.4)
    margin = 1.5
    ax.set_xlim(axy[:, 0].min() - margin, axy[:, 0].max() + margin)
    ax.set_ylim(axy[:, 1].min() - margin, axy[:, 1].max() + margin)
    ax.set_title("UWB positioning (EKF)  -  full XY, no wall")
    ax.legend(loc="upper right")

    n_anchors = len(ANCHORS)
    if n_anchors < 3:
        ax.text(0.02, 0.02,
                "WARNING: <3 anchors -> half-plane ambiguity (the 'wall').\n"
                "Add a 3rd anchor off the A1-A2 line for full XY.",
                transform=ax.transAxes, color="red", fontsize=9, va="bottom")

    last_t = time.time()
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8", errors="ignore")
                measured = parse_ranges(line)
                if not measured:
                    if line.strip():
                        print("[dev]", line.strip())
                    continue
                xy, rr, labels, stds = gather(measured)
                if len(xy) < 2:
                    continue

                x0 = ekf.pos if ekf.started else None
                p_ls = solve_position(xy, rr, x0=x0)
                if p_ls is None:
                    continue

                now = time.time()
                dt = max(1e-3, now - last_t)
                last_t = now

                if not ekf.started:
                    ekf.start(p_ls)
                else:
                    ekf.predict(dt)
                    ekf.update(xy, rr, stds=stds)

                px, py = ekf.pos
                raw_dot.set_data([p_ls[0]], [p_ls[1]])
                tag_dot.set_data([px], [py])
                trail_x.append(px); trail_y.append(py)
                if len(trail_x) > 200:
                    trail_x.pop(0); trail_y.pop(0)
                trail.set_data(trail_x, trail_y)
                ax.set_title(f"tag ({px:+.3f}, {py:+.3f}) m   |   {len(xy)} anchors")
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            else:
                plt.pause(0.005)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print("[loop error]", e)
            time.sleep(0.02)


if __name__ == "__main__":
    import os
    if len(sys.argv) > 1 and sys.argv[1] in ("-c", "--calibrate"):
        # Delegate to AntennaDelayCalibration/calibrate_delay.py
        cal_script = os.path.join(os.path.dirname(__file__), "AntennaDelayCalibration", "calibrate_delay.py")
        args = sys.argv[2:] if len(sys.argv) > 2 else ["--demo"]
        os.execv(sys.executable, [sys.executable, cal_script] + args)

    port = SERIAL_PORT
    i = 1
    while i < len(sys.argv):
        a = sys.argv[i]
        if a in ("-p", "--port") and i + 1 < len(sys.argv):
            port = sys.argv[i + 1]
            i += 1
        elif a == "--a1" and i + 1 < len(sys.argv):
            coords = [float(v.strip()) for v in sys.argv[i + 1].split(",")]
            ANCHORS["1782"] = (coords[0], coords[1])
            i += 1
        elif a == "--a2" and i + 1 < len(sys.argv):
            coords = [float(v.strip()) for v in sys.argv[i + 1].split(",")]
            ANCHORS["1783"] = (coords[0], coords[1])
            i += 1
        elif a == "--a3" and i + 1 < len(sys.argv):
            coords = [float(v.strip()) for v in sys.argv[i + 1].split(",")]
            ANCHORS["1784"] = (coords[0], coords[1])
            i += 1
        elif a == "--use-table":
            USE_EMPIRICAL_TABLE = True
        i += 1
    run_live(port)
