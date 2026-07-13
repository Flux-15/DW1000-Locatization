#!/usr/bin/env python3
# ============================================================================
#  calibrate_delay.py  -  Least-Squares Antenna Delay (ADELAY) Calibration
# ============================================================================
#  Implements the full calibration & compensation architecture:
#
#  [Survey points: known xy, 3 anchors]
#           │
#           ▼
#  [Apply bias curve: vs RX power, not xy]  <-- DecaWave APS011 Table 2
#           │
#           ▼
#  [Least-squares: solve 4 delay values]    <-- Solves Tag + A1 + A2 + A3 ADELAY
#           │
#           ▼
#  [Burn constants: fixed at runtime]       <-- #define ANTENNA_DELAY in .ino
#
#  Usage:
#     python calibrate_delay.py --demo
#     python calibrate_delay.py survey_data.json
#     python calibrate_delay.py survey_data.json --apply
#     python calibrate_delay.py --live COM7 --x 0.5 --y 0.5
# ============================================================================

import os
import sys
import json
import argparse
import numpy as np

# ---- Default anchor positions (meters) matching uwb_solver.py ----
DEFAULT_ANCHORS = {
    "1782": (0.00, 0.00),   # Anchor 1
    "1783": (0.90, 0.00),   # Anchor 2
    "1784": (0.00, 0.90),   # Anchor 3
}

# DW1000 Physical Constants (DecaWave APS014)
SPEED_OF_LIGHT = 299792458.0  # m/s
DW1000_TIME_UNIT = 1.0 / (499.2e6 * 128.0)  # ~15.6500400641 ps
# Distance change per antenna delay tick in Two-Way Ranging (round trip / 2 * c):
DIST_PER_TICK = SPEED_OF_LIGHT * DW1000_TIME_UNIT  # ~0.00469176197 m/tick

DEFAULT_OLD_ADELAY = 16512

# DecaWave APS011 Table 2: Range bias correction vs RX Power level (PRF 64 MHz)
# Format: (RX_Power_dBm, Range_Bias_meters)
# R_corrected = R_raw - apply_rx_power_bias(R_raw, RX_Power_dBm)
RX_POWER_BIAS_64MHZ = [
    (-61.0, -0.198),
    (-63.0, -0.187),
    (-65.0, -0.179),
    (-67.0, -0.163),
    (-69.0, -0.143),
    (-71.0, -0.127),
    (-73.0, -0.109),
    (-75.0, -0.084),
    (-77.0, -0.059),
    (-79.0, -0.031),
    (-81.0,  0.000),  # Reference level (-81 dBm)
    (-83.0,  0.036),
    (-85.0,  0.065),
    (-87.0,  0.084),
    (-89.0,  0.097),
    (-91.0,  0.106),
    (-93.0,  0.110),
    (-95.0,  0.112),
]


def apply_rx_power_bias(range_m, rx_power_dbm):
    """
    Apply DecaWave APS011 range bias curve vs RX power (not xy).
    Returns the corrected range in meters.
    """
    if rx_power_dbm is None or not np.isfinite(rx_power_dbm):
        return range_m

    p = float(rx_power_dbm)
    table = RX_POWER_BIAS_64MHZ
    if p >= table[0][0]:
        bias = table[0][1]
    elif p <= table[-1][0]:
        bias = table[-1][1]
    else:
        # Linear interpolation between table points
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


def solve_4_delay_values(survey_points, anchors_xy=DEFAULT_ANCHORS, old_adelays=None):
    """
    Least-squares solver for 4 delay values:
      [Tag_ADELAY, Anchor1_ADELAY, Anchor2_ADELAY, Anchor3_ADELAY]

    Parameters
    ----------
    survey_points : list of dicts
        Each item: {
            "tag_xy": [x, y],
            "measurements": {
                "1782": {"R": range_m, "P": rx_power_dbm},
                ...
            }
        }
    anchors_xy : dict
        {addr_str: (ax, ay)}
    old_adelays : dict
        Current ADELAY values, e.g. {"Tag": 16512, "1782": 16512, ...}

    Returns
    -------
    results : dict containing solved range offsets (m), solved ADELAY ticks,
              and residual statistics.
    """
    anchor_keys = sorted(list(anchors_xy.keys()))
    if len(anchor_keys) != 3:
        raise ValueError("Least-squares 4-delay solve requires exactly 3 anchors.")

    if old_adelays is None:
        old_adelays = {k: DEFAULT_OLD_ADELAY for k in ["Tag"] + anchor_keys}

    # Build linear system M * c = y
    # Parameters c = [c_Tag, c_A1, c_A2, c_A3] (range offsets in meters)
    # Model: R_meas_corrected - d_true = c_Tag + c_Ai
    rows_M = []
    rows_y = []
    details = []

    for pt_idx, pt in enumerate(survey_points):
        tx, ty = pt["tag_xy"]
        meas = pt.get("measurements", {})
        for a_idx, addr in enumerate(anchor_keys):
            if addr not in meas:
                continue
            r_raw = float(meas[addr].get("R", np.nan))
            p_dbm = meas[addr].get("P", None)
            if not np.isfinite(r_raw):
                continue

            # Note: DW1000Ranging.getRange() already applies DecaWave APS011 RX power bias internally.
            r_used = r_raw

            # 2. Compute true geometric distance
            ax, ay = anchors_xy[addr]
            d_true = np.hypot(tx - ax, ty - ay)

            # 3. Residual range discrepancy
            err_m = r_used - d_true

            row = [1.0, 0.0, 0.0, 0.0]
            row[1 + a_idx] = 1.0
            rows_M.append(row)
            rows_y.append(err_m)

            details.append({
                "point_idx": pt_idx,
                "tag_xy": (tx, ty),
                "anchor": addr,
                "r_raw": r_raw,
                "p_dbm": p_dbm,
                "r_corr": r_corr,
                "d_true": d_true,
                "error_before": err_m
            })

    if not rows_M:
        raise ValueError("No valid survey measurements found.")

    M = np.array(rows_M, dtype=float)
    y = np.array(rows_y, dtype=float)

    # Gauge Regularization:
    # Since c_Tag + c_Ai is invariant under (c_Tag + alpha, c_Ai - alpha),
    # we add a symmetric zero-mean regularization constraint:
    # c_A1 + c_A2 + c_A3 - 3*c_Tag = 0 (distributes calibration symmetrically)
    reg_row = np.array([[-3.0, 1.0, 1.0, 1.0]])
    reg_y = np.array([0.0])

    M_aug = np.vstack([M, reg_row])
    y_aug = np.hstack([y, reg_y])

    # Least squares solve
    c_sol, residuals, rank, s = np.linalg.lstsq(M_aug, y_aug, rcond=None)

    c_tag = c_sol[0]
    c_anchors = {addr: c_sol[1 + i] for i, addr in enumerate(anchor_keys)}

    # Convert range offset (m) to DW1000 ADELAY ticks
    # Increasing ADELAY by +1 tick decreases measured range by DIST_PER_TICK
    # So if measured range is positive bias (+c), we INCREASE ADELAY by c / DIST_PER_TICK
    solved_adelays = {
        "Tag": int(round(old_adelays.get("Tag", DEFAULT_OLD_ADELAY) + c_tag / DIST_PER_TICK))
    }
    for addr in anchor_keys:
        solved_adelays[addr] = int(round(old_adelays.get(addr, DEFAULT_OLD_ADELAY) + c_anchors[addr] / DIST_PER_TICK))

    # Evaluate residuals after calibration
    errors_after = []
    for d in details:
        addr = d["anchor"]
        predicted_bias = c_tag + c_anchors[addr]
        err_after = d["error_before"] - predicted_bias
        d["error_after"] = err_after
        errors_after.append(err_after)

    rmse_before = float(np.sqrt(np.mean(y ** 2)))
    rmse_after = float(np.sqrt(np.mean(np.array(errors_after) ** 2)))

    return {
        "offsets_m": {
            "Tag": float(c_tag),
            **{k: float(v) for k, v in c_anchors.items()}
        },
        "solved_adelays": solved_adelays,
        "old_adelays": old_adelays,
        "rmse_before_m": rmse_before,
        "rmse_after_m": rmse_after,
        "details": details,
        "anchors_xy": anchors_xy
    }


def generate_demo_survey():
    """Generate realistic synthetic survey data at 4 known survey positions."""
    anchors_xy = DEFAULT_ANCHORS
    # True delays: Tag=+18 ticks (+8.4 mm), A1=+35 ticks (+16.4 mm), A2=-22 ticks (-10.3 mm), A3=+10 ticks (+4.7 mm)
    true_offsets_m = {
        "Tag": 18 * DIST_PER_TICK,
        "1782": 35 * DIST_PER_TICK,
        "1783": -22 * DIST_PER_TICK,
        "1784": 10 * DIST_PER_TICK,
    }

    survey_positions = [
        (0.30, 0.30),
        (0.60, 0.30),
        (0.30, 0.60),
        (0.60, 0.60),
    ]

    survey_points = []
    for tx, ty in survey_positions:
        meas = {}
        for addr, (ax, ay) in anchors_xy.items():
            d_true = np.hypot(tx - ax, ty - ay)
            # Simulated RX power based on distance
            p_dbm = -72.0 - 10.0 * np.log10(max(d_true, 0.1))
            # Inverse of apply_rx_power_bias: add raw bias that APS011 introduces
            # At -72 dBm bias is ~ -0.118m
            rx_bias = -0.118
            # Add antenna delay offsets
            r_raw = d_true + true_offsets_m["Tag"] + true_offsets_m[addr] + rx_bias
            meas[addr] = {"R": round(r_raw, 4), "P": round(p_dbm, 1)}
        survey_points.append({"tag_xy": [tx, ty], "measurements": meas})
    return survey_points


def print_calibration_report(res):
    print("=" * 76)
    print("             UWB LEAST-SQUARES 4-DELAY CALIBRATION REPORT             ")
    print("=" * 76)
    print(f"Total Survey Measurements : {len(res['details'])}")
    print(f"RMSE Before Calibration   : {res['rmse_before_m']*1000:6.2f} mm")
    print(f"RMSE After  Calibration   : {res['rmse_after_m']*1000:6.2f} mm")
    print("-" * 76)
    print(f"{'Device':<12} {'Old ADELAY':<14} {'Solved ADELAY':<16} {'Offset (mm)':<14}")
    print("-" * 76)

    for dev in ["Tag"] + sorted([k for k in res["solved_adelays"] if k != "Tag"]):
        old_val = res["old_adelays"][dev]
        new_val = res["solved_adelays"][dev]
        off_mm = res["offsets_m"][dev] * 1000.0
        label = f"Tag (Tag.ino)" if dev == "Tag" else f"Anchor {dev}"
        print(f"{label:<14} {old_val:<14} {new_val:<16} {off_mm:+7.2f} mm")

    print("=" * 76)
    print("                   BURN CONSTANTS (FIXED AT RUNTIME)                  ")
    print("=" * 76)
    print("Copy & paste these #define values into each device's .ino file:")
    print()
    print(f"// In Tag/Tag.ino :")
    print(f"#define ANTENNA_DELAY {res['solved_adelays']['Tag']}")
    print()
    anchor_files = {"1782": "Anchor1/Anchor1.ino", "1783": "Anchor2/Anchor2.ino", "1784": "Anchor3/Anchor3.ino"}
    for addr in sorted(anchor_files.keys()):
        print(f"// In {anchor_files[addr]} (Anchor {addr}) :")
        print(f"#define ANTENNA_DELAY {res['solved_adelays'][addr]}")
    print("=" * 76)


def patch_ino_file(filepath, new_adelay):
    """Update #define ANTENNA_DELAY <value> in an Arduino .ino sketch."""
    if not os.path.exists(filepath):
        print(f"[WARNING] File not found: {filepath}")
        return False

    with open(filepath, "r", encoding="utf-8") as f:
        lines = f.readlines()

    updated = False
    new_lines = []
    for line in lines:
        if line.strip().startswith("#define ANTENNA_DELAY"):
            # Preserve indentation
            prefix = line.split("#define")[0]
            new_lines.append(f"{prefix}#define ANTENNA_DELAY {new_adelay}\n")
            updated = True
        else:
            new_lines.append(line)

    if updated:
        with open(filepath, "w", encoding="utf-8") as f:
            f.writelines(new_lines)
        print(f"[OK] Patched {filepath} -> #define ANTENNA_DELAY {new_adelay}")
        return True
    else:
        print(f"[WARNING] Could not find `#define ANTENNA_DELAY` line in {filepath}")
        return False


def load_survey_file(filepath):
    """Load survey data from JSON or CSV, supporting both grouped points and flat tabular records."""
    import csv
    if filepath.endswith(".csv"):
        with open(filepath, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            flat_rows = list(reader)
    else:
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, list) and len(data) > 0 and "measurements" in data[0]:
            return data
        flat_rows = data

    # Group flat tabular records by tag_xy coordinate
    grouped = {}
    for row in flat_rows:
        tx, ty = [float(v) for v in row.get("tag_xy", row.get("distance_cm", [0, 0]))]
        if abs(tx) > 10 or abs(ty) > 10:  # automatically convert cm -> m
            tx, ty = tx / 100.0, ty / 100.0
        key = (round(tx, 4), round(ty, 4))
        if key not in grouped:
            grouped[key] = {"tag_xy": [tx, ty], "measurements": {}}
        addr = str(row.get("anchor", "1782")).strip()
        r_val = float(row.get("R", row.get("median_R", 0)))
        p_val = float(row.get("P", row.get("median_P_dbm", -85.0)))
        grouped[key]["measurements"][addr] = {"R": r_val, "P": p_val}

    return list(grouped.values())


def main():
    parser = argparse.ArgumentParser(description="Least-squares UWB 4-delay calibration solver")
    parser.add_argument("file", nargs="?", help="JSON or CSV file containing survey measurements")
    parser.add_argument("--demo", action="store_true", help="Run calibration on synthetic demo survey data")
    parser.add_argument("--apply", action="store_true", help="Patch calibrated ADELAY values directly into .ino files")
    args = parser.parse_args()

    if args.demo or not args.file:
        print("[INFO] Running demo survey calibration workflow...")
        survey_points = generate_demo_survey()
    else:
        survey_points = load_survey_file(args.file)

    res = solve_4_delay_values(survey_points)
    print_calibration_report(res)

    if args.apply:
        print("\n[INFO] Applying calibrated ADELAY constants to firmware files...")
        root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        patch_ino_file(os.path.join(root_dir, "Tag", "Tag.ino"), res["solved_adelays"]["Tag"])
        patch_ino_file(os.path.join(root_dir, "Anchor1", "Anchor1.ino"), res["solved_adelays"]["1782"])
        patch_ino_file(os.path.join(root_dir, "Anchor2", "Anchor2.ino"), res["solved_adelays"]["1783"])
        patch_ino_file(os.path.join(root_dir, "Anchor3", "Anchor3.ino"), res["solved_adelays"]["1784"])


if __name__ == "__main__":
    main()

