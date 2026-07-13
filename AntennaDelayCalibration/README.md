# UWB Antenna Delay (ADELAY) Least-Squares Calibration

This folder implements the complete multi-anchor **Least-Squares Antenna Delay Calibration** architecture depicted below:

```
[Survey points: known xy, 3 anchors]
                │
                ▼
  [Least-squares: solve 4 delay values]
                │
                ▼
  [Burn constants: fixed at runtime]
                │
                ▼
[Range + RX power: all 3 anchors, live]
                │
                ▼
[Apply bias curve: vs RX power, not xy]
                │
                ▼
[GN + EKF solve: estimate tag xy]
```

---

## 1. Why Least-Squares 4-Delay Calibration?

Traditional single-device calibration or runtime auto-offsets assume symmetric behavior or constant range offsets across space. However:
1. **Signal level (RX power)** introduces a known hardware range bias (DecaWave APS011). This must be corrected **as a function of RX power (`vs RX power, not xy`)** before estimating true hardware antenna delays.
2. Every physical UWB board (Tag, Anchor 1, Anchor 2, Anchor 3) has its own hardware antenna delay (`ADELAY`). By surveying the tag at known $(x, y)$ coordinates, we can solve for all **4 delay values** simultaneously using **Least-Squares (`Least-squares: solve 4 delay values`)**.
3. Once calibrated, these 4 `ADELAY` values are **burned as fixed constants at runtime (`Burn constants: fixed at runtime`)**, replacing runtime auto-calibration hacks.

---

## 2. Step-by-Step Workflow

### Step 1: Survey Points (Known XY, 3 Anchors)
Place your anchors at their surveyed coordinates (e.g., matching `ANCHORS` in `uwb_solver.py`):
* `Anchor 1 (1782)`: `(0.00, 0.00)` m
* `Anchor 2 (1783)`: `(0.90, 0.00)` m
* `Anchor 3 (1784)`: `(0.00, 0.90)` m

Place the Tag at one or more known surveyed points $(x, y)$ (e.g., `(0.30, 0.30)` m). Record the measured ranges (`R`) and RX power (`P`) to all three anchors into a JSON file such as `survey_example.json`:

```json
[
  {
    "tag_xy": [0.30, 0.30],
    "measurements": {
      "1782": {"R": 0.518, "P": -75.2},
      "1783": {"R": 0.650, "P": -78.4},
      "1784": {"R": 0.680, "P": -79.1}
    }
  }
]
```

> **Tip:** You can flash `AntennaDelayCalibration.ino` onto your Tag to stream raw `{"CAL_SAMPLE": ...}` lines over serial while placing the tag at surveyed points.

---

### Step 2: Run Least-Squares 4-Delay Calibration

Run the calibration script:

```bash
# Test with synthetic demo survey dataset
python calibrate_delay.py --demo

# Or run on your recorded survey dataset
python calibrate_delay.py survey_example.json
```

The script will:
1. **Apply bias curve (`vs RX power, not xy`)**: Automatically subtract DecaWave APS011 Table 2 RX power range bias from every measured sample.
2. **Least-squares (`solve 4 delay values`)**: Formulate and solve the linear system for `Tag`, `Anchor 1 (1782)`, `Anchor 2 (1783)`, and `Anchor 3 (1784)` antenna delay offsets.
3. Output a detailed calibration report showing RMSE before and after calibration, along with the exact `ANTENNA_DELAY` ticks for each device.

---

### Step 3: Burn Constants (Fixed at Runtime)

Copy the output `#define ANTENNA_DELAY <value>` constants into each board's `.ino` file:
* `Tag/Tag.ino` -> `#define ANTENNA_DELAY <Tag_calibrated_value>`
* `Anchor1/Anchor1.ino` -> `#define ANTENNA_DELAY <Anchor1_calibrated_value>`
* `Anchor2/Anchor2.ino` -> `#define ANTENNA_DELAY <Anchor2_calibrated_value>`
* `Anchor3/Anchor3.ino` -> `#define ANTENNA_DELAY <Anchor3_calibrated_value>`

Or automatically patch all `.ino` files directly:

```bash
python calibrate_delay.py survey_example.json --apply
```

---

### Step 4: Live Positioning (`uwb_solver.py`)

Once the hardware `ADELAY` constants are burned into your boards:
1. `uwb_solver.py` receives live raw ranges + RX power (`Range + RX power: all 3 anchors, live`).
2. `uwb_solver.py` applies the DecaWave APS011 RX power bias curve (`Apply bias curve: vs RX power, not xy`).
3. Gauss-Newton least-squares trilateration + constant-velocity EKF estimates the tag's real-time position (`GN + EKF solve: estimate tag xy`).
