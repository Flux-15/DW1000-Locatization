# What changed vs. the original repo

**Architecture before:** Tag ranges to Anchor1/Anchor2, computes (x,y) on-device
with a hardcoded 2-anchor triangle formula, self-calibrates a range offset at
runtime, embeds (x,y,r1,r2) in the ranging payload, Anchor2 relays it as JSON.
Two anchors = half-plane ambiguity (the "wall").

**Architecture now:** Tag just ranges to three anchors, median-filters each
range, and forwards the three raw ranges. Anchor2 relays them verbatim.
`uwb_solver.py` on the PC does Gauss-Newton trilateration + a constant-velocity
EKF. Three anchors (with #3 off the A1-A2 line) removes the ambiguity, so the
tag can be solved anywhere in the XY plane, not just one side of it.

## Per-file changes

- **Tag.ino**
  - Added `DW1000.setAntennaDelay(ANTENNA_DELAY)` before `startAsTag`.
  - Removed `update_auto_calibration()`, `range_offset`, `min_range_sum`,
    `cal_sample_count`, and the `ema_rX - range_offset` subtraction — the
    antenna delay register replaces this now.
  - Removed the EMA layer — it just adds lag on top of what the PC EKF
    already does better with a motion model.
  - Removed `calculate_tag_pos()` and the 1cm rounding — no more on-tag
    position math.
  - Added Anchor 3 detection + its own 5-sample median buffer.
  - Payload now carries `range_a1, range_a2, range_a3` (bytes 72-83) behind
    the same `0x77 0x88` magic header, instead of `x, y, r1, r2`.
  - Starts forwarding once it has ranges from >=2 anchors rather than
    waiting on all three, so it still works if one anchor drops out.

- **Anchor1.ino / Anchor3.ino**
  - Added the same `DW1000.setAntennaDelay(ANTENNA_DELAY)` call. Set each
    board's own calibrated value from `AntennaDelayCalibration.ino` — don't
    reuse one number across boards.
  - Anchor3.ino is new: same shape as Anchor1.ino, address `84:17:...` ->
    short address `0x1784`, matching the `"1784"` key already in
    `uwb_solver.py`'s `ANCHORS` dict. Place it physically off the A1-A2 line.

- **Anchor2.ino**
  - Added the antenna delay call.
  - `newRange()` now decodes three raw range floats instead of x/y/r1/r2,
    and emits `{"links":[{"A":"1782",...},{"A":"1783",...},{"A":"1784",...}]}`
    — no more `x`/`y` in the payload; `uwb_solver.py` already ignores those
    if present and recomputes position itself.

- **uwb_solver.py / AntennaDelayCalibration.ino**
  - Unchanged — the solver already expected this raw-range JSON shape and
    already had the 3-anchor `ANCHORS` dict; the calibration sketch is a
    one-time-per-board tool, run it once per device before flashing the
    real firmware.

## Real-time adaptation (added after first pass)

Antenna delay is a fixed hardware constant - it does **not** need to change
as the tag moves near/far, and re-deriving it at runtime is what the old
`update_auto_calibration()` code got wrong. What *does* legitimately change
in real time as range/LOS conditions change is signal quality, so:

- **Tag.ino** now also tracks the latest RX power (dBm) per anchor and packs
  it into the payload as 3 extra bytes (offsets 84-86), right after the
  three range floats.
- **Anchor2.ino** decodes those and adds a `"P"` field per link in the JSON.
- **uwb_solver.py** turns RX power into a per-anchor measurement std
  (`power_to_std()`, tunable via `GOOD_DBM`/`POOR_DBM`/`MEAS_NOISE_MAX`) and
  feeds it into `EKF.update()` each cycle, so a strong close-range anchor
  gets trusted more than a weak/NLOS one, automatically, every sample - no
  geometry assumptions involved. `P` is optional per link; if omitted, that
  anchor just falls back to the fixed `MEAS_NOISE`.

The DW1000 firmware library also already applies its own internal
RX-power-based range **bias** correction (DecaWave APS011) inside
`getRange()`, so you don't need to add anything for that part.

## Complete Antenna Delay (ADELAY) Least-Squares Calibration Architecture

We implement the two-stage calibration & positioning architecture:

```
[Survey points: known xy, 3 anchors] ──► [Least-squares: solve 4 delay values] ──► [Burn constants: fixed at runtime]
                                                                                                  │
                                                                                                  ▼
[Range + RX power: all 3 anchors, live] ──► [Apply bias curve: vs RX power, not xy] ──► [GN + EKF solve: estimate tag xy]
```

### 1. Calibration Phase (`AntennaDelayCalibration/calibrate_delay.py`)
1. **Survey points (known xy, 3 anchors)**: Place the Tag at one or more surveyed coordinates $(x,y)$ with 3 known anchors. Collect measured ranges and RX power (`P`).
2. **Apply bias curve (vs RX power, not xy)**: The calibration solver (`calibrate_delay.py`) applies the DecaWave APS011 Table 2 range bias curve vs RX power before estimating antenna delays.
3. **Least-squares (solve 4 delay values)**: Formulates and solves the linear system for all 4 devices simultaneously (`Tag`, `Anchor 1`, `Anchor 2`, `Anchor 3`).
4. **Burn constants (fixed at runtime)**: Outputs exact `#define ANTENNA_DELAY <value>` constants to burn into `Tag.ino`, `Anchor1.ino`, `Anchor2.ino`, and `Anchor3.ino` (or run `python calibrate_delay.py --apply` to update the `.ino` files automatically).

### 2. Live Positioning Phase (`uwb_solver.py`)
1. **Range + RX power (all 3 anchors, live)**: PC receives raw ranges and RX power (`P`) in real time.
2. **Apply bias curve (vs RX power, not xy)**: `uwb_solver.py` applies `apply_rx_power_bias(r, p)` to remove signal-strength bias.
3. **GN + EKF solve (estimate tag xy)**: Runs Gauss-Newton least-squares trilateration + constant-velocity EKF on the bias-corrected ranges.
