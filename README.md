# DW1000-Localization: UWB 2D Positioning System

A high-accuracy, real-time 2D local positioning system using Decawave DW1000 Ultra-Wideband (UWB) transceivers and an ESP32. 
Unlike standard setups that rely on UDP/WiFi networks, this architecture operates completely offline. The Tag calculates its position locally and transmits it directly via UWB back to Anchor 2, which relays the positioning JSON stream over USB Serial to a Python PC GUI.

---

## 📐 System Architecture & Flow

```text
   [ Anchor 1 ] (0.00, 0.00)
    (Stationary, Battery Powered)
         ^
         |
    UWB Ranging (64MHz PRF)
         |
         v
     [ Tag ] (x, y) --------------UWB Relay-------------> [ Anchor 2 ] (1.25, 0.00)
  (Mobile, Triangulation,                              (Stationary, PC USB Connected)
   Auto-Calibrates Offset)                                        |
                                                              Serial (JSON)
                                                                  |
                                                                  v
                                                             [ PC GUI ] (uwb.py)
```

1. **Tag (Mobile)** ranges to both Anchor 1 and Anchor 2.
2. **Tag** filters the ranges and dynamically calculates its 2D coordinates `(x, y)` on the 2D plane.
3. **Tag** writes its coordinates and raw ranges to the standard UWB ranging packet payload (bytes 70-87) using a magic header (`0x77`, `0x88`).
4. **Anchor 2 (Stationary + PC Connected)** receives the ranging payload, decodes the magic header, and writes the coordinate JSON to the USB Serial port.
5. **PC (uwb.py)** reads the serial stream and renders the real-time position on a graphical Turtle screen.

---

## ⚡ Key Calibration & Accuracy Features

### 1. 64MHz PRF High-Accuracy RF Mode
To minimize indoor multipath reflections, all units are configured to:
```cpp
DW1000.MODE_SHORTDATA_FAST_ACCURACY // 6.8 Mbps, 64 MHz PRF, 128 Preamble Length
```
This increases leading-edge pulse detection sharpness compared to low-power long-range modes.

### 2. Dual-Layer Noise Filtering
* **Layer 1: Hardware Range Filter**: Enables the built-in library filter to reject jitter.
* **Layer 2: 5-Sample Moving Median Filter**: Rejects sudden multi-path spikes (e.g. signal bouncing off walls).
* **Layer 3: Exponential Moving Average (EMA)**: Smooths out high-frequency noise with an $\alpha = 0.2$ factor.

### 3. Dynamic Auto-Calibration (Antenna Delay)
The antenna delay adds a constant offset ($\delta$) to every range measurement ($d_{measured} = d_{true} + \delta$).
Since the baseline distance $C$ between Anchor 1 and Anchor 2 is exactly **1.25m**, we know that for any tag position:

$$r_{1_{true}} + r_{2_{true}} \ge 1.25\text{m}$$

*(with equality occurring only when the Tag is directly on the straight line between the two anchors)*

The Tag continuously tracks the minimum of the raw range sum:

$$\delta = \frac{\min(r_{1_{raw}} + r_{2_{raw}}) - 1.25}{2}$$

As the tag moves around (especially between the anchors), it dynamically calibrates and subtracts this offset from both ranges in real time.

---

## 🔌 Hardware Wiring (ESP32 to DW1000)

| DW1000 Pin | ESP32 GPIO Pin | Description |
|---|---|---|
| **VCC** | 3.3V | Power (Use stable 3.3V rail) |
| **GND** | GND | Ground |
| **MISO** | GPIO 19 | SPI MISO |
| **MOSI** | GPIO 23 | SPI MOSI |
| **SCK** | GPIO 18 | SPI Clock |
| **CS** | GPIO 4 | Chip Select |
| **RST** | GPIO 27 | Reset Pin |
| **IRQ** | GPIO 34 | Interrupt Request |

---

## 📡 MAC Addresses & Identification

To see which EUI matches which hardware board, check `mac_addresses.txt`.

| Device | MAC Address (EUI) | Short Address |
|---|---|---|
| **Anchor 1** | `24:DC:C7:2E:2A:58:00:00` | `0x24DC` |
| **Anchor 2** | `64:B7:08:6B:13:CC:00:00` | `0x64B7` |
| **Tag** | `64:B7:08:6B:83:54:00:00` | `0x64B7` (Tag address is local to its firmware) |

---

## 🚀 Getting Started

### 1. Flash the Firmware
Open the Arduino IDE or PlatformIO:
* Upload `Anchor1/Anchor1.ino` to your origin anchor `(0, 0)`.
* Upload `Anchor2/Anchor2.ino` to your second anchor `(1.25, 0)` and keep it plugged into your PC.
* Upload `Tag/Tag.ino` to the mobile tag.

### 2. Run the PC GUI
Install Python dependencies:
```bash
pip install pyserial
```
Launch the real-time visualizer:
```bash
python uwb.py -p COM6
```
*(Replace `COM6` with the COM port assigned to Anchor 2. If you don't know the port, run the command without `-p` and the script will list all active COM ports for you).*
