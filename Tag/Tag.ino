#include <SPI.h>
#include <DW1000Ranging.h>
#include "link.h"
#include <math.h>

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
#define PIN_RST 27
#define PIN_IRQ 34

// ----------------------------------------------------------------------
// ANTENNA DELAY - calibrated per-device via AntennaDelayCalibration.ino
// (Decawave APS014 method). This replaces the old runtime auto-offset
// entirely. Every board gets its OWN measured value; they'll be close
// but flash each device with its own number, not just this one.
// ----------------------------------------------------------------------
#define ANTENNA_DELAY 16493  // <-- Calibrated via Least-Squares 4-Delay solver

struct MyLink *uwb_data;
unsigned long runtime = 0;

// Raw (median-filtered only) ranges to each anchor. No offset subtraction,
// no EMA - the antenna delay register handles the constant bias now, and
// the PC-side EKF (uwb_solver.py) handles the smoothing.
float range_a1 = 0.0; // range to Anchor 1
float range_a2 = 0.0; // range to Anchor 2
float range_a3 = 0.0; // range to Anchor 3
bool ranges_ready = false;

// RX power (dBm) per anchor. This is what actually changes as the tag
// moves - closer/LOS = strong (e.g. -70), farther/obstructed = weak
// (e.g. -95+). Forwarded to the PC so the EKF can trust weak links less
// in real time, instead of treating every range as equally reliable.
int8_t power_a1 = -100, power_a2 = -100, power_a3 = -100;

const float MAX_VALID_RANGE = 10.0;  // Reject ranges above this (meters)
const float MIN_VALID_RANGE = 0.01;  // Reject ranges below this (meters)

// --- 5-sample Moving Median Filter to eliminate multipath spikes ---
float median_buf_r1[5] = {0};
float median_buf_r2[5] = {0};
float median_buf_r3[5] = {0};
int median_idx_r1 = 0, median_idx_r2 = 0, median_idx_r3 = 0;
int median_cnt_r1 = 0, median_cnt_r2 = 0, median_cnt_r3 = 0;

int8_t clamp_power(float dbm)
{
    if (dbm > 0) dbm = 0;
    if (dbm < -128) dbm = -128;
    return (int8_t)dbm;
}

float get_median(float buf[], int count)
{
    if (count <= 0) return 0.0;
    float temp[5];
    for (int i = 0; i < count; i++) temp[i] = buf[i];
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (temp[j] < temp[i]) {
                float swp = temp[i];
                temp[i] = temp[j];
                temp[j] = swp;
            }
        }
    }
    return temp[count / 2];
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== UWB Tag - raw range forwarding (PC does the solve) ===");

    // init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, DW_CS, PIN_IRQ);

    // *** calibrated antenna delay -- set BEFORE startAsTag() ***
    DW1000.setAntennaDelay(ANTENNA_DELAY);

    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

    // Enable the library's built-in range filter to reject multipath jitter
    DW1000Ranging.useRangeFilter(true);

    // Start tag in 64MHz PRF high-accuracy mode (TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ)
    DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_SHORTDATA_FAST_ACCURACY, false);

    uwb_data = init_link();
}

void loop()
{
    DW1000Ranging.loop();

    // Embed raw ranges + RX power into the DW1000Ranging data buffer
    // (bytes 70..86) so Anchor 2 can relay them to the PC. No position
    // math happens here any more - three range floats + three power
    // bytes behind the magic header.
    if (ranges_ready)
    {
        DW1000Ranging.data[70] = 0x77;
        DW1000Ranging.data[71] = 0x88;
        memcpy(&DW1000Ranging.data[72], &range_a1, sizeof(float));
        memcpy(&DW1000Ranging.data[76], &range_a2, sizeof(float));
        memcpy(&DW1000Ranging.data[80], &range_a3, sizeof(float));
        DW1000Ranging.data[84] = (uint8_t)power_a1;
        DW1000Ranging.data[85] = (uint8_t)power_a2;
        DW1000Ranging.data[86] = (uint8_t)power_a3;
    }

    if ((millis() - runtime) > 1000)
    {
        if (ranges_ready)
        {
            Serial.print("R1:");
            Serial.print(range_a1, 3);
            Serial.print(" R2:");
            Serial.print(range_a2, 3);
            Serial.print(" R3:");
            Serial.println(range_a3, 3);
        }
        runtime = millis();
    }
}

void newRange()
{
    uint16_t addr = DW1000Ranging.getDistantDevice()->getShortAddress();
    float range = DW1000Ranging.getDistantDevice()->getRange();
    float dbm = DW1000Ranging.getDistantDevice()->getRXPower();

    // --- Reject invalid/outlier ranges ---
    if (range < MIN_VALID_RANGE || range > MAX_VALID_RANGE)
    {
        return; // discard
    }

    fresh_link(uwb_data, addr, range, dbm);

    bool is_anchor1 = (addr == 0x24DC || addr == 0x8217 || addr == 0x1782 || addr == 0xDCC7);
    bool is_anchor2 = (addr == 0x64B7 || addr == 0x8317 || addr == 0x1783 || addr == 0xB764);
    bool is_anchor3 = (addr == 0x1784 || addr == 0x8417 || addr == 0x84DC);

    if (is_anchor1)
    {
        median_buf_r1[median_idx_r1] = range;
        median_idx_r1 = (median_idx_r1 + 1) % 5;
        if (median_cnt_r1 < 5) median_cnt_r1++;
        range_a1 = get_median(median_buf_r1, median_cnt_r1);
        power_a1 = clamp_power(dbm);
    }
    else if (is_anchor2)
    {
        median_buf_r2[median_idx_r2] = range;
        median_idx_r2 = (median_idx_r2 + 1) % 5;
        if (median_cnt_r2 < 5) median_cnt_r2++;
        range_a2 = get_median(median_buf_r2, median_cnt_r2);
        power_a2 = clamp_power(dbm);
    }
    else if (is_anchor3)
    {
        median_buf_r3[median_idx_r3] = range;
        median_idx_r3 = (median_idx_r3 + 1) % 5;
        if (median_cnt_r3 < 5) median_cnt_r3++;
        range_a3 = get_median(median_buf_r3, median_cnt_r3);
        power_a3 = clamp_power(dbm);
    }
    else
    {
        return;
    }

    // The PC solver only needs >=2 anchors to get a fix (3 to fully kill
    // the half-plane ambiguity), so start forwarding as soon as we have
    // at least two valid ranges rather than waiting on all three.
    int have = (range_a1 > 0.0) + (range_a2 > 0.0) + (range_a3 > 0.0);
    if (have >= 2)
    {
        ranges_ready = true;
    }
}

void newDevice(DW1000Device *device)
{
    Serial.print("ranging init; 1 device added ! -> short:0x");
    Serial.println(device->getShortAddress(), HEX);
    add_link(uwb_data, device->getShortAddress());
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: 0x");
    Serial.println(device->getShortAddress(), HEX);
    delete_link(uwb_data, device->getShortAddress());
}
