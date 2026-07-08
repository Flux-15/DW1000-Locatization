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

struct MyLink *uwb_data;
unsigned long runtime = 0;

float tag_x = 0.0;
float tag_y = 0.0;
float range_a1 = 0.0; // Corrected range to Anchor 1
float range_a2 = 0.0; // Corrected range to Anchor 2
bool pos_calculated = false;

const float DISTANCE_A1_A2 = 1.25; // Actual measured distance between Anchor 1 and Anchor 2 in meters

// --- EMA Smoothing ---
const float EMA_ALPHA = 0.2;        // Smoothing factor (0.1=very smooth, 0.5=responsive)
const float MAX_VALID_RANGE = 10.0;  // Reject ranges above this (meters)
const float MIN_VALID_RANGE = 0.01;  // Reject ranges below this (meters)
float ema_r1 = -1.0;  // EMA filtered raw range to Anchor 1
float ema_r2 = -1.0;  // EMA filtered raw range to Anchor 2

// ============================================================
// AUTO-CALIBRATION: Dynamic Range Offset Correction
// ============================================================
// The DW1000 antenna delay adds a CONSTANT offset (delta) to
// every range measurement:
//   measured_range = true_range + delta
//
// We exploit the known baseline (1.25m between anchors):
//   For ANY tag position: r1_true + r2_true >= baseline
//   (equality when tag is on the line between anchors)
//
// So: min(r1_meas + r2_meas) = baseline + 2*delta
//     => delta = (min_sum - baseline) / 2
//
// The Tag continuously tracks the minimum range sum and
// auto-computes the offset. As the tag moves around, the
// estimate improves automatically.
// ============================================================
float range_offset = 0.0;           // Auto-computed per-range offset (subtracted from each raw range)
float min_range_sum = 999.0;        // Tracked minimum of (r1_raw + r2_raw)
const float MIN_SUM_DECAY = 0.0005; // Slowly increase min_sum so it adapts if conditions change
unsigned long cal_sample_count = 0; // How many calibration samples we've collected

// Store latest raw (unfiltered, uncorrected) ranges for offset calculation
float latest_raw_r1 = -1.0;
float latest_raw_r2 = -1.0;

void calculate_tag_pos(float a, float b, float c, float *x, float *y)
{
    // a: corrected distance to Anchor 2
    // b: corrected distance to Anchor 1
    // c: distance between anchors (1.25m)
    if (b <= 0.0 || c <= 0.0) return;
    float cos_a = (b * b + c * c - a * a) / (2.0 * b * c);
    if (cos_a > 1.0) cos_a = 1.0;
    if (cos_a < -1.0) cos_a = -1.0;
    *x = b * cos_a;
    float sin_a_sq = 1.0 - cos_a * cos_a;
    if (sin_a_sq < 0.0) sin_a_sq = 0.0;
    *y = b * sqrt(sin_a_sq);

    // Round to 2 decimal places (1cm resolution)
    *x = round((*x) * 100.0) / 100.0;
    *y = round((*y) * 100.0) / 100.0;
}

void update_auto_calibration()
{
    // We need both raw ranges to compute the sum
    if (latest_raw_r1 <= 0.0 || latest_raw_r2 <= 0.0) return;

    float current_sum = latest_raw_r1 + latest_raw_r2;

    // Update minimum range sum tracker
    if (current_sum < min_range_sum)
    {
        min_range_sum = current_sum; // New minimum found
    }
    else
    {
        // Slowly decay (increase) the minimum so it adapts over time
        // This prevents a single noise spike from locking the offset forever
        min_range_sum += MIN_SUM_DECAY;
    }

    // Compute offset: delta = (min_sum - baseline) / 2
    // Only apply if min_sum > baseline (offset should be positive or zero)
    float new_offset = (min_range_sum - DISTANCE_A1_A2) / 2.0;
    if (new_offset < 0.0) new_offset = 0.0;

    // Smooth the offset update to prevent jumps
    if (cal_sample_count == 0)
        range_offset = new_offset;
    else
        range_offset = 0.05 * new_offset + 0.95 * range_offset; // Very slow adaptation

    cal_sample_count++;
}

// --- 5-sample Moving Median Filter to eliminate multipath spikes ---
float median_buf_r1[5] = {0};
float median_buf_r2[5] = {0};
int median_idx_r1 = 0;
int median_idx_r2 = 0;
int median_cnt_r1 = 0;
int median_cnt_r2 = 0;

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
    Serial.println("=== UWB Tag - 64MHz High Accuracy + Median/EMA Filter ===");
    Serial.print("Known baseline: ");
    Serial.print(DISTANCE_A1_A2, 2);
    Serial.println(" m");

    // init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, DW_CS, PIN_IRQ);
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

    // Embed calculated position and ranges into DW1000Ranging data buffer (bytes 70..87)
    if (pos_calculated)
    {
        DW1000Ranging.data[70] = 0x77;
        DW1000Ranging.data[71] = 0x88;
        memcpy(&DW1000Ranging.data[72], &tag_x, sizeof(float));
        memcpy(&DW1000Ranging.data[76], &tag_y, sizeof(float));
        memcpy(&DW1000Ranging.data[80], &range_a1, sizeof(float));
        memcpy(&DW1000Ranging.data[84], &range_a2, sizeof(float));
    }

    if ((millis() - runtime) > 1000)
    {
        if (pos_calculated)
        {
            Serial.print("POS X:");
            Serial.print(tag_x, 2);
            Serial.print(" Y:");
            Serial.print(tag_y, 2);
            Serial.print(" | R1:");
            Serial.print(range_a1, 2);
            Serial.print(" R2:");
            Serial.print(range_a2, 2);
            Serial.print(" | offset:");
            Serial.print(range_offset, 3);
            Serial.print(" minSum:");
            Serial.print(min_range_sum, 3);
            Serial.print(" samples:");
            Serial.println(cal_sample_count);
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

    if (is_anchor1)
    {
        median_buf_r1[median_idx_r1] = range;
        median_idx_r1 = (median_idx_r1 + 1) % 5;
        if (median_cnt_r1 < 5) median_cnt_r1++;
        float med_range = get_median(median_buf_r1, median_cnt_r1);

        latest_raw_r1 = med_range; // store median range for calibration
        if (ema_r1 < 0.0)
            ema_r1 = med_range;
        else
            ema_r1 = EMA_ALPHA * med_range + (1.0 - EMA_ALPHA) * ema_r1;
    }
    else if (is_anchor2)
    {
        median_buf_r2[median_idx_r2] = range;
        median_idx_r2 = (median_idx_r2 + 1) % 5;
        if (median_cnt_r2 < 5) median_cnt_r2++;
        float med_range = get_median(median_buf_r2, median_cnt_r2);

        latest_raw_r2 = med_range; // store median range for calibration
        if (ema_r2 < 0.0)
            ema_r2 = med_range;
        else
            ema_r2 = EMA_ALPHA * med_range + (1.0 - EMA_ALPHA) * ema_r2;
    }
    else
    {
        return;
    }

    // --- Auto-calibrate offset using raw range sums ---
    update_auto_calibration();

    // --- Calculate position with offset-corrected, EMA-smoothed ranges ---
    if (ema_r1 > 0.0 && ema_r2 > 0.0)
    {
        range_a1 = ema_r1 - range_offset;
        range_a2 = ema_r2 - range_offset;

        if (range_a1 < 0.01) range_a1 = 0.01;
        if (range_a2 < 0.01) range_a2 = 0.01;

        calculate_tag_pos(range_a2, range_a1, DISTANCE_A1_A2, &tag_x, &tag_y);
        pos_calculated = true;
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
