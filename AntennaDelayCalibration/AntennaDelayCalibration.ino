#include <SPI.h>
#include "DW1000Ranging.h"

// ============================================================================
//  AntennaDelayCalibration.ino
// ----------------------------------------------------------------------------
//  Survey Point Data Collection & Standalone Antenna Delay Calibration Sketch
//
//  Workflow matching the UWB Calibration Architecture:
//    1. [Survey points: known xy, 3 anchors]
//       Place Tag at known surveyed coordinates (x, y).
//    2. Collect raw ranges & RX power (dBm) for all 3 anchors.
//    3. Run PC-side script: `python calibrate_delay.py survey_data.json`
//       -> Applies RX power bias curve (`vs RX power, not xy`)
//       -> Solves 4 delay values via Least-Squares (`solve 4 delay values`)
//    4. [Burn constants: fixed at runtime]
//       Copy the solved #define ANTENNA_DELAY value into each board's sketch.
// ============================================================================

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
#define PIN_RST 27
#define PIN_IRQ 34

// Default placeholder uncalibrated antenna delay
#define ANTENNA_DELAY 16512

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== UWB Survey & Antenna Delay Calibration Sketch ===");

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, DW_CS, PIN_IRQ);

    DW1000.setAntennaDelay(ANTENNA_DELAY);

    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

    DW1000Ranging.useRangeFilter(true);

    // Start in calibration tag mode
    DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_SHORTDATA_FAST_ACCURACY, false);
}

void loop() {
    DW1000Ranging.loop();
}

void newRange() {
    uint16_t addr = DW1000Ranging.getDistantDevice()->getShortAddress();
    float range = DW1000Ranging.getDistantDevice()->getRange();
    float rxPower = DW1000Ranging.getDistantDevice()->getRXPower();

    // Print JSON survey format ready for calibrate_delay.py capture
    Serial.print("{\"CAL_SAMPLE\":{\"A\":\"");
    Serial.print(addr, HEX);
    Serial.print("\",\"R\":");
    Serial.print(range, 4);
    Serial.print(",\"P\":");
    Serial.print(rxPower, 1);
    Serial.println("}}");
}

void newDevice(DW1000Device *device) {
    Serial.print("Device detected: short:0x");
    Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
    Serial.print("Inactive device: short:0x");
    Serial.println(device->getShortAddress(), HEX);
}
r