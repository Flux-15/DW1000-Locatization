#include <SPI.h>
#include "DW1000Ranging.h"

// New third anchor. Short address resolves to 0x1784, matching the "1784"
// key in ANCHORS inside uwb_solver.py. Physically place this one OFF the
// A1-A2 line -- that's what kills the half-plane ("wall") ambiguity and
// gives full XY coverage instead of just a half-plane.
#define ANCHOR_ADD "84:17:5B:D5:A9:9A:E2:9C"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// Calibrated antenna delay for THIS board (AntennaDelayCalibration.ino).
// Every board gets its own measured value - don't just copy this number.
#define ANTENNA_DELAY 16535  // <-- Calibrated via Least-Squares 4-Delay solver (Anchor 3: 0,90 cm)

void setup()
{
    Serial.begin(115200);
    delay(1000);
    //init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

    // *** calibrated antenna delay -- set BEFORE startAsAnchor() ***
    DW1000.setAntennaDelay(ANTENNA_DELAY);

    //define the sketch as anchor. It will be great to dynamically change the type of module
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachBlinkDevice(newBlink);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    // Enable the library's built-in range filter to reject multipath jitter
    DW1000Ranging.useRangeFilter(true);

    // Start anchor in 64MHz PRF high-accuracy mode (TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ)
    DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_ACCURACY, false);
}

void loop()
{
    DW1000Ranging.loop();
}

void newRange()
{
    Serial.print("from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print("\t Range: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRange());
    Serial.print(" m");
    Serial.print("\t RX power: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
    Serial.println(" dBm");
}

void newBlink(DW1000Device *device)
{
    Serial.print("blink; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
}
