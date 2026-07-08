#include <SPI.h>
#include "DW1000Ranging.h"

#define ANCHOR_ADD "83:17:5B:D5:A9:9A:E2:9C"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

void setup()
{
    Serial.begin(115200);
    delay(1000);
    //init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
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
    // Check if packet contains our magic marker (0x77, 0x88) from Tag at bytes 70 and 71
    if (DW1000Ranging.data[70] == 0x77 && DW1000Ranging.data[71] == 0x88)
    {
        float tag_x, tag_y, r1, r2;
        memcpy(&tag_x, &DW1000Ranging.data[72], sizeof(float));
        memcpy(&tag_y, &DW1000Ranging.data[76], sizeof(float));
        memcpy(&r1, &DW1000Ranging.data[80], sizeof(float));
        memcpy(&r2, &DW1000Ranging.data[84], sizeof(float));

        // Format as JSON and send to PC over Serial
        Serial.print("{\"x\":");
        Serial.print(tag_x, 2);
        Serial.print(",\"y\":");
        Serial.print(tag_y, 2);
        Serial.print(",\"links\":[{\"A\":\"1782\",\"R\":\"");
        Serial.print(r1, 2);
        Serial.print("\"},{\"A\":\"1783\",\"R\":\"");
        Serial.print(r2, 2);
        Serial.println("\"}]}");
    }
    else
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
