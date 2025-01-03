#include <Arduino.h>
#include <Wire.h>

void scanI2CDevices()
{
    Serial1.println("Scanning I2C bus...");

    for (uint8_t address = 3; address < 128; address++)
    {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0)
        {
            Serial1.print("Device found at 0x");
            Serial1.println(address, HEX);
        }
        delay(5); // Short delay to avoid bus congestion
    }

    Serial1.println("Scan complete.");
}