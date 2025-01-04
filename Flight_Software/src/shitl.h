#ifndef SHITL_H
#define SHITL_H

#include <Arduino.h>
#include <Wire.h>

class Shitl
{
public:
    // Constructor for I2C with device address
    Shitl(uint8_t i2cAddress);

    // Initialize I2C communication
    void begin();

    // Send command and get response
    String write(const String &command);

private:
    uint8_t i2cAddress;
};

#endif
