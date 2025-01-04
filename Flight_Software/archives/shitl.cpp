#include "shitl.h"

// Constructor - Set I2C address
Shitl::Shitl(uint8_t i2cAddress)
{
    this->i2cAddress = i2cAddress;
}

// Initialize I2C
void Shitl::begin()
{
    Wire.begin(); // Initialize I2C as master
}

// Send command to I2C slave and read response
String Shitl::write(const String &command)
{
    // Send command to I2C slave (STM32)
    Wire.beginTransmission(i2cAddress);
    Wire.write(command.c_str(), command.length()); // Send string command
    Wire.endTransmission();

    // Request data from the slave
    Wire.requestFrom(i2cAddress, 32); // Request up to 32 bytes (max I2C buffer)

    // Wait for response and read data
    String response = "";
    while (Wire.available())
    {
        char c = Wire.read();
        response += c;
    }

    return response.length() > 0 ? response : "NO RESPONSE";
}
