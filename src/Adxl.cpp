#include "Adxl.h"

Adxl::Adxl(uint8_t address, ADXL_TYPE adxlType)
{
    ADXL_ADDRESS = address;
    adxlType = adxlType;
}

bool Adxl::begin()
{
    uint8_t deviceId = readRegister(0x00);
    if (deviceId != 0xE5)
    {
        Serial1.print("Error: Could not find ADXL sensor with address 0x");
        Serial1.println(ADXL_ADDRESS, HEX);
        return false;
    }

    writeRegister(0x2D, 0x08); // Power on the sensor
    writeRegister(0x31, 0x09); // Resolution and range: ±16g or appropriate setting for ADXL375
    return true;
}

void Adxl::readAccelerometer(int16_t *x, int16_t *y, int16_t *z)
{
  uint8_t buffer[6];

  // Start I2C transmission and write the register address
  Wire.beginTransmission(ADXL_ADDRESS);
  Wire.write(0x32); // Start reading from data registers
  if (Wire.endTransmission(false) != 0) { // Check if transmission failed
    Serial1.println("Error: Failed to write to ADXL345!");
    return; // Exit if transmission failed
  }

  // Request 6 bytes of data from the ADXL sensor
  uint8_t bytesReceived = Wire.requestFrom(ADXL_ADDRESS, (uint8_t)6);

  // Check if we received all the bytes we expect
  if (bytesReceived != 6) {
    Serial1.print("Error: Expected 6 bytes, received ");
    Serial1.println(bytesReceived);
    return; // Exit if we didn't receive the expected data
  }

  // Read the 6 bytes into the buffer
  for (int i = 0; i < 6; i++) {
    if (Wire.available()) {
      buffer[i] = Wire.read();
    } else {
      Serial1.println("Error: Data not available!");
      return; // Exit if data is not available
    }
  }

  // Combine bytes into 16-bit signed values
  *x = (int16_t)(buffer[1] << 8 | buffer[0]);
  *y = (int16_t)(buffer[3] << 8 | buffer[2]);
  *z = (int16_t)(buffer[5] << 8 | buffer[4]);

  // Optional: Print the raw data for debugging
  // Serial1.print("X: "); Serial1.print(*x);
  // Serial1.print(" Y: "); Serial1.print(*y);
  // Serial1.print(" Z: "); Serial1.println(*z);
}


void Adxl::writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(ADXL_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t Adxl::readRegister(uint8_t reg)
{
  Wire.beginTransmission(ADXL_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL_ADDRESS, (uint8_t)1);
  uint8_t value = 0;
  if (Wire.available())
  {
    value = Wire.read();
  }
  return value;
}

void Adxl::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len)
{
  Wire.beginTransmission(ADXL_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL_ADDRESS, len);
  for (uint8_t i = 0; i < len; i++)
  {
    if (Wire.available())
    {
      buffer[i] = Wire.read();
    }
  }
}