#include <Arduino.h>
#include <Adxl.h>
#include <Wire.h>
#include <Lps22.h>
#include <SPI.h>
#include <SD.h>


void setup() {
  // Initialize Serial communication at 115200 baud rate
  Serial1.begin(9600);

  // Set the custom SDA and SCL pins
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);

  // Initialize I2C communication
  Wire.begin();
  
  Serial1.println("I2C Scanner ready. Scanning...");

  // Give some time to the user to open the serial monitor
  delay(2000);
}

void loop() {
  byte error, address;
  int nDevices;

  Serial1.println("Scanning...");

  nDevices = 0;
  // Loop through all possible I2C addresses (1 to 127)
  for (address = 1; address < 127; address++) {
    // Begin transmission to the device at this address
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    // If the device responds with no error (error == 0), a device is present
    if (error == 0) {
      Serial1.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial1.println(address, HEX);
      nDevices++;
    } 
    // If error == 4, there's an unknown error
    else if (error == 4) {
      Serial1.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial1.println(address, HEX);
    }
  }

  // If no devices were found, print a message
  if (nDevices == 0)
    Serial1.println("No I2C devices found\n");
  else
    Serial1.println("done\n");

  // Wait for 5 seconds before scanning again
  delay(5000);
}
