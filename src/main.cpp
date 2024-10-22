#include <Arduino.h>
#include <Adxl.h>
#include <Wire.h>
#include <Lps22.h>
#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN PA4  // Chip Select pin for SD card

const int rbg_red = PB5;
const int rbg_green = PB4;
const int rbg_blue = PB3;


void setup() {
  // Initialize Serial1 communication
  Serial1.begin(9600);
  Serial1.println("Connected!");

  pinMode(rbg_red, OUTPUT);
  pinMode(rbg_green, OUTPUT);
  pinMode(rbg_blue, OUTPUT);

  digitalWrite(rbg_blue, HIGH);
  digitalWrite(rbg_red, HIGH);
  digitalWrite(rbg_green, LOW);

}

void loop() { //375 works, 345 does not
 
}
