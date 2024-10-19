#include <Arduino.h>
#include <Adxl.h>
#include <Wire.h>
#include <Lps22.h>
#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN PA4  // Chip Select pin for SD card

File dataFile;

Adxl adxl345 = Adxl(0x1D, ADXL345);
Adxl adxl375 = Adxl(0x53, ADXL375);
Lps22 lps22 = Lps22(0x5C);

const int buzzer = PA3;
const int rbg_red = PB5;
const int rbg_green = PB4;
const int rbg_blue = PB3;

const int igniter_0 = PB10;
const int igniter_1 = PB11;

const int sense0 = PB0;
const int sense1 = PB1;

const int voltage_sense = PA2;

int millisSinceLastLog = 0;

int getNextLogFileNumber() {
  int logNumber = 0;
  
  File root = SD.open("/");
  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      // No more files
      break;
    }
    
    String fileName = entry.name();
    if (fileName.startsWith("log") && fileName.endsWith(".txt")) {
      // Extract the number part of the filename (e.g., "log3.txt" -> 3)
      int currentLogNumber = fileName.substring(3, fileName.length() - 4).toInt();
      if (currentLogNumber > logNumber) {
        logNumber = currentLogNumber;
      }
    }
    entry.close();
  }
  
  // Return the next available log number
  return logNumber + 1;
}

void setup() {
  // Initialize Serial1 communication
  Serial1.begin(9600);
  // Initialize I2C communication on PB8 (SCL) and PB9 (SDA)

  pinMode(buzzer, OUTPUT);
  pinMode(rbg_red, OUTPUT);
  pinMode(rbg_green, OUTPUT);
  pinMode(rbg_blue, OUTPUT);

  pinMode(igniter_0, OUTPUT);
  pinMode(igniter_1, OUTPUT);

  pinMode(sense0, INPUT);
  pinMode(sense1, INPUT);

  pinMode(voltage_sense, INPUT);

  digitalWrite(rbg_blue, HIGH);
  digitalWrite(rbg_red, HIGH);
  digitalWrite(rbg_green, HIGH);

  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();  

  // Wait for Serial1 port to connect (only needed on some boards)
  while (!Serial1);
  delay(4000);
  // for(int i = 0; i < 20; i++){
  //   Serial1.println(i);
  //   delay(500);
  // }

  Serial1.println("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial1.println("Initialization failed!");
    while (1);
  }
  Serial1.println("Initialization done.");

  // Open or create the file
  dataFile = SD.open("log.txt", FILE_WRITE);
  if (!dataFile) {
    Serial1.println("Error opening log.txt");
    while (1);
  }  

  Serial1.println("ADXL345 Sensor Test");

  while (!adxl345.begin()) {
    Serial1.println("Could not find a valid ADXL345 sensor, check wiring!");
    delay(1000);
  }
  Serial1.println("ADXL345 sensor found!");

  while (!adxl375.begin()) {
    Serial1.println("Could not find a valid ADXL375 sensor, check wiring!");
    delay(1000);
    
  }
  Serial1.println("ADXL375 sensor found!");  

  while (!lps22.begin()) {
    Serial1.println("Could not find a valid LPS22 sensor, check wiring!");
    delay(1000);
  }
  Serial1.println("LPS22 sensor found!");

  digitalWrite(rbg_green, LOW);


}

void loop() { //375 works, 345 does not
  int16_t x, y, z;
  adxl345.readAccelerometer(&x, &y, &z);

  int16_t x2, y2, z2;
  adxl375.readAccelerometer(&x2, &y2, &z2);

  int32_t pressure;
  lps22.readPressure(&pressure);

  int16_t temperature;
  lps22.readTemperature(&temperature);

  // ADXL345 scaling factor
  float scale345 = 0.0039; // ±16g at 4 mg/LSB

  // ADXL375 scaling factor
  float scale375 = 0.049;  // ±200g at ~49 mg/LSB  

  // Convert raw values to 'g' units (assuming 4 mg/LSB at +/-16g)
  float xg = x * scale345;
  float yg = y * scale345;
  float zg = z * scale345;

  float xg2 = x2 * scale375;
  float yg2 = y2 * scale375;
  float zg2 = z2 * scale375;

  Serial1.print("X: ");
  Serial1.print(xg);
  Serial1.print(" g, ");
  Serial1.print(xg2);
  Serial1.print(" g, Y: ");
  Serial1.print(yg);
  Serial1.print(" g, ");
  Serial1.print(yg2);
  Serial1.print(" g, Z: ");
  Serial1.print(zg);
  Serial1.print(" g, ");
  Serial1.print(zg2);
  Serial1.print(" g");
  Serial1.print(", Pressure: ");
  Serial1.print(pressure / 4096.0);
  Serial1.print(" hPa, Temperature: ");
  Serial1.print(temperature / 100.0);
  Serial1.print(" °C");
  // Serial1.print(", Igniter 0: ");
  // Serial1.print(analogRead(sense0));
  // Serial1.print(", Igniter 1: ");
  // Serial1.print(analogRead(sense1));
  // Serial1.print(", Voltage: ");
  // Serial1.print(analogRead(voltage_sense));
  Serial1.println();

  // Write data to SD card
  dataFile.print(xg);
  dataFile.print(",");
  dataFile.print(yg);
  dataFile.print(",");
  dataFile.print(zg);
  dataFile.print(",");
  dataFile.print(xg2);
  dataFile.print(",");
  dataFile.print(yg2);
  dataFile.print(",");
  dataFile.print(zg2);
  dataFile.print(",");
  dataFile.print(pressure / 4096.0);
  dataFile.print(",");
  dataFile.print(temperature / 100.0);
  dataFile.println();

  if(millis() - millisSinceLastLog > 2000){
    dataFile.flush();
    millisSinceLastLog = millis();
  }
  // save SD Card data every 5 seconds
  //digitalWrite(buzzer, LOW);
  delay(200);
  //digitalWrite(buzzer, HIGH);
}
