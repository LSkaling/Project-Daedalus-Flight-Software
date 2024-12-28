#include <Arduino.h>
#include <Adxl.h>
#include <Wire.h>
#include <Lps22.h>
#include <SPI.h>
#include <SD.h>
#include <ACAN2517FD.h>
#include <Moteus.h>
#include <Buzzer.h>
#include <PinDefinitions.h>

const bool debug = true; //Changes if Serial1 is used

long last_command = 0;

int millisSinceLastLog = 0;
int millisSinceLastBeep = 0;

static uint32_t gNextSendMillis = 0;



ACAN2517FD can (PinDefs.MCP_CS, SPI, PinDefs.MCP_INT);
File dataFile;
Adxl adxl345 = Adxl(0x1D, ADXL345);
Adxl adxl375 = Adxl(0x53, ADXL375);
Lps22 lps22 = Lps22(0x5C);
Buzzer buzzer = Buzzer(PinDefs.BUZZER);

Moteus moteus1(can, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());

Moteus::PositionMode::Command position_cmd;
Moteus::PositionMode::Format position_fmt;

int getNextLogFileNumber() {
  int logNumber = 0;
  
  File root = SD.open("/");
  Serial1.println("Files found on the SD card:");
  while (true) {
    File entry = root.openNextFile();
    Serial1.print("  ");
    Serial1.println(entry.name());
    if (!entry) {
      // No more files
      break;
    }
    
    String fileName = entry.name();
    if (fileName.startsWith("LOG") && fileName.endsWith(".TXT")) {
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
  Serial1.begin(115200);
  // Initialize I2C communication on PB8 (SCL) and PB9 (SDA)

  digitalWrite(PinDefs.STATUS_LED_RED, HIGH);
  digitalWrite(PinDefs.STATUS_LED_GREEN, HIGH);
  digitalWrite(PinDefs.STATUS_LED_BLUE, HIGH);

  // Wait for Serial1 port to connect (only needed on some boards)
  while (!Serial1 && debug);
  delay(4000);
  // for(int i = 0; i < 20; i++){
  //   Serial1.println(i);
  //   delay(500);
  // }  

  Wire.setSDA(PinDefs.SDA);
  Wire.setSCL(PinDefs.SCL);
  Wire.begin();  

  SPI.setMOSI(PinDefs.SDI);
  SPI.setMISO(PinDefs.SDO);
  SPI.setSCLK(PinDefs.SCK); 
  SPI.begin();  

  ACAN2517FDSettings settings(
      ACAN2517FDSettings::OSC_40MHz, 1000ll * 1000ll, DataBitRateFactor::x1);  

  // The atmega32u4 on the CANbed has only a tiny amount of memory.
  // The ACAN2517FD driver needs custom settings so as to not exhaust
  // all of SRAM just with its buffers.
  settings.mArbitrationSJW = 2;
  settings.mDriverTransmitFIFOSize = 1;
  settings.mDriverReceiveFIFOSize = 2;

  const uint32_t errorCode = can.begin(settings, [] { can.isr(); });

  while (errorCode != 0) {
    Serial1.print(F("CAN error 0x"));
    Serial1.println(errorCode, HEX);
    delay(1000);
  }      

  // To clear any faults the controllers may have, we start by sending
  // a stop command to each.
  moteus1.SetStop();
  Serial1.println(F("all stopped"));

  position_fmt.velocity_limit = Moteus::kFloat;
  position_fmt.accel_limit = Moteus::kFloat;

  Serial1.println("Initializing SD card...");

  if (!SD.begin(PinDefs.SD_CS)) {
    Serial1.println("Initialization failed!");
    while (1);
  }
  Serial1.println("Initialization done.");

  // Find the next available log file number
  int logNumber = getNextLogFileNumber();
  String logFileName = "log" + String(logNumber) + ".txt";
  
  Serial1.print("Creating log file: ");
  Serial1.println(logFileName);

  // Create the log file
  dataFile = SD.open(logFileName, FILE_WRITE);
  if (!dataFile) {
    Serial1.println("Error opening log file.");
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

  dataFile.println("Time (ms),X (g),Y (g),Z (g),X2 (g),Y2 (g),Z2 (g),Pressure (hPa),Temperature (°C)");
  dataFile.flush();

  buzzer.slowBeep();

}

uint16_t gLoopCount = 0;

void loop() {

  // We intend to send control frames every 20ms.
  const auto time = millis();
  if (gNextSendMillis >= time)
  {
    return;
  }

  gNextSendMillis += 20;
  gLoopCount++;


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

  Moteus::PositionMode::Command cmd;
  cmd.position = NaN;
  cmd.velocity = 5;

  moteus1.SetPosition(cmd);

  if (gLoopCount % 5 != 0)
  {
    return;
  }

  if(debug){

    auto print_moteus = [](const Moteus::Query::Result &query)
    {
      Serial1.print(static_cast<int>(query.mode));
      Serial1.print(F(" "));
      Serial1.print(query.position);
      Serial1.print(F("  velocity "));
      Serial1.print(query.velocity);
    };

    print_moteus(moteus1.last_result().values);

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
    Serial1.println(" °C");
  }

  // Write data to SD card
  dataFile.print(millis());
  dataFile.print(",");
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

  buzzer.update();
  //save SD Card data every 5 seconds

  
}