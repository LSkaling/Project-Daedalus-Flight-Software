#include <Arduino.h>
#include <Adxl.h>
#include <Wire.h>
#include <Lps22.h>
#include <SPI.h>
#include <SD.h>
#include <ACAN2517FD.h>
#include <Moteus.h>

#define SD_CS_PIN PA4  // Chip Select pin for SD card

const bool debug = true; //Changes if Serial1 is used

long last_command = 0;

File dataFile;

Adxl adxl345 = Adxl(0x1D, ADXL345);
Adxl adxl375 = Adxl(0x53, ADXL375);
Lps22 lps22 = Lps22(0x5C);

const int servo = PA1;

const int voltage_sense = PA2;

const int buzzer = PA3;
const int rbg_red = PB5;
const int rbg_green = PB4;
const int rbg_blue = PB3;

const int esp32_cs = PA12;

const int igniter_0 = PB10;
const int igniter_1 = PB11;

const int sense0 = PB0;
const int sense1 = PB1;

int millisSinceLastLog = 0;
int millisSinceLastBeep = 0;

static uint32_t gNextSendMillis = 0;

static const byte MCP2517_SCK =  PA5; // SCK input of MCP2517
static const byte MCP2517_SDI =  PA7; // SDI input of MCP2517
static const byte MCP2517_SDO =   PA6; // SDO output of MCP2517

static const byte MCP2517_CS  =  PA11; // CS input of MCP2517
static const byte MCP2517_INT =  PA8; // INT output of MCP2517

ACAN2517FD can (MCP2517_CS, SPI, MCP2517_INT) ;

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

struct MotorState {
    float position;     // 4 bytes
    float velocity;     // 4 bytes
    uint8_t fault;      // 1 byte
    float torque;       // 4 bytes
    float q_current;    // 4 bytes
    float voltage;      // 4 bytes
};


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

  // Wait for Serial1 port to connect (only needed on some boards)
  while (!Serial1 && debug);
  delay(4000);
  // for(int i = 0; i < 20; i++){
  //   Serial1.println(i);
  //   delay(500);
  // }  

  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();  

  SPI.setMOSI(PA7);
  SPI.setMISO(PA6);
  SPI.setSCLK(PA5); 
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

  // Serial1.println("Initializing SD card...");

  // if (!SD.begin(SD_CS_PIN)) {
  //   Serial1.println("Initialization failed!");
  //   while (1);
  // }
  // Serial1.println("Initialization done.");

  // // Find the next available log file number
  // int logNumber = getNextLogFileNumber();
  // String logFileName = "log" + String(logNumber) + ".txt";
  
  // Serial1.print("Creating log file: ");
  // Serial1.println(logFileName);

  // // Create the log file
  // dataFile = SD.open(logFileName, FILE_WRITE);
  // if (!dataFile) {
  //   Serial1.println("Error opening log file.");
  //   while (1);
  // }

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

  digitalWrite(rbg_green, LOW);


}

uint16_t gLoopCount = 0;

void loop() {

  // // We intend to send control frames every 20ms.
  // const auto time = millis();
  // if (gNextSendMillis >= time)
  // {
  //   return;
  // }

  // gNextSendMillis += 20;
  // gLoopCount++;


  // int16_t x, y, z;
  // adxl345.readAccelerometer(&x, &y, &z);

  // int16_t x2, y2, z2;
  // adxl375.readAccelerometer(&x2, &y2, &z2);

  // int32_t pressure;
  // lps22.readPressure(&pressure);

  // int16_t temperature;
  // lps22.readTemperature(&temperature);

  // // ADXL345 scaling factor
  // float scale345 = 0.0039; // ±16g at 4 mg/LSB

  // // ADXL375 scaling factor
  // float scale375 = 0.049;  // ±200g at ~49 mg/LSB  

  // // Convert raw values to 'g' units (assuming 4 mg/LSB at +/-16g)
  // float xg = x * scale345;
  // float yg = y * scale345;
  // float zg = z * scale345;

  // float xg2 = x2 * scale375;
  // float yg2 = y2 * scale375;
  // float zg2 = z2 * scale375;

  // Moteus::PositionMode::Command cmd;
  // cmd.position = ((millis() / 1000) % 2) * 25;
  // cmd.velocity = 2;
  // cmd.accel_limit = 200;
  // cmd.velocity_limit = 200;
  // moteus1.SetPosition(cmd, &position_fmt);

  // if(debug){

  //   auto print_moteus = [](const Moteus::Query::Result &query)
  //   {
  //     Serial1.print(static_cast<int>(query.mode));
  //     Serial1.print(F(" "));
  //     Serial1.print(query.position);
  //     Serial1.print(F("  velocity "));
  //     Serial1.print(query.velocity);
  //   };

  //   print_moteus(moteus1.last_result().values);

  //   Serial1.print("X: ");
  //   Serial1.print(xg);
  //   Serial1.print(" g, ");
  //   Serial1.print(xg2);
  //   Serial1.print(" g, Y: ");
  //   Serial1.print(yg);
  //   Serial1.print(" g, ");
  //   Serial1.print(yg2);
  //   Serial1.print(" g, Z: ");
  //   Serial1.print(zg);
  //   Serial1.print(" g, ");
  //   Serial1.print(zg2);
  //   Serial1.print(" g");
  //   Serial1.print(", Pressure: ");
  //   Serial1.print(pressure / 4096.0);
  //   Serial1.print(" hPa, Temperature: ");
  //   Serial1.print(temperature / 100.0);
  //   Serial1.println(" °C");
  // }

  // // Write data to SD card
  // dataFile.print(millis());
  // dataFile.print(",");
  // dataFile.print(xg);
  // dataFile.print(",");
  // dataFile.print(yg);
  // dataFile.print(",");
  // dataFile.print(zg);
  // dataFile.print(",");
  // dataFile.print(xg2);
  // dataFile.print(",");
  // dataFile.print(yg2);
  // dataFile.print(",");
  // dataFile.print(zg2);
  // dataFile.print(",");
  // dataFile.print(pressure / 4096.0);
  // dataFile.print(",");
  // dataFile.print(temperature / 100.0);
  // dataFile.println();

  // if(millis() - millisSinceLastLog > 2000){
  //   dataFile.flush();
  //   millisSinceLastLog = millis();
  // }

  // if(millis() - millisSinceLastBeep > 10200){
  //   digitalWrite(buzzer, LOW);
  //   millisSinceLastBeep = millis();
  // }else if (millis() - millisSinceLastBeep > 10000){
  //   digitalWrite(buzzer, HIGH);
  // }
  // // save SD Card data every 5 seconds
  // //digitalWrite(buzzer, LOW);
  // delay(20);
  // //digitalWrite(buzzer, HIGH);

  // We intend to send control frames every 20ms.
  const auto time = millis();
  if (gNextSendMillis >= time)
  {
    return;
  }

  gNextSendMillis += 20;
  gLoopCount++;

  Moteus::PositionMode::Command cmd;
  cmd.position = ((millis() / 1000) % 2) * 25;
  cmd.velocity = 2;
  cmd.accel_limit = 200;
  cmd.velocity_limit = 200;
  moteus1.SetPosition(cmd, &position_fmt);

  // Moteus::CurrentMode::Command cmd;
  // cmd.d_A = 0.5;
  // cmd.q_A = 0.5;
  // moteus1.SetCurrent(cmd);

  // Moteus::PositionMode::Command cmd;
  // cmd.position = NaN;
  // cmd.velocity = 5;
  // cmd.accel_limit = 200;
  // cmd.velocity_limit = 200;
  // moteus1.SetPosition(cmd, &position_fmt);

  if (gLoopCount % 25 != 0)
  {
    return;
  }

  // Only print our status every 5th cycle, so every 1s.

  Serial1.print(F("time "));
  Serial1.print(gNextSendMillis);

  auto print_moteus = [](const Moteus::Query::Result &query)
  {
    Serial1.print(static_cast<int>(query.mode));
    Serial1.print(F(" "));
    Serial1.print(query.position);
    Serial1.print(F("  velocity "));
    Serial1.print(query.velocity);
    Serial1.print(F("  fault "));
    Serial1.print(query.fault);
    Serial1.print(F("  torque "));
    Serial1.print(query.torque);
    Serial1.print(F("  q current "));
    Serial1.print(query.q_current);
    Serial1.print(F("  voltage "));
    Serial1.print(query.voltage);
  };

  print_moteus(moteus1.last_result().values);
  Serial1.println();
}