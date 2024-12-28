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
#include <StatusIndicator.h>
#include <Logging.h>

long last_command = 0;

int millisSinceLastLog = 0;
int millisSinceLastBeep = 0;

static uint32_t gNextSendMillis = 0;

String logVariables[] = {
    "Time",
    "Xg",
    "Xg2",
    "Yg",
    "Yg2",
    "Zg",
    "Zg2",
    "Pressure",
    "Temperature",
    "Mode",
    "Position",
    "Velocity"
};

String printVariables[] = {
    "Time",
    "Xg",
    "Xg2",
    "Yg",
    "Yg2",
    "Zg",
    "Zg2",
    "P",
    "Temp",
    "Mode",
    "Pos",
    "Vel"
};

size_t logSize = sizeof(logVariables) / sizeof(logVariables[0]);

Logging logging(true, true, PinDefs.SD_CS);
ACAN2517FD can (PinDefs.MCP_CS, SPI, PinDefs.MCP_INT);
File dataFile;
Adxl adxl345 = Adxl(0x1D, ADXL345);
Adxl adxl375 = Adxl(0x53, ADXL375);
Lps22 lps22 = Lps22(0x5C);
StatusIndicator statusIndicator = StatusIndicator(PinDefs.STATUS_LED_RED, PinDefs.STATUS_LED_GREEN, PinDefs.STATUS_LED_BLUE);

Moteus moteus1(can, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());

Moteus::PositionMode::Command position_cmd;
Moteus::PositionMode::Format position_fmt;


void setup() {
  statusIndicator.off();

  Wire.setSDA(PinDefs.SDA);
  Wire.setSCL(PinDefs.SCL);
  Wire.begin();  

  SPI.setMOSI(PinDefs.SDI);
  SPI.setMISO(PinDefs.SDO);
  SPI.setSCLK(PinDefs.SCK); 
  SPI.begin();  

  while (logging.begin(logVariables, logSize) == false) {
    delay(1000);
  }

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
    logging.log("CAN initialization failed with error code: " + String(errorCode));
    delay(1000);
  }      

  // To clear any faults the controllers may have, we start by sending
  // a stop command to each.
  moteus1.SetStop();

  position_fmt.velocity_limit = Moteus::kFloat;
  position_fmt.accel_limit = Moteus::kFloat;

  while (!adxl345.begin()) {
    logging.log("Error connecting to ADXL345 sensor");
    delay(1000);
  }

  while (!adxl375.begin()) {
    logging.log("Error connecting to ADXL375 sensor");
    delay(1000);
  }

  while (!lps22.begin()) {
    logging.log("Error connecting to LPS22 sensor");
    delay(1000);
  }

  logging.flush();

  //statusIndicator.solid(StatusIndicator::Color::GREEN);


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
  pressure /= 4096;

  int16_t temperature;
  lps22.readTemperature(&temperature);
  temperature /= 100;

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

  const auto moteus1_result = moteus1.last_result();
  int mode = static_cast<int>(moteus1_result.values.mode);
  float position = moteus1_result.values.position;
  float velocity = moteus1_result.values.velocity;

  String log_values[] = {
      String(time),
      String(xg, 2),
      String(xg2, 2),
      String(yg, 2),
      String(yg2, 2),
      String(zg, 2),
      String(zg2, 2),
      String(pressure),
      String(temperature),
      String(mode),        // Convert int to String
      String(position, 2), // Convert float to String (2 decimal places)
      String(velocity, 2)  // Convert float to String (2 decimal places)
  };

  logging.log(log_values, printVariables, logSize);

  if(millis() - millisSinceLastLog > 2000){
    logging.flush();
    millisSinceLastLog = millis();
  }

  
}