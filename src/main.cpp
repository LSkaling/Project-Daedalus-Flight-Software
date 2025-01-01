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
#include <STM32FreeRTOS.h>

long last_command = 0;

int millisSinceLastLog = 0;
int millisSinceLastBeep = 0;

static uint32_t gNextSendMillis = 0;

// ADXL345 scaling factor
const float scale345 = 0.0039; // ±16g at 4 mg/LSB

// ADXL375 scaling factor
const float scale375 = 0.049; // ±200g at ~49 mg/LSB

float x, y, z, x_hg, y_hg, z_hg;

int mode;
float position, velocity, current;

int32_t pressure;
int16_t temperature;

const char* logVariables[] = {
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

const char* printVariables[] = {
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

TaskHandle_t FastLoopHandle;
TaskHandle_t LogLoopHandle;
TaskHandle_t PrintLoopHandle;
TaskHandle_t FlushLoopHandle;

// Fast Loop (Read sensors, Write motor)
void FastLoop(void *pvParameters)
{
  while (true)
  {
    int16_t x_raw, y_raw, z_raw;
    adxl345.readAccelerometer(&x_raw, &y_raw, &z_raw);

    int16_t x_raw_hg, y_raw_hg, z_raw_hg;
    adxl375.readAccelerometer(&x_raw_hg, &y_raw_hg, &z_raw_hg);

    lps22.readPressure(&pressure);
    pressure /= 4096;

    lps22.readTemperature(&temperature);
    temperature /= 100;

    // Convert raw values to 'g' units (assuming 4 mg/LSB at +/-16g)
    x = x_raw * scale345;
    y = y_raw * scale345;
    z = z_raw * scale345;

    x_hg = x_raw_hg * scale375;
    y_hg = y_raw_hg * scale375;
    z_hg = z_raw_hg * scale375;

    vTaskDelay(1); // Run as fast as possible (1 ms delay to prevent watchdog reset)
  }
}

// 50 Hz Logging Task
void LogLoop(void *pvParameters)
{
  while (true)
  {
    Moteus::PositionMode::Command cmd;
    cmd.position = NaN;
    cmd.velocity = 5;

    moteus1.SetPosition(cmd);

    const auto moteus1_result = moteus1.last_result();
    mode = static_cast<int>(moteus1_result.values.mode);
    position = moteus1_result.values.position;
    velocity = moteus1_result.values.velocity;
    current = moteus1_result.values.q_current;
    vTaskDelay(50); // 50 ms = 20 Hz
  }
}

// 5 Hz Print and State Update Task
void PrintLoop(void *pvParameters)
{
  while (true)
  {

    // float values[] = {
    //     millis(),
    //     x,
    //     x_hg,
    //     y,
    //     y_hg,
    //     z,
    //     z_hg,
    //     pressure,
    //     temperature,
    //     mode,
    //     position,
    //     velocity
    // };

    //logging.log(printVariables, values, logSize);
    Serial1.print("Time: " + String(millis()));
    Serial1.print(" Xg: " + String(x));
    Serial1.print(" Xg2: " + String(x_hg));
    Serial1.print(" Yg: " + String(y));
    Serial1.print(" Yg2: " + String(y_hg));
    Serial1.print(" Zg: " + String(z));
    Serial1.print(" Zg2: " + String(z_hg));
    Serial1.print(" Pressure: " + String(pressure));
    Serial1.print(" Temperature: " + String(temperature));
    Serial1.print(" Mode: " + String(mode));
    Serial1.print(" Position: " + String(position));
    Serial1.print(" Velocity: " + String(velocity)); 
    Serial1.print(" Current: " + String(current));
    Serial1.println();
    vTaskDelay(200); // 200 ms = 5 Hz
  }
}

// 0.5 Hz SD Card Flush Task
void FlushLoop(void *pvParameters)
{
  while (true)
  {
    logging.flush();
    vTaskDelay(2000); // 2000 ms = 0.5 Hz
  }
}

void setup() {
  Serial1.begin(115200);
  while (!Serial1){
    statusIndicator.solid(StatusIndicator::RED);
  }
  delay(4000);
  Serial1.println("Starting up...");

  statusIndicator.off();

  Wire.setSDA(PinDefs.SDA);
  Wire.setSCL(PinDefs.SCL);
  Wire.begin();  

  SPI.setMOSI(PinDefs.SDI);
  SPI.setMISO(PinDefs.SDO);
  SPI.setSCLK(PinDefs.SCK); 
  SPI.begin();  

  while (logging.begin(logVariables, logSize) == false) {
    statusIndicator.solid(StatusIndicator::RED);
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
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "CAN initialization failed with error code: %ld", errorCode);
    logging.log(buffer);
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

  // Create tasks
  xTaskCreate(FastLoop, "FastLoop", 256, NULL, 3, &FastLoopHandle); // Highest priority
  xTaskCreate(LogLoop, "LogLoop", 256, NULL, 2, &LogLoopHandle);
  xTaskCreate(PrintLoop, "PrintLoop", 256, NULL, 1, &PrintLoopHandle);
  xTaskCreate(FlushLoop, "FlushLoop", 256, NULL, 1, &FlushLoopHandle);

  statusIndicator.solid(StatusIndicator::GREEN);

  Serial1.println("Setup complete");

  // Start FreeRTOS scheduler
  vTaskStartScheduler();
}

uint16_t gLoopCount = 0;

void loop() {

  
}