#include <Arduino.h>
#include <Adxl.h>
#include <Wire.h>
#include <Lps22.h>
#include <SPI.h>
#include <SD.h>
#include <ACAN2517FD.h>
#include <Moteus.h>
#include <MotorRoutines.h>
#include <Buzzer.h>
#include <PinDefinitions.h>
#include <StatusIndicator.h>
#include <Logging.h>
#include <STM32FreeRTOS.h>
#include <States.cpp>
#include <i2c_scanner.h>

const bool SIMULATION_MODE = true;

//SemaphoreHandle_t xSerialMutex;

// ADXL345 scaling factor
const float scale345 = 0.0039; // ±16g at 4 mg/LSB

// ADXL375 scaling factor
const float scale375 = 0.049; // ±200g at ~49 mg/LSB

float x, y, z, x_hg, y_hg, z_hg;

uint32_t mode;
float position, velocity, current;

float pressure;
float temperature;

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

size_t logSize = sizeof(logVariables) / sizeof(logVariables[0]);

Logging logging(true, false, PinDefs.SD_CS);
ACAN2517FD can (PinDefs.MCP_CS, SPI, PinDefs.MCP_INT);
File dataFile;
Adxl adxl345 = Adxl(0x1D, ADXL345);
Adxl adxl375 = Adxl(0x53, ADXL375);
Lps22 lps22 = Lps22(0x5C);
StatusIndicator statusIndicator = StatusIndicator(PinDefs.STATUS_LED_RED, PinDefs.STATUS_LED_GREEN, PinDefs.STATUS_LED_BLUE);
States state = States::IDLE;

Moteus moteus1(can, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());

TaskHandle_t FastLoopHandle;
TaskHandle_t LogLoopHandle;
TaskHandle_t PrintLoopHandle;
TaskHandle_t FlushLoopHandle;

void splitString(String data, char delimiter, String parts[], int maxParts)
{
  int i = 0;
  int pos = 0;
  String token;

  while ((pos = data.indexOf(delimiter)) != -1 && i < maxParts - 1)
  {
    token = data.substring(0, pos);
    parts[i++] = token;
    data = data.substring(pos + 1);
  }
  parts[i] = data; // Last part (or whole if no delimiter left)
}

// Fast Loop (Read sensors, Write motor)
void FastLoop(void *pvParameters)
{
  while (true)
  {
    if (SIMULATION_MODE)
    {
      // Request data from serial
      //Serial1.println("Requesting Data");
      Serial1.println("REQ_DATA");
      while (Serial1.available() == 0)
      {
        statusIndicator.solid(StatusIndicator::RED);
      }
      statusIndicator.solid(StatusIndicator::GREEN);
      //Serial1.println("Data received");
      String data = Serial1.readStringUntil('\n');
      //Serial1.println("Read data");
      String dataParts[12];
      splitString(data, ',', dataParts, 12);

      x = dataParts[0].toFloat();
      x_hg = dataParts[1].toFloat();
      y = dataParts[2].toFloat();
      y_hg = dataParts[3].toFloat();
      z = dataParts[4].toFloat();
      z_hg = dataParts[5].toFloat();
      pressure = dataParts[6].toFloat();
      temperature = dataParts[7].toFloat();
      mode = dataParts[8].toInt();
      position = dataParts[9].toFloat();
      velocity = dataParts[10].toFloat();
      current = dataParts[11].toFloat();

    //   //xSemaphoreGive(xSerialMutex);
    }
    else
    {
      // Serial1.println("Test");
      int16_t x_raw, y_raw, z_raw;
      adxl345.readAccelerometer(&x_raw, &y_raw, &z_raw);

      int16_t x_raw_hg, y_raw_hg, z_raw_hg;
      adxl375.readAccelerometer(&x_raw_hg, &y_raw_hg, &z_raw_hg);

      lps22.readPressure(&pressure);

      lps22.readTemperature(&temperature);

      // Convert raw values to 'g' units (assuming 4 mg/LSB at +/-16g)
      x = x_raw * scale345;
      y = y_raw * scale345;
      z = z_raw * scale345;

      x_hg = x_raw_hg * scale375;
      y_hg = y_raw_hg * scale375;
      z_hg = z_raw_hg * scale375;
   }

    vTaskDelay(200); // Run at 50hz
  }
}

// 20 Hz Logging Task
void LogLoop(void *pvParameters)
{
  while (true)
  {
    switch (state)
    {
    case States::IDLE:
      /* code */
      break;
    
    case States::ARMED:
      /* code */
      break;

    case States::IGNITION:
      /* code */
      break;

    case States::COAST:
      /* code */
      break;

    case States::APOGEE:
      /* code */
      break;

    case States::BELLYFLOP:
      /* code */
      break;

    case States::CHUTE:
      /* code */
      break;

    case States::IMPACT:
      /* code */
      break;

    case States::LANDED:
      /* code */
      break;
    }
    vTaskDelay(50); // 50 ms = 20 Hz
  }
}

// 5 Hz Print and State Update Task
void PrintLoop(void *pvParameters)
{
  while (true)
  {
    if(millis() % 2000 < 1000){
      statusIndicator.solid(StatusIndicator::RED);
    } else{
      statusIndicator.solid(StatusIndicator::BLUE);
    }
    //if(xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE){
      Serial1.print("Time: ");
      Serial1.print(millis());
      Serial1.print(" Xg: ");
      Serial1.print(x);
      Serial1.print(" Xg2: ");
      Serial1.print(x_hg);
      Serial1.print(" Yg: ");
      Serial1.print(y);
      Serial1.print(" Yg2: ");
      Serial1.print(y_hg);
      Serial1.print(" Zg: ");
      Serial1.print(z);
      Serial1.print(" Zg2: ");
      Serial1.print(z_hg);
      Serial1.print(" Pressure: ");
      Serial1.print(pressure);
      Serial1.print(" Temperature: ");
      Serial1.print(temperature);
      Serial1.print(" Mode: ");
      Serial1.print(mode);
      Serial1.print(" Position: ");
      Serial1.print(position);
      Serial1.print(" Velocity: ");
      Serial1.print(velocity);
      Serial1.print(" Current: ");
      Serial1.print(current);
      Serial1.print(" Heap: ");
      Serial1.print(xPortGetFreeHeapSize());
      Serial1.print(" Fast loop:");
      Serial1.print(uxTaskGetStackHighWaterMark(FastLoopHandle));
      Serial1.print(" Log loop:");
      Serial1.print(uxTaskGetStackHighWaterMark(LogLoopHandle));
      Serial1.print(" Print loop:");
      Serial1.print(uxTaskGetStackHighWaterMark(PrintLoopHandle));
      Serial1.print(" Flush loop:");
      Serial1.print(uxTaskGetStackHighWaterMark(FlushLoopHandle));
      Serial1.println();


      //xSemaphoreGive(xSerialMutex);
    //}
    vTaskDelay(500); // 200 ms = 5 Hz
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
  Serial1.begin(230400);

  while (!Serial1){
    statusIndicator.solid(StatusIndicator::RED);
  }

  Wire.setSDA(PinDefs.SDA);
  Wire.setSCL(PinDefs.SCL);
  Wire.begin();

  delay(4000);

  //scanI2CDevices();

  if(SIMULATION_MODE){
    while(true){
      Serial1.println("PING");
      if(Serial1.available() == 0){
        delay(1000);
      }else{
        String response = Serial1.readStringUntil('\n');
        if(response == "TRUE"){
          Serial1.println("Simulation initialized");
          break;
        }
      }
    }
  }

  statusIndicator.off();

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

  const uint32_t errorCode = can.begin(settings, [] { can.isr(); });

  while (errorCode != 0) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "CAN initialization failed with error code: %ld", errorCode); //TODO: Doesn't work
    logging.log(buffer);
    delay(1000);
  }      

  // To clear any faults the controllers may have, we start by sending
  // a stop command to each.
  moteus1.SetStop();

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

  //MotorRoutines::runToEnd(moteus1, 5, 0.5);

  Serial1.println("Motor routine complete");

  // Create tasks
  //xSerialMutex = xSemaphoreCreateMutex();
  Serial1.print("Free heap before tasks: ");
  Serial1.println(xPortGetFreeHeapSize());

  if (xTaskCreate(FastLoop, "FastLoop", 256, NULL, 3, &FastLoopHandle) != pdPASS)
  {
    Serial1.println("FastLoop task creation failed!");
    while (1);
  }
  if (xTaskCreate(LogLoop, "LogLoop", 128, NULL, 2, &LogLoopHandle) != pdPASS)
  {
    Serial1.println("LogLoop task creation failed!");
    while (1);
  }
  if (xTaskCreate(PrintLoop, "PrintLoop", 128, NULL, 1, &PrintLoopHandle) != pdPASS)
  {
    Serial1.println("PrintLoop task creation failed!");
    while (1);
  }
  if (xTaskCreate(FlushLoop, "FlushLoop", 64, NULL, 1, &FlushLoopHandle) != pdPASS)
  {
    Serial1.println("FlushLoop task creation failed!");
    while (1);
  }

  Serial1.print("Free heap after tasks: ");
  Serial1.println(xPortGetFreeHeapSize());

  statusIndicator.solid(StatusIndicator::GREEN);

  Serial1.println("Setup complete");

  // Start FreeRTOS scheduler
  vTaskStartScheduler();
}

void loop() {

  
}