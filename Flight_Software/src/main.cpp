#include <Arduino.h>
#include <Adxl.h>
#include <Wire.h>
#include <Lps22.h>
#include <SPI.h>
#include <SD.h>
#include <ACAN2517FD.h>
#include <Moteus.h>
#include <MotorRoutines.h>
#include <PinDefinitions.h>
#include <StatusIndicator.h>
#include <Logging.h>
#include <STM32FreeRTOS.h>
#include <States.h>
#include <Igniter.h>
#include <Clutch.h>
//#include <i2c_scanner.h>

const bool SIMULATION_MODE = false;

const float movement_ratio = 1.4089; //Converts from motor frame to weight frame
float motor_frame_offset = -301.90;
float balance_frame_offset = 350;
float motor_travel_length = 446.83; // in motor frame

SemaphoreHandle_t xSerialMutex;

float x, y, z;
//float x_hg, y_hg, z_hg;

float theta;
float theta_dot;

float alpha = 0.5;       // Smoothing factor (adjust based on your needs)
float filteredTheta = 90; // Initialize the filtered value

uint32_t mode;
float position, velocity, current, torque;

float pressure;
float temperature;
float altitude;
float vertical_velocity;

float pressure_offset;

float prev_pressure = 0;
float two_prev_pressure = 0;

float pressure_at_liftoff = 0;
int millis_at_liftoff = 0;

int millis_at_chute_transition = 0;

//largest value at start
float apogee_pressure = 20000;

int last_pressure_check_millis = 0;

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

Logging logging(false, true, PinDefs.SD_CS);
ACAN2517FD can (PinDefs.MCP_CS, SPI, PinDefs.MCP_INT);
File dataFile;
Adxl adxl345 = Adxl(0x1D, ADXL345);
//Adxl adxl375 = Adxl(0x53, ADXL375);
Lps22 lps22 = Lps22(0x5C);
Clutch clutch = Clutch(PinDefs.SERVO, 120, 50);
Igniter primaryIgniter = Igniter(PinDefs.IGNITER_0, PinDefs.IGNITER_SENSE_0);
Igniter backupIgniter = Igniter(PinDefs.IGNITER_1, PinDefs.IGNITER_SENSE_1);
StatusIndicator statusIndicator = StatusIndicator(PinDefs.STATUS_LED_RED, PinDefs.STATUS_LED_GREEN, PinDefs.STATUS_LED_BLUE);
States state = States::IDLE;

Moteus moteus1(can, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());


const float K_P = 10;
const float K_D = 0.1;

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

float calculateAngle(float ax, float ay, float az)
{
  // float magnitude = sqrt(ax * ax + ay * ay + az * az);
  // float angle = atan2(az, ay);
  // return angle * (180.0 / 3.14);          // Convert to degrees

  // Calculate the total magnitude of the acceleration vector
  float magnitude = sqrt(x * x + y * y + z * z);

  // Calculate the tilt angle in radians
  float tiltAngle = acos(y / magnitude);

  // Convert radians to degrees
  tiltAngle = tiltAngle * 180.0 / PI;
  return tiltAngle;
}

float weight_frame_to_motor_frame(float pos){
  return pos / movement_ratio + motor_frame_offset;
}
float motor_frame_to_weight_frame(float pos){
  return (pos - motor_frame_offset) * movement_ratio;
}
float balance_frame_to_motor_frame(float pos){
  float pos_offset = pos + balance_frame_offset;
  return weight_frame_to_motor_frame(pos_offset);
}

float calculateAltitude(float pressure) //1Pa = 
{
  return (pressure - pressure_offset) / 12.01316023475;
}

// Fast Loop (Read sensors, Write motor)
void FastLoop(void *pvParameters)
{
  while (true)
  {
    if (SIMULATION_MODE)
    {
      // Request data from serial

      if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
      {

        Serial1.println("REQ_DATA");
        while (Serial1.available() == 0)
        {
          // statusIndicator.solid(StatusIndicator::RED);
        }
        // statusIndicator.solid(StatusIndicator::GREEN);
        // Serial1.println("Data received");
        String data = Serial1.readStringUntil('\n');
        // Serial1.println("Read data");
        String dataParts[12];
        splitString(data, ',', dataParts, 12);

        x = dataParts[0].toFloat();
        // x_hg = dataParts[1].toFloat();
        y = dataParts[2].toFloat();
        // y_hg = dataParts[3].toFloat();
        z = dataParts[4].toFloat();
        // z_hg = dataParts[5].toFloat();
        pressure = dataParts[6].toFloat();
        temperature = dataParts[7].toFloat();
        mode = dataParts[8].toInt();
        position = dataParts[9].toFloat();
        velocity = dataParts[10].toFloat();
        current = dataParts[11].toFloat();

        xSemaphoreGive(xSerialMutex);
      }
    }
    else
    {
      adxl345.readAccelerometer(&x, &y, &z);
      //adxl375.readAccelerometer(&x_hg, &y_hg, &z_hg);

      lps22.readPressure(&pressure);
      lps22.readTemperature(&temperature);

      // float new_altitude = calculateAltitude(pressure, temperature);
      // vertical_velocity = (new_altitude - altitude) / 0.02;
      // altitude = new_altitude;

      Moteus::Result lastResult = moteus1.last_result();
      mode = static_cast<uint32_t>(lastResult.values.mode); // TODO: how to get value associated with enum?
      velocity = lastResult.values.velocity;
      position = lastResult.values.position;
      current = lastResult.values.q_current;
      torque = lastResult.values.torque;

      theta = calculateAngle(x, y, z);

      if(mode == 11){
        moteus1.SetStop();
      }

   }

    vTaskDelay(20); // Run at 50hz
  }
}

// 20 Hz Logging Task
void LogLoop(void *pvParameters)
{
  while (true)
  {
    switch (state)
    {
      case States::INTEGRATING:
      {
        statusIndicator.solid(StatusIndicator::ORANGE);
        clutch.disengage();
        if (digitalRead(PinDefs.ARM))
        {
          state = States::IDLE;
        }
      }

      case States::IDLE:
      {
        statusIndicator.solid(StatusIndicator::BLUE);
        if(!digitalRead(PinDefs.ARM)){
          state = States::ARMED;
          prev_pressure = pressure;
          two_prev_pressure = pressure;
        }
        break;
      }
      
      case States::ARMED:
      {
        statusIndicator.solid(StatusIndicator::GREEN);
        clutch.begin();
        clutch.engage();

        //check every 10 seconds:
        if(millis() - last_pressure_check_millis > 10000){
          last_pressure_check_millis = millis();
          two_prev_pressure = prev_pressure;
          prev_pressure = pressure;
        }

        float pressure_change = two_prev_pressure - pressure;


        if(digitalRead(PinDefs.ARM)){
          state = States::IDLE;
        }

        if (y > 6 || (pressure_change > 1 && y > 2) || pressure_change > 2)
        {
          state = States::FLIGHT;
          pressure_at_liftoff = two_prev_pressure;
          millis_at_liftoff = millis();
        }

        break;
        
      }
      case States::FLIGHT:
      {
        statusIndicator.solid(StatusIndicator::RED);

        if(pressure < apogee_pressure){
          apogee_pressure = pressure;
        }


        if(millis() - millis_at_liftoff > 15000 || pressure - apogee_pressure > 1 && millis() - millis_at_liftoff > 10000){
          state = States::APOGEE;
        }
        break;
      }

      case States::APOGEE:
      {
        clutch.disengage();
        MotorRoutines::moveToPosition(moteus1, balance_frame_to_motor_frame(0), 600, 0.1, 400, motor_frame_offset + 10, motor_frame_offset + motor_travel_length - 10); // For light weight was 500, 0.2, 800,
        
        if(mode == 10){
          state = States::BELLYFLOP;
        }else{
          state = States::CHUTE;
        }

        break;
      }

      case States::BELLYFLOP:
      {
        float new_theta = calculateAngle(x, y, z);
        theta_dot = (new_theta - theta) / 0.02;
        theta = new_theta;

        filteredTheta = alpha * theta + (1 - alpha) * filteredTheta;

        float theta_error = -filteredTheta + 90;

        float weight_position = theta_error * K_P + theta_dot * K_D; // TODO: Add scaling for acceleration

        MotorRoutines::moveToPosition(moteus1, balance_frame_to_motor_frame(weight_position), 600, 0.1, 400, motor_frame_offset + 10, motor_frame_offset + motor_travel_length - 10); // For light weight was 500, 0.2, 800,

        if (pressure_at_liftoff - pressure < 20){
          state = States::CHUTE;
          millis_at_chute_transition = millis();
        }

        break;
      }

      case States::CHUTE:
      {
        if(millis() - millis_at_chute_transition < 1000){
          digitalWrite(PinDefs.IGNITER_0, HIGH);
        }
        else if (millis() - millis_at_chute_transition < 2000){
          digitalWrite(PinDefs.IGNITER_0, LOW);
          digitalWrite(PinDefs.IGNITER_1, HIGH);
        }
        digitalWrite(PinDefs.IGNITER_1, LOW);
        break;
      }
    vTaskDelay(50); // 50 ms = 20 Hz
  }
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
    if(xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE){

        Serial1.print(millis());
        Serial1.print("\t");
        Serial1.print(stateToString(state));
        Serial1.print("\t");
        Serial1.print(x, 3);
        // Serial1.print("\t");
        // Serial1.print(x_hg, 3);
        Serial1.print("\t");
        Serial1.print(y, 3);
        // Serial1.print("\t");
        // Serial1.print(y_hg, 3);
        Serial1.print("\t");
        Serial1.print(z, 3);
        // Serial1.print("\t");
        // Serial1.print(z_hg, 3);
        Serial1.print("\t");
        Serial1.print(theta, 3);
        Serial1.print("\t");
        Serial1.print(pressure);
        Serial1.print("\t");
        Serial1.print(temperature);
        Serial1.print("\t");
        Serial1.print(mode);
        Serial1.print("\t");
        Serial1.print(motor_frame_to_weight_frame(position));
        Serial1.print("\t");
        Serial1.print(velocity);
        Serial1.print("\t");
        Serial1.print(current);
        Serial1.print("\t");
        Serial1.print(torque, 3);
        Serial1.print("\t");
        Serial1.print(altitude);

        // Serial1.print("\t");
        // Serial1.print(xPortGetFreeHeapSize());
        // Serial1.print("\t");
        // Serial1.print(uxTaskGetStackHighWaterMark(FastLoopHandle));
        // Serial1.print("\t");
        // Serial1.print(uxTaskGetStackHighWaterMark(LogLoopHandle));
        // Serial1.print("\t");
        // Serial1.print(uxTaskGetStackHighWaterMark(PrintLoopHandle));
        // Serial1.print("\t");
        // Serial1.print(uxTaskGetStackHighWaterMark(FlushLoopHandle));
        Serial1.println();


        //convert to string to log message
        const String logMessage = String(millis()) + "," + String(x) + "," + String(y) + "," + String(z) + "," + String(pressure) + "," + String(temperature) + "," + String(mode) + "," + String(motor_frame_to_weight_frame(position)) + "," + String(velocity) + "," + String(current) + "," + String(torque) + "," + String(altitude);
        logging.log(logMessage.c_str());



      xSemaphoreGive(xSerialMutex);
    }
    vTaskDelay(250); // 200 ms = 5 Hz
    }
}

// 0.5 Hz SD Card Flush Task
void FlushLoop(void *pvParameters)
{
  while (true)
  {
    logging.flush();
    if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
    {
      Serial1.println("ms\tmode\tx\ty\tz\ttheta\tp\tt\tmode\tpos\tvel\tI\tT\talt");
      xSemaphoreGive(xSerialMutex);
    }
      vTaskDelay(4000); // 2000 ms = 0.5 Hz
    }
}

void setup() {
  Serial1.begin(230400);

  pinMode(PinDefs.ARM, INPUT_PULLUP);
  pinMode(PinDefs.IGNITER_0, OUTPUT);

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
    snprintf(buffer, sizeof(buffer), "CAN err: %ld", errorCode); //TODO: Doesn't work
    logging.log(buffer);
    delay(1000);
  }      

  // To clear any faults the controllers may have, we start by sending
  // a stop command to each.
  moteus1.SetStop();


  while (!adxl345.begin()) {
    logging.log("Err A345");
    delay(1000);
  }

  // while (!adxl375.begin()) {
  //   logging.log("ErrA375");
  //   delay(1000);
  // }

  while (!lps22.begin()) {
    logging.log("ErrLPS22");
    delay(1000);
  }

  logging.flush();

  //uncomment to measure motor travel distance after power cycle
  // motor_frame_offset = MotorRoutines::runToEnd(moteus1, -20, 1);
  // Serial1.println("Motor frame offset: " + String(motor_frame_offset));

  // for (int i = 0; i < 400; i+=50)
  // {
  //   float motor_frame = weight_frame_to_motor_frame(i);
  //   MotorRoutines::moveToPositionBlocking(moteus1, motor_frame, 50, 3);
  //   delay(5000);
  // }

  // MotorRoutines::moveToPositionBlocking(moteus1, weight_frame_to_motor_frame(0), 50, 3);
  // MotorRoutines::moveToPositionBlocking(moteus1, balance_frame_to_motor_frame(0), 50, 3);

  // delay(5000);

  // MotorRoutines::moveToPositionBlocking(moteus1, balance_frame_to_motor_frame(50), 50, 3);

  // delay(5000);

  // MotorRoutines::moveToPositionBlocking(moteus1, balance_frame_to_motor_frame(-50), 50, 3);

  // //Calibrate motor distance
  // MotorRoutines::runToEnd(moteus1, 5, 0.5);
  // float motor_travel_distance = MotorRoutines::measureTravelDistance(moteus1, 50, 3);
  // Serial1.println("Motor travel distance: " + String(motor_travel_distance));

  // Moteus::Result lastResult = moteus1.last_result();
  // // mode = lastResult.values.mode; //TODO: how to get value associated with enum?
  // position = lastResult.values.position;

  // // run motor to center
  // float center_position = motor_offset + motor_travel_distance / 2;
  // MotorRoutines::moveToPositionBlocking(moteus1, center_position, 50, 15);

  // run motor to 0
  // MotorRoutines::moveToPositionBlocking(moteus1, 0, 50, 15);
  // delay(5000);
  // MotorRoutines::moveToPositionBlocking(moteus1, 100, 50, 15);
  // delay(5000);

  // Serial1.println("Motor location:");
  // Serial1.println(moteus1.last_result().values.position);

  clutch.disengage();

  // Serial1.println("Motor offset: " + String(motor_offset));
  // Serial1.println("Motor travel distance: " + String(motor_travel_distance));

  Serial1.println("Motor routine complete");

  // Create tasks
  xSerialMutex = xSemaphoreCreateMutex();
  Serial1.print("Free heap before tasks: ");
  Serial1.println(xPortGetFreeHeapSize());

  if (xTaskCreate(FastLoop, "FastLoop", 256, NULL, 3, &FastLoopHandle) != pdPASS)
  {
    Serial1.println("ErrX");
    while (1);
  }
  if (xTaskCreate(LogLoop, "LogLoop", 128, NULL, 2, &LogLoopHandle) != pdPASS)
  {
    Serial1.println("ErrX");
    while (1);
  }
  if (xTaskCreate(PrintLoop, "PrintLoop", 128, NULL, 1, &PrintLoopHandle) != pdPASS)
  {
    Serial1.println("ErrX");
    while (1);
  }
  if (xTaskCreate(FlushLoop, "FlushLoop", 64, NULL, 1, &FlushLoopHandle) != pdPASS)
  {
    Serial1.println("ErrX");
    while (1);
  }

  statusIndicator.solid(StatusIndicator::GREEN);

  Serial1.println("Setup complete");

  // Start FreeRTOS scheduler
  vTaskStartScheduler();
}

void loop() {

  
}