#include <Arduino.h>
#include <Adxl.h>
#include <Wire.h>
#include <Lps22.h>
#include <SPI.h>
#include <SD.h>
#include <PinDefinitions.h>
#include <StatusIndicator.h>
#include <Logging.h>
#include <STM32FreeRTOS.h>
#include <States.h>
#include <Igniter.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include <i2c_scanner.h>

//#define configCHECK_FOR_STACK_OVERFLOW 2 testing

const bool SIMULATION_MODE = false;

SemaphoreHandle_t xSerialMutex;
SemaphoreHandle_t xMoteusMutex;

float x, y, z;
//float x_hg, y_hg, z_hg;

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

uint32_t mode;
float position, velocity, current, torque;

float pressure;
float temperature;
float altitude;
float vertical_velocity;

bool primaryIgniterConnected = false;
bool backupIgniterConnected = false;

float pressure_offset;

float prev_pressure = 0;
float two_prev_pressure = 0;

float pressure_at_liftoff = 0;
int millis_at_liftoff = 0;

bool alternateMotorCommand = true;

int millis_at_chute_transition = 0;


int last_sd_write = 0;


// const char* logVariables[] = {
//     "Time",
//     "Xg",
//     "Xg2",
//     "Yg",
//     "Yg2",
//     "Zg",
//     "Zg2",
//     "Pressure",
//     "Temperature",
//     "Mode",
//     "Position",
//     "Velocity"
// };

//size_t logSize = sizeof(logVariables) / sizeof(logVariables[0]);

Logging logging(false, true, PinDefs.SD_CS);
File dataFile;
Adxl adxl345 = Adxl(0x1D, ADXL345);
//Adxl adxl375 = Adxl(0x53, ADXL375);
Lps22 lps22 = Lps22(0x5C);
Igniter primaryIgniter = Igniter(PinDefs.IGNITER_0, PinDefs.IGNITER_SENSE_0);
Igniter backupIgniter = Igniter(PinDefs.IGNITER_1, PinDefs.IGNITER_SENSE_1);
StatusIndicator statusIndicator = StatusIndicator(PinDefs.STATUS_LED_RED, PinDefs.STATUS_LED_GREEN, PinDefs.STATUS_LED_BLUE);
States state = States::INTEGRATING;

// void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
// {
//   Serial1.print("Stack overflow in task: ");
//   Serial1.println(pcTaskName);
//   while (1)
//     ; // Halt system
// }

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

void printEvent(sensors_event_t *event)
{
  double x = -1000000, y = -1000000, z = -1000000; // dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER)
  {
    Serial1.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION)
  {
    Serial1.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD)
  {
    Serial1.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE)
  {
    Serial1.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR)
  {
    Serial1.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION)
  {
    Serial1.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY)
  {
    Serial1.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else
  {
    Serial1.print("Unk:");
  }

  Serial1.print("\tx= ");
  Serial1.print(x);
  Serial1.print(" |\ty= ");
  Serial1.print(y);
  Serial1.print(" |\tz= ");
  Serial1.println(z);
}

void setup() {
  Serial1.begin(230400);

  pinMode(PinDefs.ARM, INPUT_PULLUP);
  pinMode(PinDefs.IGNITER_0, OUTPUT);
  pinMode(PinDefs.IGNITER_1, OUTPUT);

  while (!Serial1){
    statusIndicator.solid(StatusIndicator::RED);
  }
  statusIndicator.solid(StatusIndicator::RED);

  pinMode(PinDefs.IGNITER_1, OUTPUT);
  pinMode(PinDefs.IGNITER_0, OUTPUT);

  Wire.setSDA(PinDefs.SDA);
  Wire.setSCL(PinDefs.SCL);
  Wire.begin();

  delay(4000);

  //scanI2CDevices();

  SPI.setMOSI(PinDefs.SDI);
  SPI.setMISO(PinDefs.SDO);
  SPI.setSCLK(PinDefs.SCK); 
  SPI.begin();  

  while (logging.begin() == false) {
    statusIndicator.solid(StatusIndicator::RED);
    delay(1000);
  }


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

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial1.print("ERRBNO");
    while(true);
  }

  const String logHeading = "Time\tXg\tYg\tZg\tPressure\tTemperature\tOrientX\tOrientY\tOrientZ\tGyroX\tGyroY\tGyroZ\tLinearX\tLinearY\tLinearZ\tMagX\tMagY\tMagZ\tAccelX\tAccelY\tAccelZ\tGravityX\tGravityY\tGravityZ\tBoardTemp\tSysCal\tGyroCal\tAccelCal\tMagCal";
  logging.log(logHeading.c_str());

  logging.flush();

  statusIndicator.solid(StatusIndicator::GREEN);

  Serial1.println("Setup complete");

}

void loop() {
  adxl345.readAccelerometer(&x, &y, &z);
  // adxl375.readAccelerometer(&x_hg, &y_hg, &z_hg);

  lps22.readPressure(&pressure);
  lps22.readTemperature(&temperature);

  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();
  Serial1.println();
  Serial1.print(F("temperature: "));
  Serial1.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial1.println();
  Serial1.print("Calibration: Sys=");
  Serial1.print(system);
  Serial1.print(" Gyro=");
  Serial1.print(gyro);
  Serial1.print(" Accel=");
  Serial1.print(accel);
  Serial1.print(" Mag=");
  Serial1.println(mag);

  Serial1.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);

  const String logMessage = String(millis()) + "\t" +
                            String(x) + "\t" +
                            String(y) + "\t" +
                            String(z) + "\t" +
                            String(pressure) + "\t" +
                            String(temperature) + "\t" +
                            String(orientationData.orientation.x) + "\t" +
                            String(orientationData.orientation.y) + "\t" +
                            String(orientationData.orientation.z) + "\t" +
                            String(angVelocityData.gyro.x) + "\t" +
                            String(angVelocityData.gyro.y) + "\t" +
                            String(angVelocityData.gyro.z) + "\t" +
                            String(linearAccelData.acceleration.x) + "\t" +
                            String(linearAccelData.acceleration.y) + "\t" +
                            String(linearAccelData.acceleration.z) + "\t" +
                            String(magnetometerData.magnetic.x) + "\t" +
                            String(magnetometerData.magnetic.y) + "\t" +
                            String(magnetometerData.magnetic.z) + "\t" +
                            String(accelerometerData.acceleration.x) + "\t" +
                            String(accelerometerData.acceleration.y) + "\t" +
                            String(accelerometerData.acceleration.z) + "\t" +
                            String(gravityData.acceleration.x) + "\t" +
                            String(gravityData.acceleration.y) + "\t" +
                            String(gravityData.acceleration.z) + "\t" +
                            String(boardTemp) + "\t" +
                            String(system) + "\t" +
                            String(gyro) + "\t" +
                            String(accel) + "\t" +
                            String(mag);
  logging.log(logMessage.c_str());

  
  if (millis() - last_sd_write > 1000) {
    logging.flush();
    last_sd_write = millis();
    Serial1.println("Flushed");
  }

  delay(10);
}
