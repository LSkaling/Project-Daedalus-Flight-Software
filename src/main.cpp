#include <Arduino.h>
#include <Adxl.h>
#include <Wire.h>
#include <Lps22.h>
#include <SPI.h>
#include <SD.h>
#include "C610Bus.h"
#include "C610Bus.tpp" 


#define SD_CS_PIN PA4  // Chip Select pin for SD card
#define TEENSY_I2C_ADDRESS 8

const bool debug = false; //Changes if Serial is used

// PID parameters
float kp = 200.0;  // Proportional gain
float ki = 0.0;   // Integral gain
float kd = 20000.0;   // Derivative gain

const int motor_max = 0;
const int motor_min = -60;

float target_position = 1.0;  // Initial target position in radians
float previous_error = 0;
float integral = 0;
long last_time = 0;

long last_command = 0;

C610Bus bus;



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
int millisSinceLastBeep = 0;

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

// Function to send position mode command
void setPositionMode(float position, float velocity_limit, float accel_limit) {
    uint8_t buffer[13]; // Command ID + 3 floats (4 bytes each)

    buffer[0] = 0x02; // Command ID for Set Position Mode
    memcpy(&buffer[1], &position, sizeof(float));
    memcpy(&buffer[5], &velocity_limit, sizeof(float));
    memcpy(&buffer[9], &accel_limit, sizeof(float));

    Wire.beginTransmission(TEENSY_I2C_ADDRESS);
    Wire.write(buffer, sizeof(buffer));
    Wire.endTransmission();
}

// Function to send velocity mode command
void setVelocityMode(float velocity, float accel_limit, float velocity_limit) {
    uint8_t buffer[13]; // Command ID + 3 floats (4 bytes each)

    buffer[0] = 0x03; // Command ID for Set Velocity Mode
    memcpy(&buffer[1], &velocity, sizeof(float));
    memcpy(&buffer[5], &accel_limit, sizeof(float));
    memcpy(&buffer[9], &velocity_limit, sizeof(float));

    Wire.beginTransmission(TEENSY_I2C_ADDRESS);
    Wire.write(buffer, sizeof(buffer));
    Wire.endTransmission();
}

// Function to read motor state from Teensy
MotorState getMotorState() {
    MotorState state;

    // Request 21 bytes from the Teensy
    Wire.requestFrom(TEENSY_I2C_ADDRESS, 21);

    if (Wire.available() >= 21) {
        Wire.readBytes((char*)&state.position, sizeof(float));
        Wire.readBytes((char*)&state.velocity, sizeof(float));
        state.fault = Wire.read();
        Wire.readBytes((char*)&state.torque, sizeof(float));
        Wire.readBytes((char*)&state.q_current, sizeof(float));
        Wire.readBytes((char*)&state.voltage, sizeof(float));
    }

    return state;
}

void setup() {
  // Initialize Serial1 communication
  Serial1.begin(9600);
  // Initialize I2C communication on PB8 (SCL) and PB9 (SDA)

  CAN.begin(1000000);  // Initialize CAN bus with a 1 Mbps bitrate


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
  while (!Serial1 && debug);
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

  if(debug){

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
    Serial1.println();
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

  if(millis() - millisSinceLastBeep > 10200){
    digitalWrite(buzzer, LOW);
    millisSinceLastBeep = millis();
  }else if (millis() - millisSinceLastBeep > 10000){
    digitalWrite(buzzer, HIGH);
  }
  // save SD Card data every 5 seconds
  //digitalWrite(buzzer, LOW);
  delay(20);
  //digitalWrite(buzzer, HIGH);

  bus.PollCAN(); // Check for messages from the motors.

  // Check for serial input to update target position
  if (Serial.available() > 0)
  {
      String input = Serial.readStringUntil('\n');  // Read the input from Serial
      target_position = input.toFloat();            // Convert input to float (radians)

      // Convert target position to range
      if (target_position > motor_max)
      {
          target_position = motor_max;
      }
      else if (target_position < motor_min)
      {
          target_position = motor_min;
      }

      Serial.print("New target position: ");
      Serial.println(target_position);
  }

  long now = millis();
  if (now - last_command >= 10) // Loop at 100Hz. Adjust control every 10ms
  {
      // Get the current position of motor 0
      float current_position = bus.Get(0).Position(); // Get the shaft position of motor 0 in radians.
      
      // Calculate the error
      float error = target_position - current_position;

      // Calculate the time difference
      long dt = now - last_time;
      last_time = now;

      // PID calculations
      integral += error * dt; // Integral term
      float derivative = (error - previous_error) / dt; // Derivative term

      // Compute the output torque based on PID control
      float output_torque = (kp * error) + (ki * integral) + (kd * derivative);

      // Limit the output torque to less than 10000
      if (output_torque > 10000)
      {
          output_torque = 10000;
      }
      else if (output_torque < -10000)
      {
          output_torque = -10000;
      }


      // Command torque to motor 0 (ensure the torque is within safe limits)
      bus.CommandTorques(output_torque, 0, 0, 0, C610Subbus::kOneToFourBlinks); 

      // Update the previous error
      previous_error = error;

      // Print debugging info (position, error, output torque)
      Serial.print("Position: ");
      Serial.print(current_position);
      Serial.print(" Error: ");
      Serial.print(error);
      Serial.print(" Output Torque: ");
      Serial.println(output_torque);

      last_command = now;
  }  
}