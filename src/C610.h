#pragma once

#include <Arduino.h>
#include <CAN.h>  // Use the Maple core's CAN library

struct C610Feedback {
  int32_t counts, rpm, current;
};

class C610 {
 private:
  bool _initialized_mechanical_angle;
  int32_t _rotations;
  int32_t _last_pos_measurement;
  int32_t _counts;
  int32_t _rpm;
  int32_t _current;

  static const int32_t kCountsPerRev = 8192;
  static constexpr float kReduction = 36.0F;
  static constexpr float kCountsPerRad = kCountsPerRev * kReduction / (2 * M_PI);
  static constexpr float kRPMPerRadS = kReduction * 60 / (2.0F * M_PI);
  static constexpr float kMilliAmpPerAmp = 1000.0F;

  static constexpr float kResistance = 0.100;
  static constexpr float kVoltageConstant = 100.0;

 public:
  static C610Feedback InterpretMessage(const CAN_message_t &msg);

  C610();
  void UpdateState(C610Feedback feedback);
  float Position();  // output [rad]
  float Velocity();  // output [rad/s]
  float Current();   // [mA]
  float Torque();    // output [Nm]
  float ElectricalPower();
  float MechanicalPower();
};
