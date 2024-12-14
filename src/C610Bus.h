#pragma once

#include <Arduino.h>
#include <C610.h>
#include <CAN.h>  // Use the Maple core's CAN library

enum class C610Subbus { kOneToFourBlinks, kFiveToEightBlinks };

class C610Bus {
 public:
  static const uint8_t kSize = 8;

 private:
  static const uint32_t kOneToFourBlinksCommandID = 0x200;
  static const uint32_t kFiveToEightBlinksCommandID = 0x1FF;
  static const uint32_t kReceiveBaseID = 0x200;
  static const uint32_t kCountsPerRev = 8192;

  static const uint32_t kSpeedCommandID_OneToFour = 0x202;
  static const uint32_t kSpeedCommandID_FiveToEight = 0x1FE;

  bool is_initialized_ = false;
  C610 controllers_[kSize];

  void InitializeCAN();

  static void TorqueToBytes(int16_t torque, uint8_t &upper, uint8_t &lower);
  static void SpeedToBytes(int16_t speed, uint8_t &upper, uint8_t &lower);

 public:
  C610Bus();
  void PollCAN();
  void Callback(CAN_message_t &msg);

  void CommandTorques(const int32_t torque0, const int32_t torque1 = 0,
                      const int32_t torque2 = 0, const int32_t torque3 = 0,
                      C610Subbus subbus = C610Subbus::kOneToFourBlinks);

  void CommandSpeeds(const int32_t speed0, const int32_t speed1 = 0,
                     const int32_t speed2 = 0, const int32_t speed3 = 0,
                     C610Subbus subbus = C610Subbus::kOneToFourBlinks);

  C610 &Get(const uint8_t i);
};
