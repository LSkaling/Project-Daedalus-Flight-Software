#include "Arduino.h"
#include "C610Bus.h"

C610Bus::C610Bus() {
  for (uint8_t i = 0; i < kSize; i++) {
    controllers_[i] = C610();
  }
  InitializeCAN();
}

void C610Bus::InitializeCAN() {
  if (!CAN.begin(1000000)) {
    Serial.println("CAN initialization failed!");
    return;
  }
  CAN.filter(0x200, 0x7FF);  // Set filter to accept relevant IDs
  CAN.onReceive([this](int packetSize) { this->Callback(); });
  is_initialized_ = true;
}


void C610Bus::TorqueToBytes(int16_t torque, uint8_t &upper, uint8_t &lower) {
  upper = (torque >> 8) & 0xFF;
  lower = torque & 0xFF;
}

void C610Bus::SpeedToBytes(int16_t speed, uint8_t &upper, uint8_t &lower) {
  upper = (speed >> 8) & 0xFF;
  lower = speed & 0xFF;
}

void C610Bus::PollCAN() {
  CAN.loop();
}

void C610Bus::Callback() {
  CANPacket msg;
  while (CAN.available()) {
    CAN.read(msg);

    if (msg.id >= kReceiveBaseID + 1 && msg.id <= kReceiveBaseID + kSize) {
      uint8_t esc_index = msg.id - kReceiveBaseID - 1;
      C610Feedback f = C610::InterpretMessage(msg);
      controllers_[esc_index].UpdateState(f);
    } else {
      Serial.print("Invalid ID for feedback message: ");
      Serial.println(msg.id, HEX);
    }
  }
}


void C610Bus::CommandTorques(const int32_t torque0, const int32_t torque1,
                             const int32_t torque2, const int32_t torque3,
                             C610Subbus subbus) {
  if (!is_initialized_) {
    Serial.println("Bus must be initialized before use.");
    return;
  }

  int16_t t0 = constrain(torque0, -32000, 32000);
  int16_t t1 = constrain(torque1, -32000, 32000);
  int16_t t2 = constrain(torque2, -32000, 32000);
  int16_t t3 = constrain(torque3, -32000, 32000);

  CAN_message_t msg;
  msg.len = 8;

  if (subbus == C610Subbus::kOneToFourBlinks) {
    msg.id = kOneToFourBlinksCommandID;
  } else if (subbus == C610Subbus::kFiveToEightBlinks) {
    msg.id = kFiveToEightBlinksCommandID;
  } else {
    Serial.println("Invalid ESC subbus.");
    return;
  }

  TorqueToBytes(t0, msg.buf[0], msg.buf[1]);
  TorqueToBytes(t1, msg.buf[2], msg.buf[3]);
  TorqueToBytes(t2, msg.buf[4], msg.buf[5]);
  TorqueToBytes(t3, msg.buf[6], msg.buf[7]);

  CAN.write(msg);
}

void C610Bus::CommandSpeeds(const int32_t speed0, const int32_t speed1,
                            const int32_t speed2, const int32_t speed3,
                            C610Subbus subbus) {
  if (!is_initialized_) {
    Serial.println("Bus must be initialized before use.");
    return;
  }

  int16_t s0 = constrain(speed0, -5000, 5000);
  int16_t s1 = constrain(speed1, -5000, 5000);
  int16_t s2 = constrain(speed2, -5000, 5000);
  int16_t s3 = constrain(speed3, -5000, 5000);

  CAN_message_t msg;
  msg.len = 8;

  if (subbus == C610Subbus::kOneToFourBlinks) {
    msg.id = kSpeedCommandID_OneToFour;
  } else if (subbus == C610Subbus::kFiveToEightBlinks) {
    msg.id = kSpeedCommandID_FiveToEight;
  } else {
    Serial.println("Invalid ESC subbus.");
    return;
  }

  SpeedToBytes(s0, msg.buf[0], msg.buf[1]);
  SpeedToBytes(s1, msg.buf[2], msg.buf[3]);
  SpeedToBytes(s2, msg.buf[4], msg.buf[5]);
  SpeedToBytes(s3, msg.buf[6], msg.buf[7]);

  CAN.write(msg);
}

C610 &C610Bus::Get(const uint8_t i) {
  return controllers_[i];
}
