#include "Buzzer.h"
#include <Arduino.h>

Buzzer::Buzzer(int pin) {
  this->pin = pin;
}

void Buzzer::slowBeep() {
    beepType = SLOW;
}

void Buzzer::fastBeep() {
    beepType = FAST;
}

void Buzzer::longBeep() {
    beepType = LONG;
}

void Buzzer::noBeep() {
    beepType = NONE;
}

void Buzzer::update() {
    int beepTime = millis() - lastBeepTime;

    if (buzzerOn){
        if (beepTime > 100 && beepType != LONG){
            buzzerOn = false;
            digitalWrite(pin, LOW);
        }
        else if (beepTime > 500 && beepType == LONG){
            buzzerOn = false;
            digitalWrite(pin, LOW);
        }

    } else {
        if (beepType == SLOW && beepTime > 1000){
            buzzerOn = true;
            digitalWrite(pin, HIGH);
        } else if (beepType == FAST && beepTime > 500){
            buzzerOn = true;
            digitalWrite(pin, HIGH);
        } else if (beepType == LONG && beepTime > 2000){
            buzzerOn = true;
            digitalWrite(pin, HIGH);
        }
    }

    lastBeepTime = millis();
}