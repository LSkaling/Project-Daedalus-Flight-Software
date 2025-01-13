#include <Clutch.h>

Clutch::Clutch(int servoPin, int engageAngle, int disengageAngle){
    this->servoPin = servoPin;
    this->engageAngle = engageAngle;
    this->disengageAngle = disengageAngle;
}

void Clutch::begin(){
    servo.attach(servoPin);
    disengage();
}

void Clutch::engage(){
    servo.write(engageAngle);
}

void Clutch::disengage(){
    servo.write(disengageAngle);
}