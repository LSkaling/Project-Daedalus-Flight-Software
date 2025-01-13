#ifndef CLUTCH_H
#define CLUTCH_H

#include <Servo.h>

class Clutch
{
public:
    Clutch(int servoPin, int engageAngle, int disengageAngle);
    void begin();
    void engage();
    void disengage();

private:
    int servoPin;
    int engageAngle = 0;
    int disengageAngle = 180;
    Servo servo; //TODO: Is it faster to use analogWrite instead of servo class?

};




#endif