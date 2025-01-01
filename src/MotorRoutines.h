#ifndef MOTOR_ROUTINES_H
#define MOTOR_ROUTINES_H

#include <Arduino.h>
#include <ACAN2517FD.h>
#include <Moteus.h>

class MotorRoutines
{
public:
    MotorRoutines();
    static bool runToEnd(Moteus &motor, float velocity, float current);

};

#endif