#pragma once
#include <Arduino.h>
#include <Moteus.h>
#include <ACAN2517FD.h>

class MotorRoutines
{
public:
    MotorRoutines();
    bool runToEnd(Moteus &motor, float velocity, float current);

};
