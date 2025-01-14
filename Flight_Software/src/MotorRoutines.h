#ifndef MOTOR_ROUTINES_H
#define MOTOR_ROUTINES_H

#include <Arduino.h>
#include <ACAN2517FD.h>
#include <Moteus.h>
#include <Clutch.h>

class MotorRoutines
{
public:
    MotorRoutines();
    static float runToEnd(Moteus &motor, float velocity, float threashold_current);//TODO: Add default values
    static float measureTravelDistance(Moteus &motor, float velocity, float current);
    static bool testClutch(Moteus &motor, Clutch &clutch, float maxCurrent);
    static float measureMovingResistance(Moteus &motor, float velocity); //TODO: How to return (float, float) array?
    static void testImpact(Moteus &motor); //TODO: What's VFOC mode, and should i Use it instead of current?
    static void moveToPositionBlocking(Moteus &motor, float position, float velocity, float current);
    static void moveToPosition(Moteus &motor, float position, float velocity, float current, float minPos, float maxPos);
};

#endif