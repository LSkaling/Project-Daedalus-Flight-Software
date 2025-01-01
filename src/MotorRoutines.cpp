#include "MotorRoutines.h"
#include <Arduino.h>    // For delay(), micros()
#include <Moteus.h>     // For motor control
#include <ACAN2517FD.h> // For CAN interface
MotorRoutines::MotorRoutines()
{
}

bool MotorRoutines::runToEnd(Moteus &motor, float velocity, float current)
{
    Moteus::PositionMode::Command cmd;
    cmd.position = NaN;
    cmd.velocity = velocity;
    motor.SetPosition(cmd);

    while (true)
    {
        const auto motor_result = motor.last_result();
        float position = motor_result.values.position;
        float velocity = motor_result.values.velocity;
        float current = motor_result.values.q_current;

        if (current > 0.5)
        {
            motor.SetStop();
            return true;
        }
    }
    return false;
}