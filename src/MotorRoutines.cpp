#include "MotorRoutines.h"

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
        int mode = static_cast<int>(motor_result.values.mode);

        motor.SetPosition(cmd);

        Serial1.print("Mode: " + String(mode));
        Serial1.print(" Position: " + String(position));
        Serial1.print(" Velocity: " + String(velocity));
        Serial1.print(" Current: " + String(current));
        Serial1.println();

        if (current > 0.5)
        {
            motor.SetStop();
            return true;
        }

        delay(50); // 50 ms = 20 Hz
    }
    return false;
}