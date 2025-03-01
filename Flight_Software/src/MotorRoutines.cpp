#include "MotorRoutines.h"

MotorRoutines::MotorRoutines()
{
}

float MotorRoutines::runToEnd(Moteus &motor, float travel_velocity, float threashold_current)
{
    Moteus::PositionMode::Command cmd;
    cmd.position = NaN;
    cmd.velocity = travel_velocity;
    cmd.velocity_limit = travel_velocity * 2;
    motor.SetPosition(cmd);

    int start_time = millis();

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

        if (abs(current) > threashold_current && millis() - start_time > 500)
        {
            Serial1.println("Found end stop");
            motor.SetStop();
            return motor.last_result().values.position;
        }

        delay(50); // 50 ms = 20 Hz
    }
    return 0;
}

float MotorRoutines::measureTravelDistance(Moteus &motor, float velocity, float current){
    float topPosition = runToEnd(motor, velocity, current);
    float bottomPosition = runToEnd(motor, -velocity, current);
    return topPosition - bottomPosition;
}

bool MotorRoutines::testClutch(Moteus &motor, Clutch &clutch, float maxCurrent){

    const int testTime = 3000;
    const float movementFailThreadshold = 0.05; //TODO: get actual number

    float startPosition = motor.last_result().values.position;

    Moteus::CurrentMode::Command cmd;
    cmd.d_A = maxCurrent;
    cmd.q_A = maxCurrent; //TODO: Are these the same?
    motor.SetCurrent(cmd);

    int startTime = millis();
    while (millis() - startTime < testTime){
        delay(10);
    }

    float endPosition = motor.last_result().values.position;

    Serial1.print("Start position: ");
    Serial1.print(startPosition);
    Serial1.print("End position: ");
    Serial1.println(endPosition);

    return endPosition - startPosition < movementFailThreadshold;
}

float MotorRoutines::measureMovingResistance(Moteus &motor, float velocity){
    const float threadsholdCurrent = 0.5;
    runToEnd(motor, 5, 0.5); //TODO: replace with actual motor resistance when horizontal.
    float current = 0;

    float currents[] = {};
    float positions[] = {};

    Moteus::PositionMode::Command cmd;
    cmd.position = NaN;
    cmd.velocity = velocity;
    motor.SetPosition(cmd);

    int i = 0;

    while(current < threadsholdCurrent){
        const auto motor_result = motor.last_result();
        float position = motor_result.values.position;
        float velocity = motor_result.values.velocity;
        current = motor_result.values.q_current;

        motor.SetPosition(cmd);

        currents[i] = current;
        positions[i] = position;

        i++;

        delay(20);
    }

    return current; //TODO: Best data structure to give position & current?

}

void MotorRoutines::testImpact(Moteus &motor){
    
}

void MotorRoutines::moveToPositionBlocking(Moteus &motor, float position, float velocity, float current){
    Moteus::PositionMode::Command cmd;
    cmd.position = position;
    cmd.velocity = NaN;
    

    Moteus::PositionMode::Format format;
    format.velocity_limit = Moteus::kFloat;
    cmd.velocity_limit = velocity;
    format.maximum_torque = Moteus::kFloat;
    cmd.maximum_torque = 5;

    motor.SetPosition(cmd, &format);

    while (true)
    {
        const auto motor_result = motor.last_result();
        float current = motor_result.values.q_current;
        float position = motor_result.values.position;
        float velocity = motor_result.values.velocity;

        motor.SetPosition(cmd, &format);

        Serial1.print("Position: " + String(position));
        Serial1.print(" Velocity: " + String(velocity));
        Serial1.print(" Current: " + String(current));
        Serial1.println();

        if (abs(position - cmd.position) < 0.1)
        {
            Serial1.println("Reached position");
            motor.SetStop();
            return;
        }

        delay(50); // 50 ms = 20 Hz
    }
}

void MotorRoutines::moveToPosition(Moteus &motor, float position, float velocity, float torque, float accel, float minPos, float maxPos){

    if(position < minPos){
        position = minPos;
    } else if(position > maxPos){
        position = maxPos;
    }

    Moteus::PositionMode::Command cmd;
    cmd.position = position;
    cmd.velocity = NaN;

    Moteus::PositionMode::Format format;
    format.accel_limit = Moteus::kFloat;
    format.velocity_limit = Moteus::kFloat;
    format.maximum_torque = Moteus::kFloat;
    cmd.accel_limit = accel;
    cmd.velocity_limit = velocity;
    cmd.maximum_torque = torque;


    motor.SetPosition(cmd, &format);
}