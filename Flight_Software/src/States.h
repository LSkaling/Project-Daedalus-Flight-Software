#ifndef STATES_H
#define STATES_H

enum States
{
    INTEGRATING,
    IDLE,
    ARMED,
    FLIGHT,
    APOGEE,
    BELLYFLOP,
    CHUTE,
    LANDED
};

const char *stateToString(States state);

#endif