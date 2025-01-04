#ifndef STATES_H
#define STATES_H

enum States
{
    IDLE,
    ARMED,
    IGNITION,
    COAST,
    APOGEE,
    BELLYFLOP,
    CHUTE,
    IMPACT,
    LANDED
};

const char *stateToString(States state);

#endif