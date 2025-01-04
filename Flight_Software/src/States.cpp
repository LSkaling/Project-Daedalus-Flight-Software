#include "States.h"

const char *stateToString(States state)
{
    switch (state)
    {
    case IDLE:
        return "IDLE";
    case ARMED:
        return "ARMED";
    case IGNITION:
        return "IGNITION";
    case COAST:
        return "COAST";
    case APOGEE:
        return "APOGEE";
    case BELLYFLOP:
        return "BELLYFLOP";
    case CHUTE:
        return "CHUTE";
    case IMPACT:
        return "IMPACT";
    case LANDED:
        return "LANDED";
    default:
        return "UNKNOWN";
    }
}