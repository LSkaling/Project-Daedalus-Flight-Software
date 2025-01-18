#include "States.h"

const char *stateToString(States state)
{
    switch (state)
    {
    case INTEGRATING:
        return "INTEGRATING";
    case IDLE:
        return "IDLE";
    case ARMED:
        return "ARMED";
    case FLIGHT:
        return "FLIGHT";
    case APOGEE:
        return "APOGEE";
    case BELLYFLOP:
        return "BELLYFLOP";
    case CHUTE:
        return "CHUTE";
    case LANDED:
        return "LANDED";
    default:
        return "UNKNOWN";
    }
}