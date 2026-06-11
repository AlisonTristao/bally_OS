#include "StatesManager.h"
States* States::instance_ = nullptr;

// conversion from stateName to string for logging purposes
static const char* stateToString(stateName state) {
    switch (state) {
        case NONE:          return "NONE";
        case SETUP:         return "SETUP";
        case WAIT:          return "WAIT";
        case CALIBRATE:     return "CALIBRATE";
        case DEBUG:         return "DEBUG";
        case RUN:           return "RUN";
        case FINISH:        return "FINISH";
        case TELEMETRY:     return "TELEMETRY";
        case ERROR:         return "ERROR";
        default:            return "UNKNOWN";
    }
}

stateName States::process_transition(stateName currentState, uint8_t buttons) {
    stateName nextState = currentState;
    for (int i = 0; i < NUM_TRANSITIONS; i++)
        if (transitionTable[i].currentState == currentState)
            if (buttons & transitionTable[i].buttonMask)
                nextState = transitionTable[i].nextState;

    #if defined(LOG_ALL) || defined(LOG_INFO)
    if (nextState != currentState)
        // log message
        robot_.logger.insert_logf(logType::INFO, "state transition: %s -> %s", stateToString(currentState), stateToString(nextState));
    #endif

    return nextState; 
}

stateName States::go_to(stateName currentState, stateName nextState) {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        robot_.logger.insert_logf(logType::INFO, "state changed: %s -> %s", stateToString(currentState), stateToString(nextState));
    #endif

    return nextState;
}