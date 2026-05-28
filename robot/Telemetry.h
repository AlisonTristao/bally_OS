#ifndef TELEMETRY_H
#define TELEMETRY_H

// header
#include <Pinout.h>

stateName telemetry_to_wait() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log(logType::INFO, "state_changed: telemetry -> wait");
    #endif

    // return the stateName of the next state
    return WAIT;
}

stateName telemetry_to_finish() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log(logType::INFO, "state_changed: telemetry -> finish");
    #endif

    // return the stateName of the next state
    return FINISH;
}

stateName telemetry_function() {
    return TELEMETRY;
}

stateName next_state_telemetry(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0))
        return telemetry_to_wait();

    // if button 2 is pressed
    if(buttons & (1 << BIT_1))
        return telemetry_to_finish();

    // stay in the same state
    return TELEMETRY;
}

#endif