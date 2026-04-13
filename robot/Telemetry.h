#ifndef TELEMETRY_H
#define TELEMETRY_H

// header
#include <Settings.h>

name telemetry_to_wait() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::log("state_changed: telemetry -> wait", logType::INFO);
    #endif

    // return the name of the next state
    return WAIT;
}

name telemetry_to_finish() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::log("state_changed: telemetry -> finish", logType::INFO);
    #endif

    // return the name of the next state
    return FINISH;
}

name telemetry_function() {
    // log message out
    #if defined(LOG_ALL)
        ROBOT::sendLogger(logType::NONE);
    #else
        #if defined(LOG_TELEMETRY)
            ROBOT::sendLogger(logType::TELEMETRY);
        #elif defined(LOG_ERROR)
            ROBOT::sendLogger(logType::ERROR);
        #elif defined(LOG_DEBUG)
            ROBOT::sendLogger(logType::DEBUG);
        #elif defined(LOG_INFO)
            ROBOT::sendLogger(logType::INFO);
        #endif
    #endif

    return telemetry_to_wait();
}

name next_state_telemetry(uint8_t buttons){
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