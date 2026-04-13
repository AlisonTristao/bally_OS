#ifndef DEBUG_H
#define DEBUG_H

// static library includes
#include <Logger.h>
#include <StaticObjects.h>

name debug_to_finish() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::log("state_changed: debug -> finish", logType::INFO);
    #endif

    // return the name of the next state
    return FINISH;
}

name debug_to_wait() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::log("state_changed: debug -> wait", logType::INFO);
    #endif

    // return the name of the next state
    return WAIT;
}

name debug_function() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::log((ROBOT::debugSensors()).c_str(), logType::INFO);
    #endif

    // safety default outside RUN
    delay(DELAY_FLAGS);

    return DEBUG;
}

name next_state_debug(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0))
        return debug_to_finish();

    // if button 2 is pressed
    if(buttons & (1 << BIT_1))
        return debug_to_wait();

    // stay in the same state
    return DEBUG;
}

#endif