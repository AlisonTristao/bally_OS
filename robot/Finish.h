#ifndef FINISH_H
#define FINISH_H

// static objects
#include <StaticObjects.h>

name finish_to_wait() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::log("state_changed: finish -> wait", logType::INFO);
    #endif

    // return the name of the next state
    return WAIT;
}

name finish_to_telemetry() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::log("state_changed: finish -> telemetry", logType::INFO);
    #endif

    // return the name of the next state
    return TELEMETRY;
}

name finish_function() {
    return FINISH;
}

name next_state_finish(uint8_t buttons){

    // if button 1 is pressed
    if(buttons & (1 << BIT_0))
        return finish_to_wait();

    // if button 2 is pressed
    if(buttons & (1 << BIT_1))
        return finish_to_telemetry();

    // stay in the same state
    return FINISH;
}

#endif