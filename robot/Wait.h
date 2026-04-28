#ifndef WAIT_H
#define WAIT_H

// header
#include <Pinout.h>

// static libraries
#include <StaticObjects.h>

stateName wait_to_debug() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("state_changed: wait -> debug", logType::INFO);
    #endif

    // return the stateName of the next state
    return DEBUG;
}

stateName wait_to_calibrate() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("state_changed: wait -> calibrate", logType::INFO);
    #endif

    // return the stateName of the next state
    return CALIBRATE;
}

stateName wait_to_run() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("state_changed: wait -> run", logType::INFO);
    #endif

    // return the stateName of the next state
    return RUN;
}

stateName wait_function() {
    return WAIT;
}

stateName next_state_wait(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_2))
        return wait_to_debug();

    // if button 2 is pressed
    if(buttons & (1 << BIT_1))
        return wait_to_calibrate();

    // if button 3 is pressed
    if(buttons & (1 << BIT_0))
        return wait_to_run();

    // stay in the same state
    return WAIT;
}

#endif