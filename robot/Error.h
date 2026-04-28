#ifndef ERROR_H
#define ERROR_H

#include <Pinout.h>

stateName error_to_telemetry() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("state_changed: error -> telemetry", logType::INFO);
    #endif

    // return the stateName of the next state
    return TELEMETRY;
}

stateName error_to_wait() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("state_changed: error -> wait", logType::INFO);
    #endif

    // return the stateName of the next state
    return WAIT;
}

stateName error_function() {
    /**********************************
    
        Error the robot here

    ***********************************/

    return ERROR;
}

stateName next_state_error(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0))
        return error_to_wait();

    // if button 2 is pressed
    if(buttons & (1 << BIT_1))
        return error_to_wait();

    // stay in the same state
    return ERROR;
}

#endif