#ifndef ERROR_H
#define ERROR_H

#include <Settings.h>

name error_to_telemetry() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("state_changed: error -> telemetry", logType::INFO);
    #endif

    // return the name of the next state
    return TELEMETRY;
}

name error_to_wait() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("state_changed: error -> wait", logType::INFO);
    #endif

    // return the name of the next state
    return WAIT;
}

name error_function() {
    /**********************************
    
        Error the robot here

    ***********************************/

    return ERROR;
}

name next_state_error(uint8_t buttons){
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