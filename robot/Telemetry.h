#ifndef TELEMETRY_H
#define TELEMETRY_H

// header
#include <Settings.h>

// verify if the telemetry function was already called
bool telemetry_ok = false;

bool telemetry_function() {
    // telemetry only once
    if (telemetry_ok)
        return true;

    // return if live is active
    #ifdef LOG_VERBOSE
        return true;
    #endif

    // log message out
    #if defined(LOG_ALL) 
        Logger::send_logger_register(logType::NONE);
    #else
        #if defined(LOG_TELEMETRY)
            Logger::send_logger_register(logType::TELEMETRY);
        #elif defined(LOG_ERROR)
            Logger::send_logger_register(logType::ERROR);
        #elif defined(LOG_DEBUG)
            Logger::send_logger_register(logType::DEBUG);
        #elif defined(LOG_INFO)
            Logger::send_logger_register(logType::INFO);
        #endif
    #endif

    // telemetry the robot
    telemetry_ok = true;
    return true;
}

name next_state_telemetry(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0)){

        // log message
        #if defined(LOG_ALL) || defined(LOG_INFO)
            Logger::insert_log("states: Telemetry -> Wait", logType::INFO);
        #endif

        return WAIT; 
    }

    // if button 2 is pressed
    if(buttons & (1 << BIT_1)){
        
        // log message
        #if defined(LOG_ALL) || defined(LOG_INFO)
            Logger::insert_log("states: Telemetry -> Finish", logType::INFO);
        #endif
        
        return FINISH;
    }
    
    // if button 3 is pressed
    /*if(buttons & (1 << BIT_2))
        return TELEMETRY;*/

    // stay in the same state
    return TELEMETRY;
}

#endif