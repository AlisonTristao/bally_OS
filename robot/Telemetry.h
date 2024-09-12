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
        Logger::OUT_LOGGER(logType::NONE);
    #else
        #if defined(LOG_TELEMETRY)
            Logger::OUT_LOGGER(logType::TELEMETRY);
        #elif defined(LOG_ERROR)
            Logger::OUT_LOGGER(logType::ERROR);
        #elif defined(LOG_DEBUG)
            Logger::OUT_LOGGER(logType::DEBUG);
        #elif defined(LOG_INFO)
            Logger::OUT_LOGGER(logType::INFO);
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
            Logger::IN_LOG("states: Telemetry -> Wait", logType::INFO);
        #endif

        return WAIT; 
    }

    // if button 2 is pressed
    if(buttons & (1 << BIT_1)){
        
        // log message
        #if defined(LOG_ALL) || defined(LOG_INFO)
            Logger::IN_LOG("states: Telemetry -> Finish", logType::INFO);
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