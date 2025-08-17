#ifndef WAIT_H
#define WAIT_H

// header
#include <Settings.h>

// static libraries
#include <StaticObjects.h>

bool wait_function() {
    // motor pwm 0
    StaticObjects::motor_left.applyPWM(0);
    StaticObjects::motor_right.applyPWM(0);

    // wait for the next state
    return true;
}

name next_state_wait(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_2)){

        // log message
        #if defined(LOG_ALL) || defined(LOG_INFO)
            Logger::insert_log("states: Wait -> Map", logType::INFO);
        #endif

        return MAP;    
    }

    // if button 2 is pressed
    if(buttons & (1 << BIT_1)){
        
        // log message
        #if defined(LOG_ALL) || defined(LOG_INFO)
            Logger::insert_log("states: Wait -> Calibrate", logType::INFO);
        #endif

        return CALIBRATE;
    }
    // if button 3 is pressed
    if(buttons & (1 << BIT_0)){

        // log message
        #if defined(LOG_ALL) || defined(LOG_INFO)
            Logger::insert_log("states: Wait -> Run", logType::INFO);
        #endif

        return RUN;
    }
    // stay in the same state
    return WAIT;
}

#endif