#ifndef FINISH_H
#define FINISH_H

// static objects
#include <StaticObjects.h>

bool finish_function() {
    // stop the robot
    StaticObjects::motor_left.brake();
    StaticObjects::motor_right.brake();

    // finish the robot
    return true;
}

name next_state_finish(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0)){
     
        // log message
        #if defined(LOG_ALL) || defined(LOG_INFO)
            Logger::insert_log("states: Finish -> Wait", logType::INFO);
        #endif
     
        return WAIT;
    }

    // if button 2 is pressed
    if(buttons & (1 << BIT_1)){
        
        // log message
        #if defined(LOG_ALL) || defined(LOG_INFO)
            Logger::insert_log("states: Finish -> Telemetry", logType::INFO);
        #endif
    
        return TELEMETRY; 
    }

    // if button 3 is pressed
    /*if(buttons & (1 << BIT_2))
        return RUN;*/

    // stay in the same state
    return FINISH;
}

#endif