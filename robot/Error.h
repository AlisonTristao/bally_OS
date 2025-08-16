#ifndef ERROR_H
#define ERROR_H

bool error_function() {
    /**********************************
    
        Error the robot here

    ***********************************/

    // error the robot
    return true;
}

name next_state_error(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0)){
        
        // log message
        #if defined(LOG_ALL) || defined(LOG_INFO)
            Logger::insert_log("states: Error -> Telemetry", logType::INFO);
        #endif
    
        return TELEMETRY; 
    }

    // if button 2 is pressed
    /*if(buttons & (1 << BIT_1))
        return ERROR;

    // if button 3 is pressed
    /*if(buttons & (1 << BIT_2))
        return TELEMETRY;*/

    // stay in the same state
    return ERROR;
}

#endif