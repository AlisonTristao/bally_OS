#ifndef MAP_H
#define MAP_H

bool map_function() {
    /**********************************
    
        Map the environment here

    ***********************************/

    // map the environment
    return true;
}

name next_state_map(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0)){
        
        // log message
        #if defined(LOG_ALL) || defined(LOG_INFO)
            Logger::IN_LOG("states: Map -> Finish", logType::INFO);
        #endif
        
        return FINISH;
    }

    // if button 2 is pressed
    /*if(buttons & (1 << BIT_1))
        return CALIBRATE;

    // if button 3 is pressed
    if(buttons & (1 << BIT_2))
        return RUN;*/

    // stay in the same state
    return MAP;
}

#endif