#ifndef DEBUG_H
#define DEBUG_H

// static library includes
#include <Logger.h>
#include <StaticObjects.h>

bool debug_function() {
    // encoder left and encoder right
    string enc_l = String(StaticObjects::encoder_left.getCount()).c_str();
    string enc_r = String(StaticObjects::encoder_right.getCount()).c_str();

    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::insert_log((enc_l + " " + enc_r).c_str(), logType::INFO);
    #endif

    delay(10);

    // map the environment
    return true;
}

name next_state_debug(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0)){
        
        // log message
        #if defined(LOG_ALL) || defined(LOG_INFO)
            Logger::insert_log("states: debug -> Finish", logType::INFO);
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
    return DEBUG;
}

#endif