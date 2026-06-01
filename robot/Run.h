#ifndef RUN_H
#define RUN_H

// header
#include <Settings.h>

// custom libraries
#include <ArraySensor.h>
#include <HBridge.h>
#include <Encoder.h>

stateName run_to_finish() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log(logType::INFO,"state_changed: run -> finish");
    #endif

    // return the stateName of the next state
    return FINISH;
}

stateName run_function() {

    return RUN;
}

stateName next_state_run(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0))
        return run_to_finish();

    // stay in the same state
    return RUN;
}

#endif