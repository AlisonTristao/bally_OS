#ifndef SETUP_H
#define SETUP_H

// header
#include <Settings.h>
#include <ParallelProcessing.h>
#include <StaticObjects.h>

name setup_to_wait() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::log("state_changed: setup -> wait", logType::INFO);
    #endif

    // return the name of the next state
    return WAIT;
}

name setup_to_error() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::log("state_changed: setup -> error", logType::INFO);
    #endif

    // return the name of the next state
    return ERROR;
}

name setup_function() {
    // init communication 
    Serial.begin(921600);  
    delay(100);

    // init pins direction, settings, i2c communication...
    if(!init_structure())
        return setup_to_error();

    // init interruptions in parallel processing
    if(!init_interruptions())
        return setup_to_error();

    // init robot objects
    ROBOT::init();

    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::log("Setup function called", logType::INFO);
    #endif

    // setup completed, go to wait state
    return setup_to_wait();
}

name next_state_setup(uint8_t buttons){
    // if button 1 is pressed or setup completed
    if(buttons & (1 << BIT_0))
        return setup_to_wait();

    // stay in the same state
    return SETUP;
}

#endif