#ifndef SETUP_H
#define SETUP_H

// header
#include <Settings.h>
#include <ParallelProcessing.h>
#include <StaticObjects.h>

// verify if the setup function was already called
bool configure_ok = false;

bool setup_function() {
    // configure only once
    if(configure_ok) 
        return true;

    // init communication 
    Serial.begin(921600);  

    // init pins direction, settings, i2c communication...
    if(!init_structure()) 
        return false;

    // init interruptions in parallel processing
    if(!init_interruptions()) 
        return false;

    // init static objects
    StaticObjects::init_objects();

    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::insert_log("Setup function called", logType::INFO);
    #endif

    // all ok
    configure_ok = true;
    return configure_ok;
}

name next_state_setup(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0)){
        // log message
        #if defined(LOG_ALL) || defined(LOG_INFO)
            Logger::insert_log("states: Setup -> Wait", logType::INFO);
        #endif

        return WAIT; 
    }

    // if button 2 is pressed
    /*if(buttons & (1 << BTN2)){
        // LOGGER conf
        return CALIBRATE;
    }

    // if button 3 is pressed
    if(buttons & (1 << BTN3)){
        // LOGGER conf
        // return ERROR;
    }*/

    // stay in the same state
    return SETUP;
}

#endif