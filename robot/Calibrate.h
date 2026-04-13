#ifndef CALIBRATE_H
#define CALIBRATE_H

// header
#include <Settings.h>

// custom libraries
#include <ArraySensor.h>

// static library
#include <StaticObjects.h>

name calibrate_to_wait() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::log("state_changed: calibrate -> wait", logType::INFO);
    #endif

    // return the name of the next state
    return WAIT;
}

name calibrate_to_error() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::log("state_changed: calibrate -> error", logType::INFO);
    #endif

    // return the name of the next state
    return ERROR;
}

name calibrate_function() {
    const bool calib = ROBOT::calibrateSensors(SAMPLES, DELAY_SAMPLE);

    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::log(("Calibrate function called: " + String(!calib ? "failed" : "success")), logType::INFO);
        ROBOT::log("Values:\n\n" + ROBOT::calibrateStatus(), logType::INFO);
    #endif

    if(calib)
        ROBOT::saveCalibration();
    else 
        return calibrate_to_error();

    return calibrate_to_wait();
}

name next_state_calibrate(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0))
        return calibrate_to_wait();

    // stay in the same state
    return CALIBRATE;
}

#endif