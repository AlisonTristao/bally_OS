#ifndef CALIBRATE_H
#define CALIBRATE_H

// header
#include <Pinout.h>

// custom libraries
#include <ArraySensor.h>

// static library
#include <StaticObjects.h>

stateName calibrate_to_wait() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("state_changed: calibrate -> wait", logType::INFO);
    #endif

    // return the stateName of the next state
    return WAIT;
}

stateName calibrate_to_error() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("state_changed: calibrate -> error", logType::INFO);
    #endif

    // return the stateName of the next state
    return ERROR;
}

stateName calibrate_function() {
    const bool calib = ROBOT::array_sensor.calibrate(SAMPLES, DELAY_SAMPLE);

    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log(("Calibrate function called: " + String(!calib ? "failed" : "success")), logType::INFO);
        ROBOT::logger.insert_log("Values:\n\n" + ROBOT::array_sensor.calibrate_status(), logType::INFO);
    #endif

    if(calib)
        ROBOT::array_sensor.saveCalibration();
    else 
        return calibrate_to_error();

    return calibrate_to_wait();
}

stateName next_state_calibrate(uint8_t buttons){
    // stay in the same state
    return CALIBRATE;
}

#endif