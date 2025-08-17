#ifndef CALIBRATE_H
#define CALIBRATE_H

// header
#include <Settings.h>

// custom libraries
#include <ArraySensor.h>

// static library
#include <StaticObjects.h>

// verify if the calibrate function was already called
bool calibrate_ok = false;

bool calibrate_function() {
    // calibrate only once
    if (calibrate_ok)
        return true;    

    // calibrate the sensors
    bool calib = StaticObjects::array_sensor.calibrate(SAMPLES, DELAY_SAMPLE);
    if(calib) StaticObjects::array_sensor.saveCalibration();

    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::insert_log(("Calibrate function called: " + String(!calib ? "failed" : "success")), logType::INFO);
        Logger::insert_log("Values:\n\n" + StaticObjects::array_sensor.calibrate_status(), logType::INFO);
    #endif

    // error 
    if(!calib) return false;

    // all ok
    calibrate_ok = true;
    return true;
}


name next_state_calibrate(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0)){

        // log message
        #if defined(LOG_ALL) || defined(LOG_INFO)
            Logger::insert_log("states: Calibrate -> Wait", logType::INFO);
        #endif

        calibrate_ok = false;
        return WAIT; 
    }

    // if button 2 is pressed
    /*if(buttons & (1 << BIT_1))
        return MAP;

    // if button 3 is pressed
    if(buttons & (1 << BIT_2))
        return RUN;*/

    // stay in the same state
    return CALIBRATE;
}

#endif