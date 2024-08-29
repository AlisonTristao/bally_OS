#ifndef CALIBRATE_H
#define CALIBRATE_H

// header
#include <Settings.h>

// custom libraries
#include <ArraySensor.h>

// verify if the calibrate function was already called
bool calibrate_ok = false;

bool calibrate_function() {
    // calibrate only once
    if (calibrate_ok)
        return true;    

    // create the sensor object
    ArraySensor sensor(LEN_SENSOR, SIG, C0, C1, C2, C3);
    sensor.set_init_arr(INIT_MUX);

    // calibrate the sensors
    if (!sensor.calibrate(SAMPLES, DELAY_SAMPLE)) 
        return false;
    else                            
        sensor.saveCalibration();

    // all ok
    calibrate_ok = true;
    return true;
}


name next_state_calibrate(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0)){
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