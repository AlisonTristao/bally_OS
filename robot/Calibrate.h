#ifndef CALIBRATE_H
#define CALIBRATE_H

bool calibrate_function() {
    /**********************************
    
        Calibrate the robot here

    ***********************************/

    // calibrate the robot
    return true;
}

name next_state_calibrate(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0))
        return WAIT; 

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