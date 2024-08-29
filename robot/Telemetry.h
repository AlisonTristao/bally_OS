#ifndef TELEMETRY_H
#define TELEMETRY_H

bool telemetry_function() {
    /**********************************
    
        Telemetry the robot here

    ***********************************/

    // telemetry the robot
    return true;
}

name next_state_telemetry(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0))
        return WAIT; 

    // if button 2 is pressed
    if(buttons & (1 << BIT_1))
        return FINISH;

    // if button 3 is pressed
    /*if(buttons & (1 << BIT_2))
        return TELEMETRY;*/

    // stay in the same state
    return TELEMETRY;
}

#endif