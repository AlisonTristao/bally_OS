#ifndef FINISH_H
#define FINISH_H

bool finish_function() {
    /**********************************
    
        Finish the robot here

    ***********************************/

    // finish the robot
    return true;
}

name next_state_finish(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0))
        return TELEMETRY; 

    // if button 2 is pressed
    if(buttons & (1 << BIT_1))
        return WAIT;

    // if button 3 is pressed
    /*if(buttons & (1 << BIT_2))
        return RUN;*/

    // stay in the same state
    return FINISH;
}

#endif