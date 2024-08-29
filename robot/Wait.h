#ifndef WAIT_H
#define WAIT_H

bool wait_function() {
    // wait for the next state
    return true;
}

name next_state_wait(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_2))
        return CALIBRATE; 

    // if button 2 is pressed
    if(buttons & (1 << BIT_1))
        return MAP;

    // if button 3 is pressed
    if(buttons & (1 << BIT_0))
        return RUN;

    // stay in the same state
    return WAIT;
}

#endif