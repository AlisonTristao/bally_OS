#ifndef RUN_H
#define RUN_H

bool run_function() {
    /**********************************
    
        Run the robot here

    ***********************************/

    // run the robot
    return true;
}

name next_state_run(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0))
        return FINISH; 

    // if button 2 is pressed
    /*if(buttons & (1 << BIT_1))
        return MAP;

    // if button 3 is pressed
    if(buttons & (1 << BIT_2))
        return RUN;*/

    // stay in the same state
    return RUN;
}

#endif