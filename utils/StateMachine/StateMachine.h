#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

// StateMachine name
enum name {
    NONE        = 0,
    SETUP       = 1,
    WAIT        = 2,
    CALIBRATE   = 3,
    MAP         = 4,
    RUN         = 5,
    FINISH      = 6,
    TELEMETRY   = 7,
    ERROR       = 8,

    NUMBER_OF_STATES // put the states before this line
};

/****************************/
/*  Class Of State Machine  */
/****************************/  

class StateMachine{
    public:
        StateMachine(name state, bool (*action)(), name (*next_state)(uint8_t buttons));
        ~StateMachine(){};

        // return the value of state
        uint8_t getValue();

        // run the state machine
        static bool run();

        // verify if the next state 
        static void next(uint8_t buttons);

        // set the state machine to a specific state
        static uint8_t current_state;
    private:
        // index of the state
        union{
            name state;
            uint8_t number;
        };

        // function to execute the state
        bool (*action)();

        // next state function
        name (*next_state)(uint8_t buttons);

        // array of StateMachine
        static StateMachine* arr_states[NUMBER_OF_STATES];
};

#endif