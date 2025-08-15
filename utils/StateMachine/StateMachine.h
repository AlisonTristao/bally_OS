#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>

// static variables
#include <Logger.h>

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
        /**
         * @brief constructor of the state machine
         * @param state: the state of the state machine
         * @param action: the function to be executed in the state
         * @param next_state: the function to verify the next state
         */
        StateMachine(name state, bool (*action)(), name (*next_state)(uint8_t buttons));
        ~StateMachine(){};

        /**
         * @brief get the value of the state
         * @return the value of the state
         */
        uint8_t getValue();

        /**
         * @brief run the current state
         * @return true if the action was executed successfully
         */
        static bool run();

        /**
         * @brief change the state machine to the next state
         * @param buttons: the buttons pressed
         */
        static void next(uint8_t buttons);

        /**
         * @brief current state of the state machine
         */
        static uint8_t current_state;
    private:
        // index of the state
        union{
            name state;
            uint8_t number;
        };

        /**
         * @brief function to be executed in the state
         * @return true if the action was executed successfully
         */
        bool (*action)();

        /**
         * @brief function to verify the next state
         * @param buttons: the buttons pressed
         * @return the next state
         */
        name (*next_state)(uint8_t buttons);

        /**
         * @brief array of states
         * @note the index of the array is the state
         */
        static StateMachine* arr_states[NUMBER_OF_STATES];
};

#endif