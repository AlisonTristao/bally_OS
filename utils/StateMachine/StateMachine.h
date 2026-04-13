#ifndef STATEMACHINE_H
#define STATEMACHINE_H

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

#include <Arduino.h>

// StateMachine name
enum name {
    NONE        = 0,
    SETUP       = 1,
    WAIT        = 2,
    CALIBRATE   = 3,
    DEBUG       = 4,
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
        using ErrorCallback = void (*)(const char* message);
        using StateOutputCallback = void (*)(uint8_t state);
        using InputCallback = uint8_t (*)();

        /**
         * @brief constructor of the state machine
         * @param state: the state of the state machine
         * @param action: state action that returns the next state
         * @param next_state: state transition function based on inputs (buttons)
         */
        StateMachine(name state, name (*action)(), name (*next_state)(uint8_t buttons) = nullptr);
        ~StateMachine(){};

        /**
         * @brief sets callback for state machine errors
         */
        static void setErrorCallback(ErrorCallback callback);

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
         * @brief apply transition to next state with validation
         * @param next_state target state returned by state action
         * @return true if transition is valid and applied
         */
        static bool next(name next_state);

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
         * @return next state after action execution
         */
        name (*action)();

        /**
         * @brief optional transition function using input bitmask
         */
        name (*next_state)(uint8_t buttons);

        /**
         * @brief array of states
         * @note the index of the array is the state
         */
        static StateMachine* arr_states[NUMBER_OF_STATES];

        static ErrorCallback errorCallback_;
        static StateOutputCallback stateOutputCallback_;
        static InputCallback inputCallback_;

        static void defaultErrorCallback(const char* message);
        static uint8_t defaultInputCallback();
        static void reportError(const char* message);
};

#endif