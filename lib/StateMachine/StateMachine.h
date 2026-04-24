#ifndef STATEMACHINE_H
#define STATEMACHINE_H

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// StateMachine Name
enum stateName {
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

        /**
         * @brief constructor of the state machine
         * @param state: the state of the state machine
         * @param action: the function to be executed in the state
         * @param next_state: the function to verify the next state
         */
        StateMachine(stateName state, stateName (*action)(), stateName (*next_state)(uint8_t buttons));
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
        bool run();

        /**
         * @brief change the state machine to the next state
         * @param buttons: the buttons pressed
         */
        bool next(uint8_t buttons);

        /**
         * @brief set the error callback function
         * @param callback: the function to be called when an error occurs
         */
        void setErrorCallback(ErrorCallback callback);

        /**
         * @brief current state of the state machine
         */
        static std::atomic<uint8_t> current_state;
    private:
        // index of the state
        union{
            stateName state;
            uint8_t number;
        };

        /**
         * @brief verify if the state is valid
         * @return true if the action was executed successfully, false otherwise
         */
        bool verifyState(uint8_t state) const;

        /**
         * @brief function to be executed in the state
         * @return true if the action was executed successfully
         */
        stateName (*action)();

        /**
         * @brief function to verify the next state
         * @param buttons: the buttons pressed
         * @return the next state
         */
        stateName (*next_state)(uint8_t buttons);

        static StateMachine* arr_states[NUMBER_OF_STATES];
        static SemaphoreHandle_t transitionMutex_;
        static ErrorCallback errorCallback_;
        static void defaultErrorCallback(const char* message);
        static bool reportError(const char* message);
};

#endif
