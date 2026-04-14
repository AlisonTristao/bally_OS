#ifndef STATEMACHINE_H
#define STATEMACHINE_H

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

#include <Arduino.h>

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

    NUMBER_OF_STATES
};

class StateMachine {
public:
    using ErrorCallback = void (*)(const char* message);
    using StateOutputCallback = void (*)(uint8_t state);
    using InputCallback = uint8_t (*)();

    StateMachine();
    StateMachine(name state, name (*action)(), name (*next_state)(uint8_t buttons) = nullptr);
    ~StateMachine(){};

    void setErrorCallback(ErrorCallback callback);
    uint8_t getValue() const;
    bool run();
    bool next(name next_state);

    uint8_t current_state;

private:
    name state = NONE;
    name (*action)() = nullptr;
    name (*next_state_fn)(uint8_t buttons) = nullptr;
    bool is_controller_ = false;

    StateMachine* arr_states[NUMBER_OF_STATES] = {};
    ErrorCallback errorCallback_ = defaultErrorCallback;
    StateOutputCallback stateOutputCallback_ = nullptr;
    InputCallback inputCallback_ = defaultInputCallback;

    static StateMachine* active_machine_;

    static void defaultErrorCallback(const char* message);
    static uint8_t defaultInputCallback();
    void reportError(const char* message);
};

#endif
