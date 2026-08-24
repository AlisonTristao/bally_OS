#ifndef STATEMACHINE_H
#define STATEMACHINE_H

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

#include <cstdint>
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
        ~StateMachine(){
            // delete the mutex
            if (transitionMutex_ == nullptr)
                return;
            vSemaphoreDelete(transitionMutex_);
            transitionMutex_ = nullptr;
        };

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
         * @brief verify if every registered state has both callbacks defined
         * @return true if all states are properly configured, false otherwise
         */
        bool verifyCallbacks();

        /**
         * @brief set the error callback function
         * @param callback: the function to be called when an error occurs
         */
        void setErrorCallback(ErrorCallback callback);

        /**
         * @brief current state of the state machine
         */
        static std::atomic<uint8_t> current_state;

        /**
         * @brief human-readable name of a state value ("WAIT", "RUN", ...),
         * or "UNKNOWN" for anything outside stateName.
         *
         * Public for the same reason current_state is (see CONTRIBUTING.md):
         * any module that may read the current state also needs to be able to
         * print it, and duplicating this switch is how the names drift.
         */
        static const char* stateToString(uint8_t state);

        /**
         * @brief Value of stateToString() read backwards: "WAIT" -> WAIT.
         * Case-insensitive. @return NONE when the name is not a state.
         */
        static uint8_t stateFromString(const char* name);

        /**
         * @brief Ask for a move to `state` at the next transition check,
         * instead of pretending a button was pressed.
         *
         * This class only HOLDS the request; whoever owns the transition
         * policy is what consumes it (here: States::process_transition in
         * src/robot/StatesManager.cpp), so a remote request still passes
         * through the same gates a button press does -- USB owning the SD
         * card, an OTA write in progress, and the lock below.
         *
         * Written by the shell task, read by the state-machine task; one
         * pending request at a time, a second one overwrites the first.
         *
         * @return false when `state` is not a valid stateName.
         */
        static bool request_state(uint8_t state);

        /**
         * @brief Consume a pending request. @return the requested state, or
         * NONE when there was none. Single consumer by construction.
         */
        static uint8_t take_request();

        /** @brief Peek at a pending request without consuming it. */
        static uint8_t pending_request();

        /**
         * @brief While locked, the transition policy must keep the current
         * state -- no button, no side sensor and no request moves it.
         *
         * The point is remote configuration: holding the machine still while
         * settings are edited over the radio, without physically holding the
         * robot. It does NOT stop the current state's action function from
         * running, and it deliberately does not survive a reboot.
         */
        static void set_locked(bool locked);
        static bool is_locked();

        /** @brief One entry of the transition history ring. */
        struct TransitionRecord {
            uint8_t  from;
            uint8_t  to;
            uint32_t uptime_ms;
        };

        static constexpr size_t kHistoryDepth = 8;

        /**
         * @brief Copy up to `max_records` transitions, newest first.
         *
         * Answers "how did it get to ERROR" without depending on the log
         * having survived. Recorded inside run()/next(), under the same mutex
         * that publishes the new state, so a reader never sees a half-written
         * entry.
         *
         * @return the number of records written.
         */
        static size_t history(TransitionRecord* out, size_t max_records);
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
        static bool verifyState(uint8_t state);

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

        // NONE means "no request pending". Only take_request() clears it.
        static std::atomic<uint8_t> requested_state_;
        static std::atomic<bool>    locked_;

        // Ring of the last kHistoryDepth transitions. Written only from
        // run()/next() with transitionMutex_ held; history() takes the same
        // mutex to read. head_ is the index the NEXT record goes to.
        static TransitionRecord history_[kHistoryDepth];
        static size_t           history_head_;
        static size_t           history_count_;

        // Call with transitionMutex_ held, right after current_state changes.
        // A no-op when from == to, so a state that simply returns itself every
        // pass does not flood the ring.
        static void recordTransition(uint8_t from, uint8_t to);
        static ErrorCallback errorCallback_;
        static void defaultErrorCallback(const char* message);
        static bool reportError(const char* message);
};

#endif
