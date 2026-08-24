#include <StateMachine.h>
#include <cstdio>
#include <cstring>

#include "freertos/task.h"

std::atomic<uint8_t> StateMachine::current_state{NONE};
std::atomic<uint8_t> StateMachine::requested_state_{NONE};
std::atomic<bool>    StateMachine::locked_{false};
SemaphoreHandle_t StateMachine::transitionMutex_ = nullptr;
StateMachine::ErrorCallback StateMachine::errorCallback_ = nullptr;

StateMachine::TransitionRecord StateMachine::history_[StateMachine::kHistoryDepth] = {};
size_t StateMachine::history_head_  = 0;
size_t StateMachine::history_count_ = 0;

StateMachine *StateMachine::arr_states[NUMBER_OF_STATES] = {
	NULL,
}; 

StateMachine::StateMachine(stateName state, stateName (*action)(), stateName (*next_state)(uint8_t buttons)){
    this->state = state;
    this->action = action;
    this->next_state = next_state;
    arr_states[number] = this;

    if (transitionMutex_ == nullptr) {
        transitionMutex_ = xSemaphoreCreateRecursiveMutex();
    }
}

const char* StateMachine::stateToString(uint8_t state) {
    switch (state) {
        case NONE: return "NONE";
        case SETUP: return "SETUP";
        case WAIT: return "WAIT";
        case CALIBRATE: return "CALIBRATE";
        case DEBUG: return "DEBUG";
        case RUN: return "RUN";
        case FINISH: return "FINISH";
        case TELEMETRY: return "TELEMETRY";
        case ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

bool StateMachine::verifyState(uint8_t state) {
    return (state > NONE && state < NUMBER_OF_STATES);
}

uint8_t StateMachine::stateFromString(const char* name) {
    if (name == nullptr) return NONE;

    for (uint8_t candidate = NONE + 1; candidate < NUMBER_OF_STATES; ++candidate) {
        const char* text = stateToString(candidate);

        size_t i = 0;
        for (; text[i] != '\0' && name[i] != '\0'; ++i) {
            char a = text[i];
            char b = name[i];
            if (b >= 'a' && b <= 'z') b = static_cast<char>(b - ('a' - 'A'));
            if (a != b) break;
        }
        if (text[i] == '\0' && name[i] == '\0') return candidate;
    }
    return NONE;
}

bool StateMachine::request_state(uint8_t state) {
    if (!verifyState(state)) return false;
    requested_state_.store(state, std::memory_order_release);
    return true;
}

uint8_t StateMachine::take_request() {
    return requested_state_.exchange(NONE, std::memory_order_acq_rel);
}

uint8_t StateMachine::pending_request() {
    return requested_state_.load(std::memory_order_acquire);
}

void StateMachine::set_locked(bool locked) {
    locked_.store(locked, std::memory_order_release);
}

bool StateMachine::is_locked() {
    return locked_.load(std::memory_order_acquire);
}

void StateMachine::recordTransition(uint8_t from, uint8_t to) {
    if (from == to) return;

    history_[history_head_].from      = from;
    history_[history_head_].to        = to;
    history_[history_head_].uptime_ms =
        static_cast<uint32_t>(xTaskGetTickCount()) *
        static_cast<uint32_t>(portTICK_PERIOD_MS);

    history_head_ = (history_head_ + 1) % kHistoryDepth;
    if (history_count_ < kHistoryDepth) ++history_count_;
}

size_t StateMachine::history(TransitionRecord* out, size_t max_records) {
    if (out == nullptr || max_records == 0) return 0;
    if (transitionMutex_ == nullptr) return 0;
    if (xSemaphoreTake(transitionMutex_, portMAX_DELAY) != pdTRUE) return 0;

    const size_t wanted = (max_records < history_count_) ? max_records
                                                         : history_count_;
    // Walk backwards from the newest entry, which sits one slot behind head_.
    for (size_t i = 0; i < wanted; ++i) {
        const size_t index =
            (history_head_ + kHistoryDepth - 1 - i) % kHistoryDepth;
        out[i] = history_[index];
    }

    xSemaphoreGive(transitionMutex_);
    return wanted;
}

void StateMachine::defaultErrorCallback(const char* message) {
    (void)message;
    current_state.store(ERROR, std::memory_order_release);
}

void StateMachine::setErrorCallback(ErrorCallback callback) {
    if (callback) {
        errorCallback_ = callback;
    } else {
        errorCallback_ = defaultErrorCallback;
        current_state.store(ERROR, std::memory_order_release);
    }
}

bool StateMachine::reportError(const char* message) {
    if (errorCallback_ != nullptr)
        errorCallback_(message);

    current_state.store(ERROR, std::memory_order_release);
    return false;
}

bool StateMachine::verifyCallbacks() {
    if (transitionMutex_ != nullptr) {
        if (xSemaphoreTake(transitionMutex_, portMAX_DELAY) != pdTRUE)
            return reportError("Failed to lock transition mutex");
    }

    for (uint8_t i = 1; i < NUMBER_OF_STATES; ++i) {
        StateMachine* stateMachine = arr_states[i];

        if (stateMachine == nullptr) {
            if (transitionMutex_ != nullptr) xSemaphoreGive(transitionMutex_);
            return reportError("StateMachine instance is missing");
        }

        if (stateMachine->action == nullptr) {
            if (transitionMutex_ != nullptr) xSemaphoreGive(transitionMutex_);
            char message[80];
            snprintf(message, sizeof(message), "StateMachine action callback is missing in %s", stateToString(i));
            return reportError(message);
        }

        if (stateMachine->next_state == nullptr) {
            if (transitionMutex_ != nullptr) xSemaphoreGive(transitionMutex_);
            char message[80];
            snprintf(message, sizeof(message), "StateMachine next_state callback is missing in %s", stateToString(i));
            return reportError(message);
        }
    }

    if (transitionMutex_ != nullptr) {
        xSemaphoreGive(transitionMutex_);
    }

    return true;
}

uint8_t StateMachine::getValue(){
    return number;
}

bool StateMachine::run(){
    if (transitionMutex_ == nullptr)
        return reportError("Transition mutex not initialized");

    if (xSemaphoreTake(transitionMutex_, portMAX_DELAY) != pdTRUE)
        return reportError("Failed to lock transition mutex");

    const uint8_t activeState = current_state.load(std::memory_order_acquire);

    // return if state is None
    if(!verifyState(activeState)) {
        xSemaphoreGive(transitionMutex_);
        return reportError("State is not valid");
    }

    if (arr_states[activeState] == nullptr || arr_states[activeState]->action == nullptr) {
        xSemaphoreGive(transitionMutex_);
        return reportError("State action callback is not defined");
    }

    // execute action and use returned state as the next active state
    try {
        const uint8_t nextState =
            static_cast<uint8_t>(arr_states[activeState]->action());
        current_state.store(nextState, std::memory_order_release);
        // Catches the self-transitions the action functions do on their own
        // (SETUP/CALIBRATE/TELEMETRY -> WAIT, via States::go_to), which never
        // pass through next() below.
        recordTransition(activeState, nextState);
    } catch(const std::exception& e) {
        xSemaphoreGive(transitionMutex_);
        // LOGGER erro
        reportError("Error in the loop function state machine");
        return reportError(e.what());
    }

    // all is okay    
    xSemaphoreGive(transitionMutex_);
    return true;
}

bool StateMachine::next(uint8_t buttons){
    if (transitionMutex_ == nullptr)
        return reportError("Transition mutex not initialized");

    if (xSemaphoreTake(transitionMutex_, portMAX_DELAY) != pdTRUE)
        return reportError("Failed to lock transition mutex");

    const uint8_t activeState = current_state.load(std::memory_order_acquire);

    // return if state is None
    if(!verifyState(activeState)) {
        xSemaphoreGive(transitionMutex_);
        return reportError("State is not valid");
    }

    if (arr_states[activeState] == nullptr || arr_states[activeState]->next_state == nullptr) {
        xSemaphoreGive(transitionMutex_);
        return reportError("State next_state callback is not defined");
    }

    try {
        const uint8_t nextState =
            static_cast<uint8_t>(arr_states[activeState]->next_state(buttons));
        current_state.store(nextState, std::memory_order_release);
        recordTransition(activeState, nextState);
    } catch(const std::exception& e) {
        xSemaphoreGive(transitionMutex_);
        // LOGGER erro
        reportError("Error in the next function state machine");
        return reportError(e.what());
    }

    // all is okay
    xSemaphoreGive(transitionMutex_);
    return true;
}