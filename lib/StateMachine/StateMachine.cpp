#include <StateMachine.h>

std::atomic<uint8_t> StateMachine::current_state{NONE};
SemaphoreHandle_t StateMachine::transitionMutex_ = nullptr;
StateMachine::ErrorCallback StateMachine::errorCallback_ = &StateMachine::defaultErrorCallback;

StateMachine *StateMachine::arr_states[NUMBER_OF_STATES] = {
	NULL,
}; 

StateMachine::StateMachine(stateName state, stateName (*action)(), stateName (*next_state)(uint8_t buttons)){
    this->state = state;
    this->action = action;
    this->next_state = next_state;
    arr_states[number] = this;

    if (transitionMutex_ == nullptr) {
        transitionMutex_ = xSemaphoreCreateMutex();
    }
}

bool StateMachine::verifyState(uint8_t state) const {
    return (state > NONE && state < NUMBER_OF_STATES);
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

    // execute action and use returned state as the next active state
    try {
        current_state.store(arr_states[activeState]->action(), std::memory_order_release);
    } catch(const std::exception& e) {
        xSemaphoreGive(transitionMutex_);
        // LOGGER erro
        reportError("Error in the loop function state machine");
        return reportError(e.what());
    }

    xSemaphoreGive(transitionMutex_);

    // all is okay    
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

    try {
        current_state.store(arr_states[activeState]->next_state(buttons), std::memory_order_release);
    } catch(const std::exception& e) {
        xSemaphoreGive(transitionMutex_);
        // LOGGER erro
        reportError("Error in the next function state machine");
        return reportError(e.what());
    }

    xSemaphoreGive(transitionMutex_);

    // all is okay
    return true;
}