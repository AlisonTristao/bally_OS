#include <StateMachine.h>

uint8_t StateMachine::current_state = NONE;
StateMachine::ErrorCallback StateMachine::errorCallback_ = StateMachine::defaultErrorCallback;
StateMachine::StateOutputCallback StateMachine::stateOutputCallback_ = nullptr;
StateMachine::InputCallback StateMachine::inputCallback_ = StateMachine::defaultInputCallback;

StateMachine *StateMachine::arr_states[NUMBER_OF_STATES] = {
	NULL,
}; 

StateMachine::StateMachine(name state, name (*action)(), name (*next_state)(uint8_t buttons)){
    this->state = state;
    this->action = action;
    this->next_state = next_state;
    arr_states[number] = this;
}

void StateMachine::setErrorCallback(ErrorCallback callback) {
    errorCallback_ = (callback != nullptr) ? callback : defaultErrorCallback;
}

void StateMachine::defaultErrorCallback(const char* message) {
    (void)message;
}

uint8_t StateMachine::defaultInputCallback() {
    return 0;
}

void StateMachine::reportError(const char* message) {
    if (errorCallback_ != nullptr) {
        errorCallback_(message);
    }
}

uint8_t StateMachine::getValue(){
    return number;
}

bool StateMachine::run(){
    if (current_state >= NUMBER_OF_STATES) {
        reportError("Invalid state index");
        current_state = ERROR;
        return false;
    }

    // return if function not defined
    if(!arr_states[current_state]){
        reportError("State not defined");
        current_state = ERROR;
        return false;
    }

    if (stateOutputCallback_ != nullptr) {
        stateOutputCallback_(current_state);
    }

    // try execute function 
    name actionNextState = NONE;
    try {
        actionNextState = arr_states[current_state]->action();
    } catch(const std::exception& e) {
        reportError("Error in the state machine");
        reportError(e.what());
        current_state = ERROR;
        return false;
    }

    // Action has priority when requesting immediate transition.
    if (actionNextState != static_cast<name>(current_state)) {
        return next(actionNextState);
    }

    // When action keeps current state, use input-based transition if available.
    name transitionState = static_cast<name>(current_state);
    if (arr_states[current_state]->next_state != nullptr) {
        transitionState = arr_states[current_state]->next_state(inputCallback_());
    }

    if (!next(transitionState)) {
        return false;
    }

    // all is okay
    return true;
}

bool StateMachine::next(name next_state) {
    if (next_state >= NUMBER_OF_STATES) {
        reportError("Invalid next state");
        current_state = ERROR;
        return false;
    }

    if (arr_states[next_state] == nullptr) {
        reportError("Next state not registered");
        current_state = ERROR;
        return false;
    }

    current_state = static_cast<uint8_t>(next_state);
    return true;
}


