#include <StateMachine.h>

uint8_t StateMachine::current_state = NONE;
StateMachine::ErrorCallback StateMachine::errorCallback_ = &StateMachine::defaultErrorCallback;

StateMachine *StateMachine::arr_states[NUMBER_OF_STATES] = {
	NULL,
}; 

StateMachine::StateMachine(stateName state, stateName (*action)(), stateName (*next_state)(uint8_t buttons)){
    this->state = state;
    this->action = action;
    this->next_state = next_state;
    arr_states[number] = this;
}

bool StateMachine::verifyState() {
    return (current_state > NONE && current_state < NUMBER_OF_STATES);
}

void StateMachine::defaultErrorCallback(const char* message) {
    (void)message;
    current_state = ERROR;
}

void StateMachine::setErrorCallback(ErrorCallback callback) {
    if (callback) {
        errorCallback_ = callback;
    } else {
        errorCallback_ = defaultErrorCallback;
        current_state = ERROR;
    }
}

bool StateMachine::reportError(const char* message) {
    if (errorCallback_ != nullptr)
        errorCallback_(message);

    current_state = ERROR;
    return false;
}

uint8_t StateMachine::getValue(){
    return number;
}

bool StateMachine::run(){
    // return if state is None
    if(!verifyState())
        return reportError("State is not valid");

    // execute action and use returned state as the next active state
    stateName next_state = NONE;
    try {
        next_state = arr_states[current_state]->action();
    } catch(const std::exception& e) {
        // LOGGER erro
        reportError("Error in the state machine");
        return reportError(e.what());
    }

    if(next_state <= NONE || next_state >= NUMBER_OF_STATES)
        return reportError("State function returned an invalid state");

    current_state = next_state;

    // all is okay    
    return true;
}

void StateMachine::next(uint8_t buttons){
    try {
        current_state = arr_states[current_state]->next_state(buttons);
    } catch(const std::exception& e) {
        // LOGGER erro
        reportError("Error in the state machine");
        reportError(e.what());
        return;
    }
}