#include <StateMachine.h>

uint8_t StateMachine::current_state = NONE;

StateMachine *StateMachine::arr_states[NUMBER_OF_STATES] = {
	NULL,
}; 

StateMachine::StateMachine(name state, bool (*action)(), name (*next_state)(uint8_t buttons)){
    this->state = state;
    this->action = action;
    this->next_state = next_state;
    arr_states[number] = this;
}

uint8_t StateMachine::getValue(){
    return number;
}

bool StateMachine::run(){
    // return if function not defined
    if(!arr_states[current_state]){
        // log message
        log_error_state_machine("State not defined");
        return false;
    }

    // try execute function 
    bool result = false;
    try {
        result = arr_states[current_state]->action();
    } catch(const std::exception& e) {
        // LOGGER erro
        log_error_state_machine("Error in the state machine");
        log_error_state_machine(e.what());
        return false;
    }

    if(!result){
        // LOGGER erro
        log_error_state_machine("State function returned false");
        return false;
    }

    // all is okay    
    return result;
}

void StateMachine::next(uint8_t buttons){
    // return if function not defined
    if(!arr_states[current_state]){
        // LOGGER erro
        log_error_state_machine("State not defined");
        return;
    }
    
    current_state = arr_states[current_state]->next_state(buttons);
}


