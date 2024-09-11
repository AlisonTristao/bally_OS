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
        Logger::IN_LOG("State not defined", logType::ERROR);
        return false;
    }

    // try execute function 
    bool result = false;
    try {
        result = arr_states[current_state]->action();
    } catch(const std::exception& e) {
        // LOGGER erro
        Logger::IN_LOG("Error in the state machine", logType::ERROR);
        Logger::IN_LOG(e.what(), logType::ERROR);
        return false;
    }

    if(!result){
        // LOGGER erro
        Logger::IN_LOG("State function returned false", logType::ERROR);
        return false;
    }

    // all is okay    
    return result;
}

void StateMachine::next(uint8_t buttons){
    // return if function not defined
    if(!arr_states[current_state]){
        // LOGGER erro
        Logger::IN_LOG("State not defined", logType::ERROR);
        return;
    }
    
    current_state = arr_states[current_state]->next_state(buttons);
}


