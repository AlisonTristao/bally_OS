#include <StateMachine.h>

uint8_t StateMachine::current_state = 0;

StateMachine *StateMachine::arr_states[NUMBER_OF_STATES] = {
	NULL,
}; 

StateMachine::StateMachine(name state, bool (*action)()){
    this->state = state;
    this->action = action;
    arr_states[number] = this;
}

uint8_t StateMachine::getValue(){
    return number;
}

bool StateMachine::run(){
    if(!arr_states[current_state]) return false;
    arr_states[current_state]->action();
    return true;
}

