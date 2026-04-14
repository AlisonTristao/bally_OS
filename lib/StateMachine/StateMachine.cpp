#include <StateMachine.h>

StateMachine* StateMachine::active_machine_ = nullptr;

StateMachine::StateMachine()
    : current_state(NONE),
      is_controller_(true) {
    active_machine_ = this;
}

StateMachine::StateMachine(name stateValue, name (*actionFn)(), name (*nextStateFn)(uint8_t buttons))
    : current_state(NONE),
      state(stateValue),
      action(actionFn),
      next_state_fn(nextStateFn),
      is_controller_(false) {
    if (active_machine_ != nullptr && static_cast<uint8_t>(state) < NUMBER_OF_STATES) {
        active_machine_->arr_states[static_cast<uint8_t>(state)] = this;
    }
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

uint8_t StateMachine::getValue() const {
    return static_cast<uint8_t>(state);
}

bool StateMachine::run() {
    if (!is_controller_) {
        return false;
    }

    if (current_state >= NUMBER_OF_STATES) {
        reportError("Invalid state index");
        current_state = ERROR;
        return false;
    }

    if (arr_states[current_state] == nullptr) {
        reportError("State not defined");
        current_state = ERROR;
        return false;
    }

    if (stateOutputCallback_ != nullptr) {
        stateOutputCallback_(current_state);
    }

    name actionNextState = NONE;
    try {
        actionNextState = arr_states[current_state]->action();
    } catch (const std::exception& e) {
        reportError("Error in the state machine");
        reportError(e.what());
        current_state = ERROR;
        return false;
    }

    if (actionNextState != static_cast<name>(current_state)) {
        return next(actionNextState);
    }

    name transitionState = static_cast<name>(current_state);
    if (arr_states[current_state]->next_state_fn != nullptr) {
        transitionState = arr_states[current_state]->next_state_fn(inputCallback_());
    }

    if (!next(transitionState)) {
        return false;
    }

    return true;
}

bool StateMachine::next(name nextState) {
    if (!is_controller_) {
        return false;
    }

    if (nextState >= NUMBER_OF_STATES) {
        reportError("Invalid next state");
        current_state = ERROR;
        return false;
    }

    if (arr_states[nextState] == nullptr) {
        reportError("Next state not registered");
        current_state = ERROR;
        return false;
    }

    current_state = static_cast<uint8_t>(nextState);
    return true;
}
