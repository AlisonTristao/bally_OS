#ifndef STATEFLOW_H
#define STATEFLOW_H


#include <Arduino.h>

// estado 1 -> inicial
// estado 2 -> Calibrando sensores
// estado 3 -> Corrida
// estado 4 -> Fim corrida

enum states {
    POWER_ON,
    CALIBRATION,
    RUNNING,
    STANDBY
};

extern states state;
void state_machine() {
    if (state == STANDBY) {
        state = POWER_ON; // Manter o estado em STATE4 se já estiver no final
    } else {
        state = static_cast<states>(state + 1); // Incrementar o estado
    }
}


#endif