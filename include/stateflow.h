#ifndef STATEFLOW_H
#define STATEFLOW_H


#include <Arduino.h>

enum states {
    POWER_ON,       // estado 1 -> inicial / Fim corrida
    CALIBRATION,    // estado 2 -> Calibrando sensores
    RUNNING         // estado 3 -> Corrida
};

extern states state;
void state_machine() {
    if (state == RUNNING) {
        state = POWER_ON; // Manter o estado em STATE4 se já estiver no final
    } else {
        state = static_cast<states>(state + 1); // Incrementar o estado
    }
}


#endif