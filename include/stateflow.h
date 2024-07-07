#ifndef STATEFLOW_H
#define STATEFLOW_H


#include <Arduino.h>

enum states {
    POWER_ON,       // estado 1 -> inicial / Fim corrida
    CALIBRATION,    // estado 2 -> Calibrando sensores
    RUNNING         // estado 3 -> Corrida
};


extern bool right_flag; // -> Definada no interrupt.h
extern bool left_flag;  // -> Definada no interrupt.h

extern states state; // -> Definida na MAIN


void state_machine() {
    if (state == RUNNING) {
        state = POWER_ON; // Manter o estado em STATE4 se já estiver no final
    } else {
        state = static_cast<states>(state + 1); // Incrementar o estado
    }
}


void state_1() {

    right_flag = false;
    left_flag = false;

    digitalWrite(LED2, LOW);
    digitalWrite(LED0, HIGH);
    Serial.println(" -> PARADO <- ");
}

void state_2() {
    digitalWrite(LED0, LOW);
    digitalWrite(LED1, HIGH);
	Serial.println(" -> CALIBRANDO <-");
}

void state_3() {
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, HIGH);
	Serial.println(" -> CORRENDO <- ");
}


#endif