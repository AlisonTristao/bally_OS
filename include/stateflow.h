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
extern int tempo;       // -> Definada no interrupt.h


extern states state; // -> Definida na MAIN


int counter = 0;

void state_machine() {
    if (state == RUNNING) {
        state = POWER_ON; // Manter o estado em STATE4 se já estiver no final
    } else {
        state = static_cast<states>(state + 1); // Incrementar o estado
    }
}


void state_1() {

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

    if (millis() >= tempo + 1000 && (right_flag ||left_flag)) {
        if (left_flag  && !right_flag && counter == 0) {
            counter++;
        } else if  (left_flag  && !right_flag && counter == 1) {
            right_flag = false;
            left_flag = false;
            counter = 0;
            state_machine();
        }

        right_flag = false;
        left_flag = false;
    }

    

    digitalWrite(LED1, LOW);
    digitalWrite(LED2, HIGH);
    Serial.print(" ");
    Serial.print(left_flag);
    Serial.print(" ");    
    Serial.print(right_flag);
	Serial.println(" -> CORRENDO <- ");
}


#endif