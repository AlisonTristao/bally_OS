#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <Arduino.h>
#include "stateflow.h"


bool right_flag = false;
bool left_flag = false;
int tempo = 0;

extern states state; // -> Definida na MAIN


void IRAM_ATTR button_isr_handler() {
    digitalWrite(LED0, HIGH);
    if (state == POWER_ON) {
        state_machine();
    }

}

void IRAM_ATTR button_isr_handler2() {
    digitalWrite(LED0, LOW);
    if (state == CALIBRATION) {
        state_machine();
    }
}

void IRAM_ATTR LEFT_interrupt() {
    if (state == RUNNING) {
        left_flag = true;
        tempo = millis();
    }
}

void IRAM_ATTR RIGHT_interrupt() {
    if (state == RUNNING) {
        right_flag = true;
        tempo = millis();
    }
}

void set_all_interruptions(){
    attachInterrupt(digitalPinToInterrupt(BTN1), button_isr_handler, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN2), button_isr_handler2, FALLING);
    attachInterrupt(digitalPinToInterrupt(LEFT), LEFT_interrupt, RISING );
    attachInterrupt(digitalPinToInterrupt(RIGHT), RIGHT_interrupt, RISING );
}


#endif