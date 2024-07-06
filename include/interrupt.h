#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <Arduino.h>
#include "stateflow.h"

extern states state;
void IRAM_ATTR BTN1_interrupt() {
    Serial.print("AAAAA");
    if (state == STANDBY) {
        state_machine();
    }
}

void IRAM_ATTR BTN2_interrupt() {
    Serial.print("BBBBBBB");
    if (state == POWER_ON) {
        state_machine();
    }
}

void IRAM_ATTR LEFT_interrupt() {
    if (state == CALIBRATION) {
        state_machine();
    }
}

void IRAM_ATTR RIGHT_interrupt() {
    if (state == RUNNING) {
        state_machine();
    }
}

void set_all_interruptions(){
    attachInterrupt(digitalPinToInterrupt(BTN1), BTN1_interrupt, LOW );
    attachInterrupt(digitalPinToInterrupt(BTN2), BTN2_interrupt, LOW );
    attachInterrupt(digitalPinToInterrupt(LEFT), LEFT_interrupt, RISING );
    attachInterrupt(digitalPinToInterrupt(RIGHT), RIGHT_interrupt, RISING );
}


#endif