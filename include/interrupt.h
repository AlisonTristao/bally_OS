#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <Arduino.h>
#include "stateflow.h"

extern states state;
void IRAM_ATTR button_isr_handler() {
    digitalWrite(LED0, HIGH);
    if (state == STANDBY) {
        state_machine();
    }

}

void IRAM_ATTR button_isr_handler2() {
    digitalWrite(LED0, LOW);
    if (state == POWER_ON) {
        state_machine();
    }
}

void IRAM_ATTR LEFT_interrupt() {
    digitalWrite(LED3, HIGH);
    if (state == CALIBRATION) {
        state_machine();
    }
}

void IRAM_ATTR RIGHT_interrupt() {
    digitalWrite(LED3, LOW);
    if (state == RUNNING) {
        state_machine();
    }
}

void set_all_interruptions(){
    attachInterrupt(digitalPinToInterrupt(BTN1), button_isr_handler, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN2), button_isr_handler2, FALLING);
    attachInterrupt(digitalPinToInterrupt(LEFT), LEFT_interrupt, RISING );
    attachInterrupt(digitalPinToInterrupt(RIGHT), RIGHT_interrupt, RISING );
}


#endif