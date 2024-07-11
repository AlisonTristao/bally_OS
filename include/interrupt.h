#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <vector>
#include <Arduino.h>
#include "stateflow.h"
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <ESP32Encoder.h>

using namespace std;

bool right_flag = false;
bool left_flag = false;
int tempo = 0;

extern states state; // -> Definida na MAIN
/*extern vector<double> speedD;
extern vector<double> speedE;
extern vector<double> s_d;
extern ESP32Encoder encoderE;
extern ESP32Encoder encoderD;*/

void IRAM_ATTR button_isr_handler() {
    digitalWrite(LED0, HIGH);
    if (state == POWER_ON) {
        state_machine();
    } else if (state == DEBUG) { 
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
    attachInterrupt(digitalPinToInterrupt(RIGHT), LEFT_interrupt, RISING); // -- inertido
    attachInterrupt(digitalPinToInterrupt(LEFT), RIGHT_interrupt, RISING);
    //const TickType_t timer_period = pdMS_TO_TICKS(100);
    //TimerHandle_t time_ = xTimerCreate("amostra", timer_period, pdTRUE, 0, timer_interrupt);
}


#endif