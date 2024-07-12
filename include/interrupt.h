#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <vector>
#include <Arduino.h>
#include "stateflow.h"
#include "esp_timer.h"
#include <ESP32Encoder.h>

using namespace std;

bool right_flag = false;
bool left_flag = false;
int tempo = 0;

extern ESP32Encoder encoderD;
extern ESP32Encoder encoderE;
// sen_E -> ta no proprio codigo, não precisa de extern
// sen_D -> ta no proprio codigo, não precisa de extern
extern int16_t velRight;
extern int16_t velLeft;
extern uint32_t cont;
extern double pid0;
extern double position;

extern states state; // -> Definida na MAIN
extern vector<double> countD, countE, sen_E, sen_D, PWM_D, PWM_E, timer_gabriel, ctrl, erro;
esp_timer_handle_t timer_get_handle;

void IRAM_ATTR button_isr_handler() {
    digitalWrite(LED0, HIGH);
    if (state == POWER_ON || state == DEBUG) { 
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


static void timer_get_values(void* arg) {
    if(!timer_flag) return;
    countD.push_back(encoderD.getCount());
    countE.push_back(encoderE.getCount());
    sen_E.push_back(right_flag);
    sen_D.push_back(left_flag);
    PWM_D.push_back(velRight);
    PWM_E.push_back(velLeft);
    timer_gabriel.push_back(cont++);
    ctrl.push_back(pid0);
    erro.push_back(position);  
}

void set_all_interruptions(){
    attachInterrupt(digitalPinToInterrupt(BTN1), button_isr_handler, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN2), button_isr_handler2, FALLING);
    attachInterrupt(digitalPinToInterrupt(LEFT), LEFT_interrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT), RIGHT_interrupt, RISING);
    
    
    esp_timer_create_args_t timer_args = {
      .callback = &timer_get_values,
      .arg = NULL,
      .name = "timer_get_values"
    };
    esp_timer_create(&timer_args, &timer_get_handle);

    // Inicie o timer (intervalo em microsegundos)
    esp_timer_start_periodic(timer_get_handle, TIMER_SAMPLE_MS*1000);
}


#endif