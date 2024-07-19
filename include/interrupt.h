#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <Arduino.h>
#include <pins.h>
#include "esp_timer.h"

esp_timer_handle_t timer_get_handle;

void IRAM_ATTR btn1_ISR() {
    
}

void IRAM_ATTR btn2_ISR() {

}

void IRAM_ATTR btn3_ISR() {

}

void IRAM_ATTR left_ISR() {

}

void IRAM_ATTR right_ISR() {

}


static void timer_ISR(void* arg) {

}

void set_all_interruptions(){
    attachInterrupt(digitalPinToInterrupt(BTN1), btn1_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN2), btn2_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN3), btn3_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(LEFT), left_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT), right_ISR, RISING);
    
    
    esp_timer_create_args_t timer_args = {
      .callback = &timer_ISR,
      .arg = NULL,
      .name = "timer_get_values"
    };
    esp_timer_create(&timer_args, &timer_get_handle);

    // Inicie o timer (intervalo em microsegundos)
    esp_timer_start_periodic(timer_get_handle, TIMER_SAMPLE_MS*1000);
}


#endif