#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <Arduino.h>
#include <Config.h>
#include <Flags.h>
#include "esp_timer.h"

esp_timer_handle_t timer_get_handle;

static IRAM_ATTR void timer_ISR(void* arg) {
    
}

void IRAM_ATTR configure_interruptions(void *param){
    // set the buttons interruptions
    attachInterrupt(digitalPinToInterrupt(BTN1), Signals_IN::isrBtn0, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN2), Signals_IN::isrBtn1, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN3), Signals_IN::isrBtn2, FALLING);

    // set the side sensors interruptions
    attachInterrupt(digitalPinToInterrupt(LEFT), Signals_IN::isrsideSensor0, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT), Signals_IN::isrsideSensor1, RISING);

    // set the timer interruptions
    esp_timer_create_args_t timer_args = {
      .callback = &timer_ISR,
      .arg = NULL,
      .name = "timer_get_values"
    };
    esp_timer_create(&timer_args, &timer_get_handle);
    esp_timer_start_periodic(timer_get_handle, SAMPLE_MICROS);

    // delete this task
    vTaskDelete(NULL);
}

bool init_interruptions(){
    // init interruptions on secondary core
    try {
        xTaskCreatePinnedToCore(configure_interruptions, "setup_interrupts", 2048, NULL, 2, NULL, SECONDARY_CORE);
        return true;
    } catch (const std::exception& e) {
        // LOGGER ERROR
        return false;
    }
}

#endif