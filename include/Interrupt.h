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
    attachInterrupt(digitalPinToInterrupt(BTN1), Flags::isrFlag0, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN2), Flags::isrFlag1, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN3), Flags::isrFlag2, FALLING);
    attachInterrupt(digitalPinToInterrupt(LEFT), Flags::isrFlag3, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT), Flags::isrFlag4, RISING);

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