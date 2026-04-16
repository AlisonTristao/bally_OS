#ifndef PARALLELPROCESSING_H
#define PARALLELPROCESSING_H

// native libraries
#include <Arduino.h>
#include "esp_timer.h"

// header 
#include <Settings.h>
#include <Wrapper.h>

// static libraries
#include <Flags.h>
#include <Logger.h>

// header of the state machine with a sample function
#include <Run.h>

// handle of timer
esp_timer_handle_t timer_get_handle;

// result of timer interrupt
bool result_interrupt = false;

static IRAM_ATTR void sampleISR(void* arg) {
    #if defined(LOG_ALL) || defined(LOG_TELEMETRY)
        if(!(ROBOT::machine.current_state == RUN)) return;
        /*ROBOT::logger.insert_log(
                            String(ROBOT::encoder_left.getCount())    + ";" +
                            String(ROBOT::encoder_right.getCount()),
                            logType::TELEMETRY);*/
    #endif
}

void configure_interruptions(void *param){
    // set the buttons interruptions
    /*attachInterrupt(digitalPinToInterrupt(BTN1), Signals_IN::isrBtn0, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN2), Signals_IN::isrBtn1, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN3), Signals_IN::isrBtn2, FALLING);

    // set the side sensors interruptions
    attachInterrupt(digitalPinToInterrupt(LEFT), Signals_IN::isrsideSensor0, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT), Signals_IN::isrsideSensor1, RISING);*/

    // set the timer interruptions
    #ifdef SAMPLING_ACTIVE
        esp_timer_create_args_t timer_args = {
          .callback = &sampleISR,
          .arg = NULL,
          .name = "timer_get_values"
        };

        // try init the timer interrupt
        result_interrupt = !(esp_timer_create(&timer_args, &timer_get_handle)         != ESP_OK
                        || esp_timer_start_periodic(timer_get_handle, SAMPLE_MICROS)  != ESP_OK);
    #else
        result_interrupt = true;
    #endif

    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("Interruptions configured", logType::INFO);
    #endif

    // delete this task
    vTaskDelete(NULL);
}

bool init_interruptions(){
    // init interruptions on secondary core
    xTaskCreatePinnedToCore(configure_interruptions, "setup_interrupts", 2048, NULL, 2, NULL, SECONDARY_CORE);

    // wait for the task to finish
    uint32_t timer = millis();
    while(!result_interrupt && (millis() - timer) <= 1000)
        delay(1);

    // all ok
    return result_interrupt;
}

#endif