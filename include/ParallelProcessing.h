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

void routine(void *param){
    // delay to stabilize the system (wait checkFlagsDuration() to be ready)
    delay(DELAY_FLAGS);

	// set the filter time for flags 
    ROBOT::buttons.setFilterTime(DELAY_FLAGS); 
    ROBOT::sideSensors.setFilterTime(DELAY_FLAGS);

	// timer of state machine actualization
	uint32_t timer_state_machine = millis();
	
    // log message  
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("Parallel processing initialized", logType::INFO);
    #endif

    // main loop of the parallel processing
	while(true) {	
        // logger print live
        #ifdef LOG_VERBOSE
            ROBOT::logger.send_logger_live();
        #endif

		// check flags duration
		ROBOT::buttons.checkFlagsDuration();
		ROBOT::sideSensors.checkFlagsDuration();
        ROBOT::leds.checkFlagsDuration();
        ROBOT::motors.checkFlagsDuration();

        // keep periodic debug prints without changing state here
		if(millis() - timer_state_machine > DELAY_FLAGS)
			timer_state_machine = millis();

		// sample delay... (wait for the whatchdog to be ready) 
		delay(1);
	}
}

#endif