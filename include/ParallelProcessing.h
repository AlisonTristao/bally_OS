#ifndef PARALLELPROCESSING_H
#define PARALLELPROCESSING_H

// native libraries
#include <Arduino.h>
#include "esp_timer.h"

// header 
#include <Settings.h>

// static libraries
#include <Flags.h>
#include <Logger.h>

// header of the state machine with a sample function
#include <Run.h>

// handle of timer
esp_timer_handle_t timer_get_handle;

// result of timer interrupt
bool result_interrupt = false;

void configure_interruptions(void *param){
    // set the buttons interruptions
    attachInterrupt(digitalPinToInterrupt(BTN1), Signals_IN::isrBtn0, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN2), Signals_IN::isrBtn1, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN3), Signals_IN::isrBtn2, FALLING);

    // set the side sensors interruptions
    attachInterrupt(digitalPinToInterrupt(LEFT), Signals_IN::isrsideSensor0, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT), Signals_IN::isrsideSensor1, RISING);

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
        Logger::insert_log("Interruptions configured", logType::INFO);
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
	Signals_IN::setFilterTime(DELAY_FLAGS); 

	// timer of state machine actualization
	uint32_t timer_state_machine = millis();
	
    // log message  
    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::insert_log("Parallel processing initialized", logType::INFO);
    #endif

    // main loop of the parallel processing
	while(true) {	
        // logger print live
        #ifdef LOG_VERBOSE
            Logger::send_logger_register_live();
        #endif

		// check flags duration
		Signals_IN::checkFlagsDuration();

        // set leds according to the state machine
        setLeds(Signals_IN::getLeds());

		// verify if the state machine needs to be updated
		if(millis() - timer_state_machine > DELAY_FLAGS){
			// update state machine
			StateMachine::next(Signals_IN::getButtons());
			timer_state_machine = millis();

            // log message
            #ifdef LOG_DEBUG
                Logger::insert_log("Butoes: " + String(Signals_IN::getButtons()) + "\t" + 
                                    "Sensores laterais: " + String(Signals_IN::getSideSensors()) , logType::INFO);
            #endif
		}
		// sample delay... (wait for the whatchdog to be ready) 
		delay(1);
	}
}

#endif