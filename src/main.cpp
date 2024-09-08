// header
#include <Settings.h>
#include <ParallelProcessing.h>

// static libraries
#include <Flags.h>
#include <Logger.h>
#include <StateMachine.h>

// custom libraries
#include <ArraySensor.h>
#include <Encoder.h>
#include <HBridge.h>

// state machine header
#include <Setup.h>  
#include <Wait.h>
#include <Calibrate.h>
#include <Map.h>
#include <Run.h>
#include <Finish.h>
#include <Telemetry.h>
#include <Error.h>

// state machine
StateMachine state1(SETUP, setup_function, next_state_setup);
StateMachine state2(WAIT, wait_function, next_state_wait);	
StateMachine state3(CALIBRATE, calibrate_function, next_state_calibrate);
StateMachine state4(MAP, map_function, next_state_map);
StateMachine state5(RUN, run_function, next_state_run);
StateMachine state6(FINISH, finish_function, next_state_finish);
StateMachine state7(TELEMETRY, telemetry_function, next_state_telemetry);
StateMachine state8(ERROR, error_function, next_state_error);

void setup() {
	// init state machine
	StateMachine::current_state = SETUP;

	// init parallel processing into secondary core
	xTaskCreatePinnedToCore(routine, 				// task function 
							"parallel_processing", 	// task name
							10240, 					// stack size
							NULL, 					// not used
							2, 						// priority
							NULL, 					// not used
							SECONDARY_CORE);		// secondary core
}

// timer to print
uint32_t timer_print = 0;

void loop() {
	// run state machine
	StateMachine::run(); //) StateMachine::current_state = ERROR;

	// sample delay... (wait for the whatchdog to be ready) 
	delay(1);

	if(millis() - timer_print > 500){
		// print the state machine
		Serial.println(StateMachine::current_state);
		timer_print = millis();
	}
} 