//  SYSTEM & FRAMEWORK 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

//  PROJECT HEADERS 
#include <Pinout.h>

//  EXTERNAL LIBRARIES 
#include <TinyShell.h>

//  CUSTOM MODULES 
#include <ArraySensor.h>
#include <Encoder.h>
#include <HBridge.h>
#include <Control.h>

//  UTILITIES 
#include <Flags.h>
#include <Logger.h>
#include <StateMachine.h>
#include <StaticObjects.h>

//  ROBOT STATE MACHINE 
#include <Setup.h>  
#include <Wait.h>
#include <Calibrate.h>
#include <Debug.h>
#include <Run.h>
#include <Finish.h>
#include <Telemetry.h>
#include <Error.h>

//  STATE MACHINE INSTANCES 
StateMachine state1(SETUP, 		setup_function, 	next_state_setup);
StateMachine state2(WAIT, 		wait_function,		next_state_wait);	
StateMachine state3(CALIBRATE, 	calibrate_function, next_state_calibrate);
StateMachine state4(DEBUG, 		debug_function,		next_state_debug);
StateMachine state5(RUN, 		run_function,		next_state_run);
StateMachine state6(FINISH, 	finish_function,	next_state_finish);
StateMachine state7(TELEMETRY, 	telemetry_function, next_state_telemetry);
StateMachine state8(ERROR, 		error_function,		next_state_error);

void setup() {
	// init static objects
	ROBOT::init();

	// define the callbacks for the logger and state machine
	ROBOT::logger.setSendCallback(esp_now_send);
	ROBOT::machine.setErrorCallback(ROBOT::logStateMachineError);

	// init state machine
	// ATTENTION: the state machine must be initialized after the set this callbacks
	ROBOT::machine.current_state.store(SETUP, std::memory_order_release);
	if (!ROBOT::machine.verifyCallbacks()) {
		ROBOT::logger.insert_log("State machine callbacks are not fully configured", logType::ERROR);
		while (true) delay(1000);
	}

	// init parallel processing into secondary core
	xTaskCreatePinnedToCore(ROBOT::routine, 				// task function 
							"parallel_processing", 			// task name
							10240, 							// stack size
							NULL, 							// not used
							3, 								// priority
							NULL, 							// not used
							SECONDARY_CORE);				// secondary core

    // init interruptions on secondary core
    xTaskCreatePinnedToCore(ROBOT::configure_interruptions, // task function
							"setup_interrupts", 			// task name
							2048, 							// stack size
							NULL, 							// not used
							2, 								// priority 
							NULL, 							// not used
							SECONDARY_CORE);				// secondary core*/
}

void loop() {
	// run state machine
	ROBOT::machine.run();

	// sample delay... (wait for the whatchdog to be ready) 
	delay(10);
}