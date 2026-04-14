// ==================== SYSTEM & FRAMEWORK ====================
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ==================== PROJECT HEADERS ====================
#include <Settings.h>
#include <ParallelProcessing.h>
#include <Pinout.h>

// ==================== EXTERNAL LIBRARIES ====================
#include <TinyShell.h>

// ==================== CUSTOM MODULES ====================
#include <ArraySensor.h>
#include <Encoder.h>
#include <HBridge.h>
#include <Control.h>

// ==================== UTILITIES ====================
#include <Flags.h>
#include <Logger.h>
#include <StateMachine.h>
#include <StaticObjects.h>

// ==================== ROBOT STATE MACHINE ====================
#include <Setup.h>  
#include <Wait.h>
#include <Calibrate.h>
#include <Debug.h>
#include <Run.h>
#include <Finish.h>
#include <Telemetry.h>
#include <Error.h>

// ==================== STATE MACHINE INSTANCES ====================
StateMachine state1(SETUP, 		setup_function, 	next_state_setup);
StateMachine state2(WAIT, 		wait_function,		next_state_wait);	
StateMachine state3(CALIBRATE, 	calibrate_function, next_state_calibrate);
StateMachine state4(DEBUG, 		debug_function,	next_state_debug);
StateMachine state5(RUN, 		run_function,		next_state_run);
StateMachine state6(FINISH, 	finish_function,	next_state_finish);
StateMachine state7(TELEMETRY, 	telemetry_function, next_state_telemetry);
StateMachine state8(ERROR, 		error_function,	next_state_error);

void setup() {
	// init log register
	ROBOT::logger.insert_log("Welcome! the car is starting...", logType::INFO);

	// init shell
	ROBOT::logger.setSendCallback(send_data);
    ROBOT::logger.setCommandCallback(run_command);

	// set state machine error callback
	ROBOT::machine.setErrorCallback(ROBOT::logStateMachineError);

	// init state machine
	// ATTENTION: the state machine must be initialized after the set this callbacks
	ROBOT::machine.current_state = SETUP;

	// init parallel processing into secondary core
	xTaskCreatePinnedToCore(routine, 				// task function 
							"parallel_processing", 	// task name
							10240, 					// stack size
							NULL, 					// not used
							2, 						// priority
							NULL, 					// not used
							SECONDARY_CORE);		// secondary core

}

void loop() {
	// run state machine
	ROBOT::machine.run();

	// sample delay... (wait for the whatchdog to be ready) 
	delay(1);
}