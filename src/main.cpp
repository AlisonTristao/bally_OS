//  SYSTEM & FRAMEWORK 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>

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

// ROBOT INSTANCE
ROBOT robot;

// callback to print the logger messages in the serial monitor, used when the esp-now is not working
esp_err_t printLoggerSerial(const uint8_t *peer_addr, const uint8_t *data, size_t len) {
	// the peer address is not relevant in this case, so we ignore it
	(void)peer_addr;

	// convert the data to a struct LogMessage 
	message logMessage;
	memcpy(&logMessage, data, sizeof(message));
	logMessage.msg[sizeof(logMessage.msg) - 1] = '\0';
	
	// print the log message in the serial monitor
	Serial.printf("[%u ms] [%s] %s\n", logMessage.timer, 
		(logMessage.type == logType::INFO) ? "INFO" :
		(logMessage.type == logType::CMD) ? "CMD" :
		(logMessage.type == logType::TELEMETRY) ? "TELEMETRY" :
		(logMessage.type == logType::ERROR) ? "ERROR" :
		(logMessage.type == logType::DEBUG) ? "DEBUG" :
		(logMessage.type == logType::PAKG) ? "PAKG" : "NONE",
		logMessage.msg);

	return ESP_OK;
}

void setup() {
	// start serial communication for debuggind when the espnow is not working
	// using USB CDC communication, the baud rate is not relevant
	Serial.begin(3000000); 
	robot.logger.setSendCallback(printLoggerSerial);

	// init static objects and espnow settings
	if(!robot.init()) {
		for (uint8_t i = 0; i < 10; i++) {
			// print error message and wait for a while before trying again, to avoid spamming the logs
			robot.logger.insert_log("Failed to initialize robot", logType::ERROR);

			// if the parallel processing is not working, 
			// we need to send the logs from here to be able to debug the problem
			robot.logger.send_logger_live();
			delay(1000); // wait 10s for the user to see the message before restarting
		}	
		ESP.restart(); // there nothing we can do...
	}

	// define the callbacks for the logger and state machine
	// in this case, the logger will use the esp_now_send function to send the logs, 
	// and the state machine will save the error messages in the logger
	robot.logger.setSendCallback(esp_now_send); // now, the logger send the messages using esp-now
	robot.machine.setErrorCallback(robot.staticInsertLog);

	// init state machine
	// ATTENTION: the state machine must be initialized after the set the callbacks
	robot.machine.current_state.store(SETUP, std::memory_order_release);
	if (!robot.machine.verifyCallbacks()) {
		for (uint8_t i = 0; i < 10; i++) {
			// print error message and wait for a while before trying again, to avoid spamming the logs
			robot.logger.insert_log("State machine callbacks are not properly configured", logType::ERROR);
			
			// if the parallel processing is not working, 
			// we need to send the logs from here to be able to debug the problem
			robot.logger.send_logger_live();
			delay(1000);
		}
		ESP.restart(); // there nothing we can do...
	}

	// init parallel processing into secondary core
	xTaskCreatePinnedToCore(// routine to verify the state machine and send the logs using esp-now
							robot.routine, 				
							"parallel_processing", 
							// 32kb na stack			
							32*1024, 							
							NULL, 						
							// this task needs to be lower priority than interruptions
							3,
							NULL, 	
							// this task run in the secondary core to avoid blocking the main loop						
							SECONDARY_CORE);

    // init interruptions on secondary core
    xTaskCreatePinnedToCore(// task to create the interruptions on the secondary core
							robot.configure_interruptions,
							"setup_interrupts", 
							// 2kb na stack			
							2048, 							
							NULL, 	
							// this task needs to be higher priority than the parallel processing			
							2, 								 
							NULL, 		
							// this task run in the secondary core to avoid blocking the main loop					
							SECONDARY_CORE);				
}

// main loop focused on running the state machine function, 
// the parallel processing is responsible for the rest
void loop() {
	// run state machine
	robot.machine.run();

	// sample delay... (wait for the whatchdog to be ready) 
	vTaskDelay(1/portTICK_PERIOD_MS);
}