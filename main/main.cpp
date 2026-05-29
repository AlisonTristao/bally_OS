//  SYSTEM & FRAMEWORK 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <cstring>
#include <inttypes.h>

//  PROJECT HEADERS 
#include <Pinout.h>
#include <SharedMessageTypes.h>

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

static const char* TAG = "main";

static esp_log_level_t logTypeToLevel(logType type) {
	 switch (type) {
		 case logType::ERRO: return ESP_LOG_ERROR;
		 case logType::WARN: return ESP_LOG_WARN;
		 case logType::DEBG: return ESP_LOG_DEBUG;
		 case logType::CMDO: return ESP_LOG_INFO;
		 case logType::INFO: return ESP_LOG_INFO;
		 case logType::NONE: return ESP_LOG_INFO;
		 default: return ESP_LOG_INFO;
	 }
}

// callback to print the logger messages in the serial monitor, used when the esp-now is not working
bool printLoggerSerial(const uint8_t *data, size_t len) {
	// convert the data to a struct LogMessage 
	message logMessage;
	std::memcpy(&logMessage, data, sizeof(message));
	logMessage.content.text[sizeof(logMessage.content.text) - 1] = '\0';
	
	// print the log message in the serial monitor
	esp_log_level_t level = logTypeToLevel(logMessage.type);
	ESP_LOG_LEVEL(level, TAG, "[%" PRIu32 " ms] [%s] %s", logMessage.timer, logTypeToString(logMessage.type), logMessage.content.text);

	return true;
}


static void robot_setup() {
	// initialize NVS required by WiFi/ESP-NOW and Preferences replacement
	esp_err_t nvs_err = nvs_flash_init();
	if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		nvs_flash_erase();
		nvs_err = nvs_flash_init();
	}
	if (nvs_err != ESP_OK) {
		ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(nvs_err));
	}

	// set callback to print the logger messages when esp-now is not working
	robot.logger.set_send_callback(printLoggerSerial);

	// init static objects and espnow settings
	if(!robot.init()) {
		for (uint8_t i = 0; i < 10; i++) {
			// print error message and wait for a while before trying again, to avoid spamming the logs
			robot.logger.insert_log(logType::ERRO, "Failed to initialize robot");

			// if the parallel processing is not working, 
			// we need to send the logs from here to be able to debug the problem
			robot.logger.flush_logs();
			vTaskDelay(pdMS_TO_TICKS(1000)); // wait 1s for the user to see the message before restarting
		}	
		esp_restart(); // there nothing we can do...
	}

	// define the callbacks for the logger and state machine
	// in this case, the logger will use the esp_now_send function to send the logs, 
	// and the state machine will save the error messages in the logger
	//robot.logger.set_send_callback(esp_now_send); // now, the logger send the messages using esp-now
	robot.machine.setErrorCallback(robot.staticInsertLog);

	// init state machine
	// ATTENTION: the state machine must be initialized after the set the callbacks
	robot.machine.current_state.store(SETUP, std::memory_order_release);
	if (!robot.machine.verifyCallbacks()) {
		for (uint8_t i = 0; i < 10; i++) {
			// print error message and wait for a while before trying again, to avoid spamming the logs
			robot.logger.insert_log( logType::ERRO, "State machine callbacks are not properly configured");
			
			// if the parallel processing is not working, 
			// we need to send the logs from here to be able to debug the problem
			robot.logger.flush_logs();
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
		esp_restart(); // there nothing we can do...
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


static void robot_loop_task(void* param) {
	(void)param;
	for (;;) {
		// run state machine
		robot.machine.run();

		// sample delay... (wait for the whatchdog to be ready) 
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

extern "C" void app_main(void) {
	robot_setup();
	xTaskCreatePinnedToCore(robot_loop_task, "main_loop", 4096, NULL, 3, NULL, PRIMARY_CORE);
}
