//  standard libraries
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>     
#include <esp_system.h>  
#include <esp_now.h>     

// projetct header
#include <Settings.h>

//  external library 
#include <TinyShell.h>
#include <TinyEKF.h>

//  custom library
#include <ArraySensor.h>
#include <Encoder.h>
#include <HBridge.h>
#include <Flags.h>
#include <Logger.h>
#include <StateMachine.h>

// main module 
#include <BallyRobot.h>

//  robot state machine 
#include <Setup.h>  
#include <Wait.h>
#include <Calibrate.h>
#include <Debug.h>
#include <Run.h>
#include <Finish.h>
#include <Telemetry.h>
#include <Error.h>

//  robot instante
ROBOT& robot = ROBOT::getInstance();

// main tag for logging
static const char* TAG = "ROBOT_MAIN";

//  STATE MACHINE INSTANCES 
StateMachine state1(SETUP,      setup_function,     next_state_setup);
StateMachine state2(WAIT,       wait_function,      next_state_wait);   
StateMachine state3(CALIBRATE,  calibrate_function, next_state_calibrate);
StateMachine state4(DEBUG,      debug_function,     next_state_debug);
StateMachine state5(RUN,        run_function,       next_state_run);
StateMachine state6(FINISH,     finish_function,    next_state_finish);
StateMachine state7(TELEMETRY,  telemetry_function, next_state_telemetry);
StateMachine state8(ERROR,      error_function,     next_state_error);

extern "C" void app_main(void) {    
    // init static objects and espnow settings
    if(!robot.init()) {
        while (true) {
            // nothing we can do...
            ESP_LOGE(TAG, "Failed to initialize the robot");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // define the callbacks for the logger
    robot.logger.set_send_callback([](const uint8_t *data, size_t len) {
        uint8_t peer_mac[6] = {MAC_ADDR};
        return (esp_now_send(peer_mac, data, len)) == ESP_OK; // send the log message using esp-now
    });

    // define a default callback for the state machine errors, to log the error messages using the logger
    robot.machine.setErrorCallback([](const char* message) {
        if (message != nullptr)
            robot.logger.insert_log(logType::ERRO, message); // log the error message using the logger
    });

    // define the callback for the shell output, to log the command outputs using the logger
    robot.shell.set_output_callback([](const std::string& text) {
        if (!text.empty())
            robot.logger.insert_log(logType::DEBG, text.c_str());
    });

    // init state machine
    // ATTENTION: the state machine must be initialized after the set the callbacks
    robot.machine.current_state.store(SETUP, std::memory_order_release);
    
    if (!robot.machine.verifyCallbacks()) {
        while (true) {
            // print error message and wait for a while before trying again, to avoid spamming the logs
            robot.logger.insert_log(logType::ERRO, "State machine callbacks are not properly configured");
            
            // if the parallel processing is not working, 
            // we need to send the logs from here to be able to debug the problem
            robot.logger.flush_logs();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
		// there nothing we can do...
    }

    // init parallel processing into secondary core
    xTaskCreatePinnedToCore(
        robot.routine,              
        "parallel_processing", 
        16*1024, // 16kb               
        NULL,                       
        2,
        NULL,   
        APP_CPU_NUM 
    );

    // init interruptions on secondary core
    xTaskCreatePinnedToCore(
        robot.configure_interruptions,
        "setup_interrupts", 
        2*1024, // 2kb                      
        NULL,   
        2,                               
        NULL,       
        APP_CPU_NUM 
    );

    
    // main loop focused on running the state machine function, 
    // the parallel processing is responsible for the rest
    while (true) {
        // run state machine
        robot.machine.run();
		// need to add a small delay to avoid blocking the CPU and allow other tasks to run
        vTaskDelay(pdMS_TO_TICKS(WDOG_TIMEOUT_MS));
    }
}