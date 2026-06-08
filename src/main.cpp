// standard libraries
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>     
#include <esp_system.h>  
#include <esp_now.h>     

// projetct header
#include <Settings.h>

// external library 
#include <TinyShell.h>
#include <TinyEKF.h>

// custom library
#include <ArraySensor.h>
#include <Encoder.h>
#include <HBridge.h>
#include <Flags.h>
#include <Logger.h>
#include <StateMachine.h>

// main module 
#include <BallyRobot.h>

// robot state machine 
#include <Setup.h>  
#include <Wait.h>
#include <Calibrate.h>
#include <Debug.h>
#include <Run.h>
#include <Finish.h>
#include <Telemetry.h>
#include <Error.h>

// robot instante
ROBOT& robot = ROBOT::getInstance();

// monitor to report system stats
#ifdef ENABLE_SYSTEM_MONITOR
#include <SystemMonitor.h>
SystemMonitor monitor;
static StackType_t xSystemMonitorStack[M4KB];
static StaticTask_t xSystemMonitorBuffer;
#endif

//  STATE MACHINE INSTANCES 
StateMachine state1(SETUP,      setup_function,     next_state_setup);
StateMachine state2(WAIT,       wait_function,      next_state_wait);   
StateMachine state3(CALIBRATE,  calibrate_function, next_state_calibrate);
StateMachine state4(DEBUG,      debug_function,     next_state_debug);
StateMachine state5(RUN,        run_function,       next_state_run);
StateMachine state6(FINISH,     finish_function,    next_state_finish);
StateMachine state7(TELEMETRY,  telemetry_function, next_state_telemetry);
StateMachine state8(ERROR,      error_function,     next_state_error);

// ==============================================================================

static StackType_t xRoutineStack[M8KB];
static StaticTask_t xRoutineBuffer;

static StackType_t xInterruptsStack[M2KB];
static StaticTask_t xInterruptsBuffer;

static StackType_t xShellStack[M8KB];
static StaticTask_t xShellBuffer;

static StackType_t xStateMachineStack[M4KB];
static StaticTask_t xStateMachineBuffer;

// ==============================================================================

extern "C" void app_main(void) {    
    // init static objects and espnow settings
    if(!robot.init()) {
        while (true) {
            // nothing we can do...
            ESP_LOGE("ROBOT_MAIN", "Failed to initialize the robot");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI("ROBOT_MAIN", "Robot initialized successfully");

    // define the callbacks for the logger
    robot.logger.set_send_callback([](const uint8_t *data, size_t len) {
        uint8_t peer_mac[6] = {MAC_ADDR};
        return (esp_now_send(peer_mac, data, len)) == ESP_OK; // send the log message using esp-now
    });

    ESP_LOGI("ROBOT_MAIN", "Logger callbacks configured successfully");

    // define a default callback for the state machine errors, to log the error messages using the logger
    robot.machine.setErrorCallback([](const char* message) {
        if (message != nullptr)
            robot.logger.insert_log(logType::ERRO, message); // log the error message using the logger
    });

    ESP_LOGI("ROBOT_MAIN", "State machine error callback configured successfully");

    // define the callback for the shell output, to log the command outputs using the logger
    robot.shell.set_output_callback([](const std::string& text) {
        if (!text.empty())
            robot.logger.insert_log(logType::DEBG, text.c_str());
    });

    ESP_LOGI("ROBOT_MAIN", "Shell output callback configured successfully");

    // init state machine
    // ATTENTION: the state machine must be initialized after the set the callbacks
    robot.machine.current_state.store(SETUP, std::memory_order_release);

    ESP_LOGI("ROBOT_MAIN", "State machine initialized successfully");
    
    if (!robot.machine.verifyCallbacks()) {
        while (true) {
            // print error message and wait for a while before trying again, to avoid spamming the logs
            robot.logger.insert_log(logType::ERRO, "State machine callbacks are not properly configured");
            ESP_LOGE("ROBOT_MAIN", "State machine callbacks are not properly configured");
            
            // if the parallel processing is not working, 
            // we need to send the logs from here to be able to debug the problem
            robot.logger.flush_logs();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
		// there nothing we can do...
    }

    ESP_LOGI("ROBOT_MAIN", "State machine callbacks verified successfully");

    // init parallel processing into secondary core
    xTaskCreateStaticPinnedToCore(
        robot.routine,         
        "routine",             
        M8KB,               
        NULL,                   
        2,                     
        xRoutineStack,             
        &xRoutineBuffer,           
        PRO_CPU_NUM           
    );

    ESP_LOGI("ROBOT_MAIN", "Routine processing initialized successfully");

    // init interruptions on secondary core
    xTaskCreateStaticPinnedToCore(
        robot.configure_interruptions,
        "setup_interrupts", 
        M2KB,                 
        NULL,   
        2,                               
        xInterruptsStack,       
        &xInterruptsBuffer,       
        PRO_CPU_NUM 
    );

    ESP_LOGI("ROBOT_MAIN", "Interruptions configured successfully");

    // init the shell task 
    xTaskCreateStaticPinnedToCore(
        robot.executeReceivedCommandFromQueue,
        "shell",
        M8KB,
        &robot.shell,
        1,
        xShellStack,
        &xShellBuffer,
        PRO_CPU_NUM
    );

    ESP_LOGI("ROBOT_MAIN", "Shell task initialized successfully");

    xTaskCreateStaticPinnedToCore(
        robot.runStateMachine,
        "state_machine",
        M4KB,
        &robot.machine,
        1,
        xStateMachineStack,
        &xStateMachineBuffer,
        APP_CPU_NUM
    );

    // init the system monitor to report the system stats in the logs
    #ifdef ENABLE_SYSTEM_MONITOR
        monitor.begin();
        monitor.setOutputCallback([](const std::string& data) {
            if (!data.empty())
                robot.logger.insert_log(logType::DEBG, data.c_str());
        });
        monitor.setLoggerCallback([]() {
            return robot.logger.get_write_pct();
        });

        // init the task to report the system stats periodically, every 5 seconds
        xTaskCreateStaticPinnedToCore(
            [](void* param) {
                SystemMonitor* monitor = static_cast<SystemMonitor*>(param);
                while (true) {
                    monitor->update();
                    monitor->report();
                    vTaskDelay(pdMS_TO_TICKS(SYSMON_FREQ_MS)); // report every SYSMON_FREQ_MS milliseconds
                }
            },
            "system_monitor",
            M4KB,
            &monitor,
            1,
            xSystemMonitorStack,
            &xSystemMonitorBuffer,
            PRO_CPU_NUM
        );

        ESP_LOGI("ROBOT_MAIN", "System monitor initialized successfully");
    #endif

    // delete 
    vTaskDelete(NULL);
}