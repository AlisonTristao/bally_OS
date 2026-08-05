// standard libraries
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_now.h>
#include <esp_ota_ops.h>

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
#include <StatesManager.h>

ROBOT& robot = ROBOT::getInstance();
States& states = States::getInstance();

#ifdef ENABLE_SYSTEM_MONITOR
static StackType_t xSystemMonitorStack[M4KB];
static StaticTask_t xSystemMonitorBuffer;
static void init_system_monitor();
#endif

static StackType_t xRoutineStack[M8KB];
static StaticTask_t xRoutineBuffer;
static StackType_t xShellStack[M8KB];
static StaticTask_t xShellBuffer;
static StackType_t xStateMachineStack[M8KB];
static StaticTask_t xStateMachineBuffer;
static StackType_t xEKFStack[M2KB];
static StaticTask_t xEKFBuffer;
static StackType_t xInterruptsStack[M2KB];
static StaticTask_t xInterruptsBuffer;

static void setup_system_callbacks();
static void start_freertos_tasks();

extern "C" void app_main(void) {  
    // initialize the robot, configure the pins, the wifi and the esp-now settings  
    while (!robot.init()) {
        ESP_LOGE("ROBOT_MAIN", "Failed to initialize the robot");
        vTaskDelay(WDOG_TIMEOUT_TK);
    }

    // Reaching this point means SD card, ESP-NOW, encoders and the rest of
    // robot.init() all came up correctly. Confirm the running OTA image is
    // healthy so the bootloader does not roll it back on the next boot.
    esp_ota_mark_app_valid_cancel_rollback();

    ESP_LOGI("ROBOT_MAIN", "Robot initialized successfully");

    // configure the system callbacks for the logger, state machine, shell, etc.
    setup_system_callbacks();
    
    // start the FreeRTOS tasks for the robot's operation
    start_freertos_tasks();

    // initialize the system monitor if enabled
    #ifdef ENABLE_SYSTEM_MONITOR
    init_system_monitor();
    #endif

    vTaskDelete(NULL); // goodbye main app
}

static void setup_system_callbacks() {
    // configure the logger to send log messages via ESP-NOW using a callback function
    // if you want to change the output method of the logger, 
    // you can change this callback to use a different method (e.g., serial, network, etc.)
    robot.logger.set_send_callback([](const uint8_t *data, size_t len) {
        static const uint8_t peer_mac[6] = {MAC_ADDR};
        return (esp_now_send(peer_mac, data, len)) == ESP_OK;
    });

    // configure the state machine to log errors using the logger's insert_log method
    // if an error occurs in the state machine, it will call the error callback function, 
    // which will log the error message using the logger
    robot.machine.setErrorCallback([](const char* message) {
        // need to use the esp log, because the logger dont work if the state machine is not properly configured
        if (message != nullptr) ESP_LOGE("STATE_MACHINE", "%s", message); 
    });

    // Retain normal shell output in PSRAM. While USB owns the SD card, or just
    // after a successful SD flush, send the response directly through ESP-NOW
    // so the response itself does not make the retained buffer non-empty again.
    robot.shell.set_output_callback([](const std::string& text) {
        if (text.empty()) return;

        if (robot.usb_storage.is_exposed() ||
            robot.consumeDirectShellOutputRequest()) {
            robot.logger.send_log_direct(logType::DEBG, text.c_str());
        } else {
            robot.logger.insert_log(logType::DEBG, text.c_str());
        }
    });

    // start the state machine in the SETUP state
    robot.machine.current_state.store(SETUP, std::memory_order_release);

    // verify that all the callbacks for the state machine are properly configured before starting the tasks
    // it is important to ensure that the state machine is properly configured to avoid errors during runtime,
    // if the callbacks are not properly configured, the system will log an error message and halt in an infinite loop
    while (!robot.machine.verifyCallbacks()) {
        ESP_LOGE("ROBOT_MAIN", "State machine callbacks are not properly configured");
        vTaskDelay(WDOG_TIMEOUT_TK);
    }

    ESP_LOGI("ROBOT_MAIN", "System callbacks and state machine configured successfully");
}

static void start_freertos_tasks() {
    xTaskCreateStaticPinnedToCore(robot.routine,           "routine",       M8KB, NULL, 3,  xRoutineStack,      &xRoutineBuffer,      PRO_CPU_NUM);
    xTaskCreateStaticPinnedToCore(robot.initInterruptions, "interrupts",    M2KB, NULL, 0,  xInterruptsStack,   &xInterruptsBuffer,   PRO_CPU_NUM);
    xTaskCreateStaticPinnedToCore(robot.runStateMachine,   "state_machine", M8KB, NULL, 10, xStateMachineStack, &xStateMachineBuffer, APP_CPU_NUM);
    xTaskCreateStaticPinnedToCore(robot.runShell,          "shell_task",    M8KB, NULL, 2,  xShellStack,        &xShellBuffer,        PRO_CPU_NUM);
    xTaskCreateStaticPinnedToCore(robot.runEKF,            "EKF_task",      M2KB, NULL, 4,  xEKFStack,          &xEKFBuffer,          PRO_CPU_NUM);
    ESP_LOGI("ROBOT_MAIN", "All FreeRTOS tasks started successfully");
}

#ifdef ENABLE_SYSTEM_MONITOR
static void init_system_monitor() {
    // robot.sysmon is already configured (begin()/callbacks) by robot.init();
    // this task only drives its periodic report.
    xTaskCreateStaticPinnedToCore(
        [](void* param) {
            (void)param;
            while (true) {
                robot.sysmon.update();
                robot.sysmon.report();
                vTaskDelay(pdMS_TO_TICKS(robot.settings.data().sysmon_freq_ms));
            }
        }, "system_monitor", M4KB, NULL, 1, xSystemMonitorStack, &xSystemMonitorBuffer, PRO_CPU_NUM
    );
    ESP_LOGI("ROBOT_MAIN", "System monitor initialized successfully");
}
#endif
