// standard libraries
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_system.h>
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

// the link: BTPDevice, configured by the files in src/btp/
#include <BtpDevice.h>
#include "btp/btp_robot.h"

ROBOT& robot = ROBOT::getInstance();
States& states = States::getInstance();

// Filled from settings.conf after robot.init(), before device.begin().
static btpdev::DeviceConfig device_config;
static btpdev::StorageConfig storage_config;
static BtpDevice device(device_config, storage_config);

static StackType_t xRoutineStack[M8KB];
static StaticTask_t xRoutineBuffer;
static StackType_t xStateMachineStack[M8KB];
static StaticTask_t xStateMachineBuffer;
static StackType_t xEKFStack[M2KB];
static StaticTask_t xEKFBuffer;
// interrupts and junkebox keep their stacks in PSRAM (psramStack() below),
// not in .bss: internal RAM is what the BLE stack needs (NimBLE's host task
// alone wants an 8 KB contiguous block -- without it the robot never
// advertised), and these two tasks are neither timing-critical nor ever
// touch the flash (a task with a PSRAM stack must not: NVS, partition reads
// and OTA disable the cache). interrupts only arms GPIO ISRs, which run on
// the ISR stack; junkebox drives LEDC and reads songs off the SD card.
// Their TCBs stay internal.
static StackType_t* xInterruptsStack = nullptr;
static StaticTask_t xInterruptsBuffer;
// SD-backed playback enters stdio/VFS/FatFs/SDSPI before the first note.
// That call chain needs materially more stack than compiled-in playback,
// which never leaves Junkebox's parser. 2 KiB was enough for builtins but
// could trip the FreeRTOS stack canary as soon as play_file() called fopen().
static StackType_t* xJunkeboxStack = nullptr;
static StaticTask_t xJunkeboxBuffer;

// A task stack in PSRAM, falling back to internal RAM if PSRAM is missing.
static StackType_t* psramStack(size_t bytes) {
    void* stack = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (stack == nullptr) {
        ESP_LOGW("ROBOT_MAIN", "no PSRAM for a %u-byte task stack, using internal RAM",
                 static_cast<unsigned>(bytes));
        stack = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    return static_cast<StackType_t*>(stack);
}

static void start_device();
static void setup_system_callbacks();
static void start_freertos_tasks();

extern "C" void app_main(void) {  
    // initialize the robot: pins, SD card, settings, sensors and actuators
    while (!robot.init()) {
        ESP_LOGE("ROBOT_MAIN", "Failed to initialize the robot");
        vTaskDelay(WDOG_TIMEOUT_TK);
    }

    // Reaching this point means SD card, encoders and the rest of
    // robot.init() all came up correctly. Confirm the running OTA image is
    // healthy so the bootloader does not roll it back on the next boot.
    esp_ota_mark_app_valid_cancel_rollback();

    ESP_LOGI("ROBOT_MAIN", "Robot initialized successfully");

    // the link, the terminal, the key and the telemetry
    start_device();

    // configure the state machine's callbacks and initial state
    setup_system_callbacks();
    
    // start the FreeRTOS tasks for the robot's operation
    start_freertos_tasks();

    vTaskDelete(NULL); // goodbye main app
}

static void start_device() {
    // After init(): the name, the link (comm_mode or a boot into OTA) and the
    // SD card all come from it. begin() registers the robot's shell modules
    // and topics (src/btp/), imports bally.key from the card and starts the
    // link and BtpDevice's comms and shell tasks. A failure is logged, not
    // fatal: the robot still drives, just unreachable.
    robot_device_config(robot, device_config);
    robot_storage_config(robot, storage_config);
    const esp_err_t err = device.begin(robot_commands, robot_topics);
    if (err != ESP_OK) {
        ESP_LOGE("ROBOT_MAIN", "BtpDevice: %s (%s)", device.last_error().message,
                 btpdev::err_name(err));
    }
    robot.attachDevice(device, err == ESP_OK);
}

static void setup_system_callbacks() {
    // configure the state machine to log errors using the logger's insert_log method
    // if an error occurs in the state machine, it will call the error callback function,
    // which will log the error message using the logger
    //
    // Also goes through robot.logger, not just ESP_LOGE: by the time this
    // callback can even be registered, robot.init() (which calls
    // logger.begin() as one of its first steps) has already returned, so the
    // logger is ready for every reportError() call this can ever receive --
    // including the ones raised later during normal operation (StateMachine::
    // run()/next() catching an exception thrown from a state's own action),
    // not just the verifyCallbacks() check right below. ESP_LOGE alone left
    // those invisible to anyone not physically wired to the UART.
    robot.machine.setErrorCallback([](const char* message) {
        if (message == nullptr) return;
        ESP_LOGE("STATE_MACHINE", "%s", message);
        robot.logger.insert_log(logType::ERRO, message);
    });

    // Start the state machine in SETUP, unless a boot-time button chord
    // picked OTA/USB storage (DEBUG) or COMM_CONFIG (see ROBOT::bootState()).
    robot.machine.current_state.store(robot.bootState(), std::memory_order_release);

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
    // Priority order on PRO_CPU_NUM (core 0 — core 1/APP_CPU_NUM is reserved
    // for state_machine alone, see CONTRIBUTING.md): EKF (4) = BtpDevice's
    // comms (4) > routine (3) = BtpDevice's shell (3) > junkebox (1) >
    // interrupts (0). BtpDevice started its two tasks in begin() (see
    // src/btp/btp_device.cpp). junkebox sits just above interrupts because
    // note timing only needs to be roughly on-beat, not exact — it should
    // never delay EKF sampling, motor routine upkeep or shell command
    // handling.

    xInterruptsStack = psramStack(M2KB);
    xJunkeboxStack = psramStack(M4KB);
    if (xInterruptsStack == nullptr || xJunkeboxStack == nullptr) {
        ESP_LOGE("ROBOT_MAIN", "no memory for the interrupts/junkebox task stacks");
    }
    xTaskCreateStaticPinnedToCore(robot.routine,           "routine",       M8KB, NULL, 3,  xRoutineStack,      &xRoutineBuffer,      PRO_CPU_NUM);
    xTaskCreateStaticPinnedToCore(robot.initInterruptions, "interrupts",    M2KB, NULL, 0,  xInterruptsStack,   &xInterruptsBuffer,   PRO_CPU_NUM);
    xTaskCreateStaticPinnedToCore(robot.runStateMachine,   "state_machine", M8KB, NULL, 10, xStateMachineStack, &xStateMachineBuffer, APP_CPU_NUM);
    xTaskCreateStaticPinnedToCore(robot.runEKF,            "EKF_task",      M2KB, NULL, 4,  xEKFStack,          &xEKFBuffer,          PRO_CPU_NUM);
    xTaskCreateStaticPinnedToCore(Junkebox::task,          "junkebox_task", M4KB, &(*robot.junkebox), 1, xJunkeboxStack, &xJunkeboxBuffer, PRO_CPU_NUM);
    ESP_LOGI("ROBOT_MAIN", "All FreeRTOS tasks started successfully");
}
