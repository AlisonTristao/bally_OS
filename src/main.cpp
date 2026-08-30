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

static StackType_t xSystemMonitorStack[M4KB];
static StaticTask_t xSystemMonitorBuffer;
static void init_system_monitor();

static StackType_t xRoutineStack[M8KB];
static StaticTask_t xRoutineBuffer;
static StackType_t xCommsStack[M8KB];
static StaticTask_t xCommsBuffer;
static StackType_t xShellStack[M8KB];
static StaticTask_t xShellBuffer;
static StackType_t xStateMachineStack[M8KB];
static StaticTask_t xStateMachineBuffer;
static StackType_t xEKFStack[M2KB];
static StaticTask_t xEKFBuffer;
static StackType_t xInterruptsStack[M2KB];
static StaticTask_t xInterruptsBuffer;
// SD-backed playback enters stdio/VFS/FatFs/SDSPI before the first note.
// That call chain needs materially more stack than compiled-in playback,
// which never leaves Junkebox's parser. 2 KiB was enough for builtins but
// could trip the FreeRTOS stack canary as soon as play_file() called fopen().
static StackType_t xJunkeboxStack[M4KB];
static StaticTask_t xJunkeboxBuffer;

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

    // System health reporting. Always started; timers.sysmon_freq_ms == 0
    // silences the periodic report without taking the on-demand commands away.
    init_system_monitor();

    vTaskDelete(NULL); // goodbye main app
}

static void setup_system_callbacks() {
    // Every BTP producer enqueues through the priority scheduler. The radio
    // callback below is the only place that starts an ESP-NOW transmission.
    //
    // 60 ms delivery timeout (down from the 250 ms default): the scheduler
    // sends the next frame only once the previous one's ESP-NOW send callback
    // lands or this timeout expires. Near the motors the callback is regularly
    // late or lost to RF noise, and at 250 ms that collapsed the whole radio
    // output to ~4 frames/s — STATUS, telemetry and COMMAND_RESULT all with it.
    // A callback that has not arrived in 60 ms is not going to change the
    // outcome; the frame was already handed to the driver. A late callback is
    // harmless (on_delivery() no-ops once awaiting_delivery_ clears).
    robot.tx_scheduler.configure([](void*, const uint8_t *data, size_t len) {
        static const uint8_t peer_mac[6] = {MAC_ADDR};
        return (esp_now_send(peer_mac, data, len)) == ESP_OK;
    }, nullptr, 60U);
    robot.protocol.set_send_callback(TxScheduler::enqueue_callback,
                                     &robot.tx_scheduler);

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

    // The "state" shell module. Registered from here, not from
    // ROBOT::startWrappers(), because the transition policy lives in
    // src/robot/StatesManager.cpp -- main.cpp is the one place that already
    // knows both the States singleton and the robot's shell, so the
    // composition root never has to include application state policy.
    states.register_shell_commands(robot.shell, robot.logger);

    // Start the state machine in SETUP, unless robot.init() already jumped
    // straight into DEBUG because button 1/2 was held at boot (see
    // ROBOT::bootState()).
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
    // for state_machine alone, see CONTRIBUTING.md): EKF (4) = comms (4) >
    // routine (3) > shell (2) > junkebox (1) > interrupts (0). junkebox sits
    // just above interrupts because note timing only needs to be roughly
    // on-beat, not exact — it should never delay EKF sampling, motor routine
    // upkeep or shell command handling.
    //
    // comms is split out of routine on purpose: the robot<->dongle link's
    // liveness signal (CONTROL/STATUS on channel C) and the ESP-NOW TX pump
    // must keep running at a steady rate even when routine() stalls on an SD
    // flush, a contended logger mutex or a slow control-loop pass. When they
    // rode routine(), a routine() hiccup made the robot look offline to the
    // dongle (hub.peers) and stalled every MANIFEST_DATA / COMMAND_RESULT the
    // TX scheduler was holding. comms only pumps queues and re-publishes
    // STATUS — it never blocks — so it is safe at EKF's priority; EKF itself
    // spends almost all its time blocked on ulTaskNotifyTake().
    xTaskCreateStaticPinnedToCore(robot.routine,           "routine",       M8KB, NULL, 3,  xRoutineStack,      &xRoutineBuffer,      PRO_CPU_NUM);
    xTaskCreateStaticPinnedToCore(robot.runComms,          "comms",         M8KB, NULL, 4,  xCommsStack,        &xCommsBuffer,        PRO_CPU_NUM);
    xTaskCreateStaticPinnedToCore(robot.initInterruptions, "interrupts",    M2KB, NULL, 0,  xInterruptsStack,   &xInterruptsBuffer,   PRO_CPU_NUM);
    xTaskCreateStaticPinnedToCore(robot.runStateMachine,   "state_machine", M8KB, NULL, 10, xStateMachineStack, &xStateMachineBuffer, APP_CPU_NUM);
    xTaskCreateStaticPinnedToCore(robot.runShell,          "shell_task",    M8KB, NULL, 2,  xShellStack,        &xShellBuffer,        PRO_CPU_NUM);
    xTaskCreateStaticPinnedToCore(robot.runEKF,            "EKF_task",      M2KB, NULL, 4,  xEKFStack,          &xEKFBuffer,          PRO_CPU_NUM);
    xTaskCreateStaticPinnedToCore(Junkebox::task,          "junkebox_task", M4KB, &(*robot.junkebox), 1, xJunkeboxStack, &xJunkeboxBuffer, PRO_CPU_NUM);
    ESP_LOGI("ROBOT_MAIN", "All FreeRTOS tasks started successfully");
}

static void init_system_monitor() {
    // robot.sysmon is already configured (begin()/callbacks) by robot.init();
    // this task only drives its periodic report.
    //
    // sysmon_freq_ms is read every pass, not captured: "settings set timers
    // sysmon_freq_ms 0" silences the report from the next pass on, and any
    // non-zero value resumes it, with no reboot and without disturbing the
    // on-demand sysmon/sys commands. The idle poll below is what makes 0 mean
    // "quiet" instead of "spin at full speed on vTaskDelay(0)".
    static constexpr uint32_t kDisabledPollMs = 1000;

    xTaskCreateStaticPinnedToCore(
        [](void* param) {
            (void)param;
            while (true) {
                const uint32_t period_ms = robot.settings.data().sysmon_freq_ms;
                if (period_ms == 0) {
                    vTaskDelay(pdMS_TO_TICKS(kDisabledPollMs));
                    continue;
                }
                robot.sysmon.update();
                robot.sysmon.report();
                vTaskDelay(pdMS_TO_TICKS(period_ms));
            }
        }, "system_monitor", M4KB, NULL, 1, xSystemMonitorStack, &xSystemMonitorBuffer, PRO_CPU_NUM
    );
    ESP_LOGI("ROBOT_MAIN", "System monitor task started");
}
