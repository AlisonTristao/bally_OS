#ifndef ROBOT_STATIC_OBJECTS_H
#define ROBOT_STATIC_OBJECTS_H

#include <atomic>
#include <cstdint>
#include <optional>
#include <esp_timer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdio.h>
#include "esp_attr.h"

#include <Settings.h>

#include <ArraySensor.h>
#include <Encoder.h>
#include <HBridge.h>

#include <TinyShell.h>
#include <TinyEKF.h>
#include <ICM42688.h>

#include <Flags.h>
#include <Logger.h>
#include <OTAUpdater.h>
#include <RobotSettings.h>
#include <SDCard.h>
#include <StateMachine.h>
#include <USBMassStorage.h>

#ifdef ENABLE_SYSTEM_MONITOR
#include <SystemMonitor.h>
#endif

/**
 * @brief Non-blocking, periodic sample scheduler. Originally built for the
 * "debug" shell module's per-sensor tests (test_arr_sensor, test_encoder,
 * and future ones such as IMU/H-bridge current); also reused by the
 * "kalman" module to throttle start_log/stop_log.
 *
 * schedule()/cancel()/active() are called from the shell task; poll() is
 * called once per pass from the consuming task — ROBOT::processDebug() on
 * the state-machine task for debug tests, ROBOT::runEKF() for the kalman
 * log. Every field is atomic for that reason.
 */
class ScheduledDebugTest {
public:
    bool schedule(uint32_t samples, uint32_t interval_ms) {
        if (samples == 0) return false;

        interval_ms_.store(interval_ms, std::memory_order_relaxed);
        next_ms_.store(static_cast<uint32_t>(esp_timer_get_time() / 1000ULL),
                       std::memory_order_relaxed);
        remaining_.store(samples, std::memory_order_release);
        return true;
    }

    void cancel() { remaining_.store(0, std::memory_order_release); }

    bool active() const {
        return remaining_.load(std::memory_order_acquire) != 0;
    }

    /**
     * @brief Check whether a sample is due right now.
     * @return true once per due sample; advances the schedule and consumes
     * one remaining sample as a side effect.
     */
    bool poll() {
        const uint32_t remaining = remaining_.load(std::memory_order_acquire);
        if (remaining == 0) return false;

        const uint32_t now_ms =
            static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
        const uint32_t next_ms = next_ms_.load(std::memory_order_relaxed);

        // Signed subtraction keeps the comparison valid across millis overflow.
        if (static_cast<int32_t>(now_ms - next_ms) < 0) return false;

        remaining_.store(remaining - 1, std::memory_order_release);
        next_ms_.store(now_ms + interval_ms_.load(std::memory_order_relaxed),
                       std::memory_order_relaxed);
        return true;
    }

private:
    std::atomic<uint32_t> remaining_{0};
    std::atomic<uint32_t> interval_ms_{0};
    std::atomic<uint32_t> next_ms_{0};
};

class ROBOT {
public:
    // singleton pattern
    // getInstance returns a reference to the single instance of the ROBOT class
    // we use this to acess the robot inside all files without needing to pass the instance as a parameter
    static ROBOT& getInstance() {
        static ROBOT instance;
        return instance;
    }
    void* getInstancePtr() {
        return static_cast<void*>(instance_);
    }

    // destructor, copy etc
    virtual ~ROBOT() {
        // delete the queue
        if (receivedDataQueue == nullptr)
            return;
        vQueueDelete(receivedDataQueue);
        receivedDataQueue = nullptr;
    };
    ROBOT(const ROBOT&) = delete;
    ROBOT& operator=(const ROBOT&) = delete;

    // initialize the robot, configure the pins, the wifi and the esp-now settings
    bool init();

    // routine to be executed in parallel processing
    static void routine(void *param);

    // configure the interruptions for the buttons and side sensors
    static void initInterruptions(void *param);

    // excute a command in the shell and return the result as string
    static void runShell(void *param);

    // run the state machine main loop
    static void runStateMachine(void *param);

    // sample the sensor and wake up de run ekf task
    static IRAM_ATTR void sampleEKF(void *param);

    // run the ekf in parallel processing
    static void runEKF(void *param);

    // core utility objects
    static Logger logger;
    StateMachine machine;
    TinyShell shell;
    RobotSettings settings;
    // Constructed in init(), once pins are known from settings.load() —
    // ArraySensor's constructor configures GPIO/ADC immediately, so it
    // cannot run at static-init time like the other members below.
    std::optional<ArraySensor> array_sensor;
    SDCard sd_card;
    USBMassStorage usb_storage;
    OTAUpdater ota;
#ifdef ENABLE_SYSTEM_MONITOR
    SystemMonitor sysmon;
#endif

    // Execute scheduled DEBUG tests without blocking the state-machine task.
    void processDebug();
    void cancelDebugTests();

    // Force both motors to zero immediately. Belt-and-suspenders on top of
    // the flags' own timeout-based auto shutoff (see Flags_pwm) — used by
    // DEBUG (processDebug) and ERROR (error_function), where "keep the
    // motors off" cannot wait for a flag to expire.
    void stopMotors();

    // Toggle every LED together at settings.data().error_blink_ms (module
    // "error") to make the fail-safe ERROR state visible without telemetry.
    // Call once per pass; non-blocking.
    void blinkErrorLeds();

    // Keep selected shell responses out of the retained PSRAM log.
    void sendNextShellOutputDirect();
    bool consumeDirectShellOutputRequest();
private:
    // default constructor
    ROBOT() :   machine(NONE, NULL, NULL),
                sd_card(MISO, SCK, MOSI, CS),
                buttons("Buttons"),
                sideSensors("Side Sensors"),
                leds("LEDs"),
                motors("Motors")
                {
        // save the instance of the robot class to be used in the static functions
        instance_ = this;
    }

    // private peripheral objects
    // Constructed in init(), once pins are known from settings.load() — same
    // reason as array_sensor/encoder_* above.
    std::optional<HBridge> motor_left;
    std::optional<HBridge> motor_right;
    std::optional<Encoder> encoder_left;
    std::optional<Encoder> encoder_right;
    std::optional<ICM42688> imu;
    // Constructed in initEKF() (called from init(), after settings.load())
    // since Q/R are const and set from settings — same reason as
    // array_sensor/encoder_*/motor_* above.
    std::optional<TinyEKF> EKF;
    TaskHandle_t ekf_task_handle = nullptr; 

    // Signals and flags for buttons, sensors, LEDs, and motors
    Flags_in buttons;
    Flags_in sideSensors;
    Flags_out leds;
    Flags_pwm motors;

    // save a instance of the ROBOT class to be used in the static functions
    static ROBOT* instance_;
    bool initialized = false;

    // Guards configureCommunication(): init() can be retried by app_main's
    // `while (!robot.init())` loop after a later step fails (e.g. motor),
    // and wifi/esp-now bring-up is not safe to redo (esp_wifi_init(),
    // esp_now_init(), esp_now_add_peer()... all error/warn on a second call).
    bool communication_configured_ = false;

    // Set once in init() from imu->begin()'s result. Gates the IMU read in
    // sampleEKF() — skipping it when the sensor never answered avoids
    // retrying (and blocking on) an I2C timeout on every EKF tick.
    bool imu_ready_ = false;

    // Non-blocking per-sensor tests controlled by the "debug" shell module.
    // Add one ScheduledDebugTest member per sensor (IMU, H-bridge current, ...).
    ScheduledDebugTest array_sensor_test_;
    ScheduledDebugTest encoder_test_;
    std::atomic<bool> direct_next_shell_output{false};

    // matriz of data to kalman filter
    float control_input[EKF_CONTROL_DIM] = {0, 0}; // left and right motor pwm
    float measurement[EKF_MEASURE_DIM] = {0, 0, 0, 0, 0};

    // Periodic EKF state + measurement logging for offline tuning, started
    // by "kalman start_log" / stopped by "kalman stop_log". Reuses
    // ScheduledDebugTest's interval/counter machinery but is polled from
    // runEKF() every cycle (not processDebug()) and deliberately left out of
    // canScheduleDebugTest()/anyDebugTestActive()/cancelDebugTests(): unlike
    // the DEBUG-only sensor tests, it must keep logging across state changes
    // (e.g. during RUN, where the filter is actually being tuned).
    ScheduledDebugTest kalman_log_test_;

    void initEKF();

    // Shared gate for every "debug" module test: only in the DEBUG state,
    // and never while the SD card belongs to the USB host.
    bool canScheduleDebugTest() const;
    bool scheduleDebugTest(ScheduledDebugTest& test, uint32_t samples,
                           uint32_t interval_ms);
    bool anyDebugTestActive() const;

    // configure the one pin needed before the SD card can be mounted (CS,
    // fixed at compile time — see include/Settings.h)
    bool configurePinsEarly();

    // configure the remaining pins, sourced from settings.data(), once
    // settings.load() has run
    bool configurePinsFromSettings();

    // configure the wifi and the esp-now settings for the robot
    bool configureCommunication();

    // start wrappers
    void startWrappers();

    // set the time limit for the flags, to reset them after a certain time
    void setTimeLimit();

    // set the outputs flags (leds and motors) to 0 after the time limit is reached
    void setOutputs();

    // get the speed of the robot based on the encoders values
    float getSpeedFromEncoders();
    float getOmegaFromEncoders();

    // Callbacks for ESP-NOW 
    static void handleReceiveStatic(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);

    // Callbacks for ESP-NOW
    static void handleSendStatic(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) ;

    // reset the flags: buttons, side sensors, leds and motors
    void resetFlags();

    // verify the condictions to change the state machine to the next state
    uint32_t stateMachineTimer = 0;
    void checkStateMachine();

    // queue for the logs to be sent in the parallel processing
    QueueHandle_t receivedDataQueue;
};

#endif
