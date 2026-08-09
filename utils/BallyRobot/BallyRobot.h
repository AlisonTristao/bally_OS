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
#include <Junkebox.h>

#include <TinyShell.h>
#include <TinyEKF.h>
#include <ICM42688.h>

#include <Flags.h>
#include <Logger.h>
#include <BtpTransport.h>
#include <CommandProcessor.h>
#include <TelemetryPublisher.h>
#include <TxScheduler.h>
#include <OTAUpdater.h>
#include <RobotSettings.h>
#include <SDCard.h>
#include <StateMachine.h>
#include <USBMassStorage.h>
#include <btp/fragmentation.hpp>

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
    virtual ~ROBOT() = default;
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
    BtpEndpoint protocol;
    TxScheduler tx_scheduler;
    CommandProcessor command_processor;
    TelemetryPublisher telemetry;
    StateMachine machine;
    TinyShell shell;
    RobotSettings settings;
    // Constructed in init(), once pins are known from settings.load() —
    // ArraySensor's constructor configures GPIO/ADC immediately, so it
    // cannot run at static-init time like the other members below.
    std::optional<ArraySensor> array_sensor;
    SDCard sd_card;
    // Constructed in init() once cfg.bzr (settings) is known — same reason
    // as array_sensor above. Public (not alongside motor_left/imu below)
    // because state functions in src/robot/ call junkebox->play() directly,
    // the same way 03_Calibrate.cpp calls array_sensor->calibrate().
    std::optional<Junkebox> junkebox;
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

    // Reacts to a button/side-sensor flag edge or a state transition (see
    // kSoundTriggerTable in BallyRobot.cpp) by triggering the matching
    // Junkebox::BuiltinSound. Lives here, not in Junkebox: it combines
    // buttons/sideSensors + StateMachine + junkebox with no single natural
    // owner — exactly the case CONTRIBUTING.md's decision rule puts on
    // ROBOT. Called once per routine() pass, after checkStateMachine().
    void updateSoundFeedback();
    uint8_t   previous_buttons_ = 0;      // buttons.getFlags() as of the last call
    uint8_t   previous_side_sensors_ = 0; // sideSensors.getFlags() as of the last call
    stateName previous_state_ = NONE;     // StateMachine::current_state as of the last call

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
    // Add one ScheduledDebugTest member per sensor (H-bridge current, ...).
    ScheduledDebugTest array_sensor_test_;
    ScheduledDebugTest encoder_test_;
    ScheduledDebugTest imu_test_;
    // Repeatedly re-checks WHO_AM_I on the already-open IMU bus and tallies
    // pass/fail, so intermittent I2C flakiness (bad pull-ups, noise, a loose
    // wire) shows up as a hit rate over time instead of one-off scan_i2c
    // samples. Counters reset each time "test_i2c" (re)starts the test.
    ScheduledDebugTest imu_i2c_test_;
    uint32_t           imu_i2c_ok_count_    = 0;
    uint32_t           imu_i2c_fail_count_  = 0;
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

    // One-shot I2C bus scan, logged at boot right before imu.emplace()
    // claims sda_pin/scl_pin. Must run before the IMU's own bus is created:
    // ICM42688::begin() keeps its i2c_master_bus_handle_t for the object's
    // whole lifetime (freed only in its destructor, which optional<ICM42688>
    // never triggers), even when the WHO_AM_I check fails — so the GPIOs are
    // permanently claimed the moment begin() runs, and any later scan on the
    // same pins would fail to create its own bus.
    // @return true if any address ACKed.
    bool scanI2CBus(uint8_t sda_pin, uint8_t scl_pin);

    // Manual, bit-banged I2C scan run right before scanI2CBus() (same
    // pin-ownership constraint applies — see its comment above). Purely
    // diagnostic: drives START/address/ACK/STOP by hand with generous,
    // way-below-spec timing (hundreds of microseconds per half-bit, vs. the
    // few microseconds a real 100/400kHz bus allows), so it can still get an
    // ACK out of a bus too electrically weak (missing external pull-ups,
    // long wires) for the hardware I2C peripheral's stricter timing, which
    // just times out instead of ever reporting NACK.
    // @return true if any address ACKed.
    bool bitbangI2CScan(uint8_t sda_pin, uint8_t scl_pin);

    // Hand-solder short check: drives one line low (open-drain) while
    // leaving the other released, then reads the released one. If the
    // "released" line follows the driven one down, the two nets are
    // electrically the same wire somewhere (solder bridge between the SDA
    // and SCL pads/pins) — an idle-level read alone can't tell this apart
    // from "nothing connected", since a genuinely shorted pair still floats
    // HIGH together at rest once both are released.
    // @return true if a short is detected either direction.
    bool checkSdaSclShort(uint8_t sda_pin, uint8_t scl_pin);

    // configure the wifi and the esp-now settings for the robot
    bool configureCommunication();
    bool configureProtocolIdentity();
    void processCommandRequest(const btp::Header& header,
                               btp::ByteView payload);
    void sampleTelemetry();

    // Register every shell module. Most modules are owned by the subsystem
    // they operate on (see each lib's own register_shell_commands); this
    // function is now just the composition root wiring those together,
    // plus the three modules below that stay here because their guards or
    // shared state span multiple private members with no single owning
    // subsystem.
    void startWrappers();

    // "robot": raw actuator/virtual-input I/O (btn/ssr/set_pwm/set_led).
    // Stays here — buttons/sideSensors/leds/motors are this composition's
    // own wiring of the generic Flags_in/out/pwm primitives (lib/Flags),
    // not something any single subsystem owns.
    void registerRobotIOCommands();

    // "kalman": EKF state + periodic tuning log. Stays here — EKF is an
    // external vendored filter (TinyEKF) with no home-grown wrapper lib of
    // its own; ROBOT is what actually owns the filter instance, the sample
    // timer and the control/measurement vectors.
    void registerKalmanCommands();

    // "debug": scheduled, non-blocking per-sensor tests. Stays here — the
    // DEBUG-state/USB-idle gate (canScheduleDebugTest) and the scheduler
    // state (array_sensor_test_/encoder_test_) are ROBOT-private, applied
    // uniformly across sensors rather than owned by any one of them.
    void registerDebugCommands();

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

    struct QueuedCommand {
        uint8_t cache_slot;
        char text[btp_command::kMaxShellCommandSize + 1U];
    };

    static constexpr size_t kCommandQueueLength = 10U;
    QueueHandle_t receivedDataQueue = nullptr;
    StaticQueue_t received_data_queue_control_{};
    uint8_t received_data_queue_storage_[
        kCommandQueueLength * sizeof(QueuedCommand)]{};

    static constexpr size_t kCommandReassemblySlots = 2U;
    btp::ReassemblySlot command_reassembly_slots_[kCommandReassemblySlots];
    uint8_t command_reassembly_buffers_[kCommandReassemblySlots]
                                       [btp_command::kMaxLogicalRequestSize]{};
    btp::ReassemblyStorage command_reassembly_storage_[kCommandReassemblySlots]{};
    std::optional<btp::Reassembler> command_reassembler_;

    // The state-machine task is the sole telemetry producer. The routine task
    // only consumes the publisher's bounded SPSC queue.
    static constexpr uint64_t kProtocolTestPeriodUs = 20000U;
    uint64_t next_protocol_test_us_ = 0U;
    uint32_t protocol_test_counter_ = 0U;
    stateName last_telemetry_state_ = NONE;
};

#endif
