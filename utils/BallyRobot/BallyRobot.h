#ifndef ROBOT_STATIC_OBJECTS_H
#define ROBOT_STATIC_OBJECTS_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include "esp_adc/adc_oneshot.h"
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
#include <btp/endpoint.hpp>
#include <OTAUpdater.h>
#include <RobotSettings.h>
#include <SDCard.h>
#include <StateMachine.h>
#include <USBMassStorage.h>
// After TinyShell.h (above): BtpDevice.h asks TinyShell not to inject
// `using namespace std`, which this codebase still relies on.
#include <BtpDevice.h>
#include <JobScheduler.h>

// SD card root file run automatically at boot, if present. Same convention as
// OTA_WIFI_LIST_FILE (include/Settings.h), ROBOT_SETTINGS_FILE and
// KEY_STORE_FILE: plain text, relative to the SDCard mount point. One shell
// command per line; blank lines and lines starting with '#' are ignored.
#define JOB_AUTOEXEC_FILE "autoexec.job"

// SD card root file "job -save" writes and loadJobs() reads back once at
// boot, after JOB_AUTOEXEC_FILE runs. NOT a script -- its lines are never fed
// through pollScript()/run_command_line(), on purpose: JobScheduler::
// is_job_command() forbids a script line from being a "job" command (a
// script that reschedules itself has no bound), and every restorable job here
// IS one. Its own tiny format (kind,params,command) is parsed directly by
// loadJobs() into JobScheduler::schedule_interval/schedule_on_state calls.
#define JOB_SAVE_FILE "jobs.conf"

#include <SystemMonitor.h>

// Human-readable name for esp_reset_reason() ("panic", "brownout", ...).
// Defined once in BallyRobotShell.cpp (the "sys reset_reason" command) and
// declared here so ROBOT::init() can log the same string automatically at
// boot without duplicating the table.
const char* resetReasonName(esp_reset_reason_t reason);

class ROBOT;

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
    bool schedule(uint32_t samples, uint32_t interval_ms,
                  uint32_t source_id = 0U, uint32_t boot_id = 0U) {
        if (samples == 0) return false;

        interval_ms_.store(interval_ms, std::memory_order_relaxed);
        source_id_.store(source_id, std::memory_order_relaxed);
        boot_id_.store(boot_id, std::memory_order_relaxed);
        next_ms_.store(static_cast<uint32_t>(esp_timer_get_time() / 1000ULL),
                       std::memory_order_relaxed);
        remaining_.store(samples, std::memory_order_release);
        return true;
    }

    void cancel() { remaining_.store(0, std::memory_order_release); }

    bool active() const {
        return remaining_.load(std::memory_order_acquire) != 0;
    }

    uint32_t source_id() const { return source_id_.load(std::memory_order_relaxed); }
    uint32_t boot_id() const { return boot_id_.load(std::memory_order_relaxed); }

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
    std::atomic<uint32_t> source_id_{0};
    std::atomic<uint32_t> boot_id_{0};
};

/**
 * @brief High-rate snapshot of encoder/IMU readings, written once per
 * sampleEKF() tick (cfg.sample_micros) and read by anyone who needs the
 * latest values without re-reading hardware: runEKF() (same task, right
 * after the write) and readSensors() (BtpDevice's comms task, publishing
 * "robot.sensors" at whatever rate is subscribed). Each field is an
 * independent atomic -- same tolerated-torn-read posture already used by
 * ScheduledDebugTest: a cross-task reader may see a value from one tick earlier, never a
 * corrupt one, and self-heals on the next tick. No lock, so the 1kHz writer
 * never waits on a slower reader.
 *
 * current_a/current_b and the array sensor are NOT sampled at 1kHz: an
 * earlier version did, and up to 10 ADC reads with mux-settle delays every
 * tick made that task run long enough to starve every other task on the
 * core (every shell command hanging forever on the bench). Neither feeds the
 * EKF, so sampleSlowSensors() reads them on the state-machine task -- the
 * one that already drives the mux in RUN, so the two never interleave --
 * and only while "robot.sensors" has a subscriber.
 */
struct SensorSnapshot {
    std::atomic<float> enc_linear_speed{0.0f};    // m/s
    std::atomic<float> enc_angular_speed{0.0f};   // rad/s
    std::atomic<float> gyro_z{0.0f};              // rad/s
    std::atomic<float> accel_x{0.0f};             // m/s^2
    std::atomic<float> accel_y{0.0f};             // m/s^2
    // sampleSlowSensors(), at most kSlowSensorPeriodUs apart.
    std::atomic<uint16_t> current_a{0U};          // raw ADC counts
    std::atomic<uint16_t> current_b{0U};
    std::atomic<uint8_t> array[ArraySensor::MAX_LEN] = {};  // raw >> 4
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

    // initialize the robot: pins, SD card, settings, sensors and actuators.
    // The link comes after, from main.cpp (BtpDevice, then attachDevice()).
    bool init();

    // routine to be executed in parallel processing
    static void routine(void *param);

    // configure the interruptions for the buttons and side sensors
    static void initInterruptions(void *param);

    // run the state machine main loop
    static void runStateMachine(void *param);

    // sample the sensor and wake up de run ekf task
    static IRAM_ATTR void sampleEKF(void *param);

    // run the ekf in parallel processing
    static void runEKF(void *param);

    // core utility objects
    static Logger logger;
    // Numbers the log records and encodes the SD log files (Logger::
    // configure_btp) with this device's BTP identity (btpdev::identity()).
    // Nothing is sent through it: the link is BtpDevice's.
    btp::Endpoint protocol;
    StateMachine machine;
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
    // Always built (the old ENABLE_SYSTEM_MONITOR build flag is gone):
    // observability that a commented-out line in platformio.ini can switch
    // off is observability the field build never has. Periodic reporting
    // goes out over the BTP system.monitor telemetry topic (see
    // src/btp/btp_topics.cpp); this instance also backs
    // the on-demand "sysmon"/"sys" shell commands.
    SystemMonitor sysmon;

    // Time- and state-triggered shell commands (the "job" module). Public
    // because main.cpp has no business here but BallyRobotShell.cpp registers
    // against it; poll() is driven from routine(), see pollJobs().
    JobScheduler jobs;

    // Execute scheduled DEBUG tests without blocking the state-machine task.
    void processDebug();
    void cancelDebugTests();

    // Motor arming, enforced in setOutputs() and not merely checked by the
    // shell commands: while disarmed the outputs are forced to zero on every
    // pass, whatever is left in the PWM flags. That is what makes
    // "motion -disarm" a kill switch rather than a polite request.
    //
    // Armed at boot so the existing bench flow (robot -set_pwm, the state
    // machine) behaves exactly as before this existed.
    std::atomic<bool> motors_armed_{true};

    // Deadline for "motion -coast", in ms since boot; 0 means "not coasting".
    //
    // A latch is needed because setOutputs() re-applies the PWM flags every
    // routine pass and HBridge::applyPWM(0) is an ACTIVE BRAKE -- a bare
    // coast() call would be undone within a millisecond. Any non-zero PWM
    // command wins over a pending coast; see setOutputs().
    std::atomic<uint32_t> motors_coast_until_ms_{0U};
    void setCoast(uint32_t duration_ms);
    void clearCoast();

    // Force both motors to zero immediately. Belt-and-suspenders on top of
    // the flags' own timeout-based auto shutoff (see Flags_pwm) — used by
    // DEBUG (processDebug) and ERROR (error_function), where "keep the
    // motors off" cannot wait for a flag to expire.
    void stopMotors();

    // Toggle every LED together at settings.data().error_blink_ms (module
    // "error") to make the fail-safe ERROR state visible without telemetry.
    // Call once per pass; non-blocking.
    void blinkErrorLeds();

    // COMM_CONFIG when init() found both cfg.btn1 and cfg.btn2 held at
    // boot; DEBUG when only button 3 / cfg.btn2 (OTA) or only button 2 /
    // cfg.btn1 (USB storage) was held and that sub-mode actually started;
    // SETUP otherwise. cfg.btn0 can't be used for this: it is
    // GPIO0, the boot strapping pin -- holding it at reset drops the chip
    // into ROM download mode instead of running the firmware.
    // Read once by main.cpp, right after init() returns, to pick the state
    // machine's initial state — this is the boot-time equivalent of the
    // existing DEBUG-state shell commands ("ota start" / "storage expose"),
    // which are unaffected and still work the normal way.
    stateName bootState() const { return boot_state_; }

    // ---- BtpDevice (see device_ below) ------------------------------------
    // The link this boot uses: comm_mode from settings.conf, or TCP when the
    // boot-time chord asked for OTA (the update server rides BtpDevice's
    // Wi-Fi connection). Valid after init().
    btpdev::Link bootLink() const { return boot_link_; }
    // Everything that needs the running device: OTA, the boot-time USB
    // storage mode, the logger's link, the jobs. Called by main.cpp right
    // after device.begin(), whatever it returned (`ready` false: the robot
    // runs without a link).
    void attachDevice(BtpDevice& device, bool ready);
    // Every robot shell module, on BtpDevice's shell (src/btp/btp_commands.cpp).
    void registerShell(TinyShell& shell);

    // Telemetry snapshots for src/btp/btp_topics.cpp, read on BtpDevice's
    // comms task at each topic's subscribed rate.
    struct SensorsSample {
        float linear_speed = 0.0f;    // m/s
        float angular_speed = 0.0f;   // rad/s
        float gyro_z = 0.0f;          // rad/s
        float accel_x = 0.0f;         // m/s^2
        float accel_y = 0.0f;         // m/s^2
        uint16_t current_a = 0U;      // raw ADC counts
        uint16_t current_b = 0U;
        uint8_t array[ArraySensor::MAX_LEN] = {};  // raw >> 4
    };
    void readSensors(SensorsSample& out);
    struct FlagsSample {
        uint8_t buttons = 0U;
        uint8_t side_sensors = 0U;
        uint8_t leds = 0U;
        int8_t pwm_left = 0;   // %
        int8_t pwm_right = 0;
    };
    void readFlags(FlagsSample& out);
    uint8_t currentState() const;
    uint32_t nextProtocolTestCounter() {
        return protocol_test_counter_.fetch_add(1U, std::memory_order_relaxed);
    }
    // The "system.monitor" text (task table, heap), at most max_size octets.
    std::string systemReport(size_t max_size);

    // Keep selected shell responses out of the retained PSRAM log.
    void sendNextShellOutputDirect();
    bool consumeDirectShellOutputRequest();

    // ---- COMM_CONFIG (T25b, TAREFAS_TCP_BLE_ANDROID.txt, ETAPA 3B) ----
    //
    // Action function for the COMM_CONFIG state (src/robot/09_CommConfig.cpp
    // just forwards to this). Lives here, not in src/robot/, for the same
    // reason blinkErrorLeds()/processDebug() do: it needs leds/buttons/
    // settings/sd_card, all private/ROBOT-owned. Called once per
    // state-machine task pass while current_state == COMM_CONFIG (i.e. quite
    // often -- see StateMachine::run()), so button edge-detection here uses
    // its own lightweight debounce rather than anything that assumes a slow
    // cadence.
    //
    // Menu: btn1 advances the selection (0->1->2->3->0 = ESP-NOW/TCP/BLE/
    // none), btn2 goes back, btn0 confirms (persists RobotSettings'
    // "comm.comm_mode" and reboots), ~30s with no button press exits without
    // saving and reboots anyway (never leaves the robot stuck in this menu).
    // Always returns COMM_CONFIG -- the only ways out are the two reboots
    // above, never a live state transition (see the note next to
    // transitionTable in StatesManager.h for why a button-driven row here
    // would be actively wrong, not just redundant).
    stateName commConfigTick();

    // Bound to esp_register_shutdown_handler() in init(): esp_restart()'s
    // shutdown_handler_t is a plain void(*)(void), no context parameter, so
    // this cannot be a capturing lambda or a non-static member -- it reaches
    // the singleton itself. Public only because esp_register_shutdown_handler
    // needs a pointer to it from init(); nothing else should call it.
    static void flushLogsOnShutdown();
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
    // The periodic timer that drives sampleEKF() at cfg.sample_micros.
    // Kept as a member (it used to be a local in initInterruptions(), created
    // and then thrown away) so "settings apply timers" can esp_timer_restart()
    // it with a new period instead of the value being frozen until reboot.
    esp_timer_handle_t ekf_timer_handle_ = nullptr;

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

    // ---- COMM_CONFIG menu state (T25b), owned by commConfigTick() ----
    // comm_config_active_ latches true on the first commConfigTick() call
    // after entering COMM_CONFIG (one-time setup: seed comm_config_selected_
    // from the persisted comm_mode, prime the edge trackers below to the
    // CURRENT button levels so a button already held the instant the state
    // is entered -- e.g. btn1/btn2, still down from the boot-time chord that
    // got here (ROBOT::init()) -- is not misread as a fresh press). Never reset back to false during
    // a boot: COMM_CONFIG is only ever entered once per boot (every exit is
    // a reboot), so there is nothing to re-arm.
    bool     comm_config_active_ = false;
    bool     comm_config_restart_scheduled_ = false; // guards scheduling the reboot timer twice
    uint8_t  comm_config_selected_ = 0;              // 0..3, currently displayed option
    uint32_t comm_config_last_activity_ms_ = 0;
    // Simple "already processed this edge" debounce per button -- see
    // commConfigTick()'s own comment for why this is deliberately simpler
    // than the interrupts task's ANYEDGE ISR debounce.
    bool     comm_config_btn0_was_low_ = false;
    bool     comm_config_btn1_was_low_ = false;
    bool     comm_config_btn2_was_low_ = false;
    uint32_t comm_config_last_btn0_edge_ms_ = 0;
    uint32_t comm_config_last_btn1_edge_ms_ = 0;
    uint32_t comm_config_last_btn2_edge_ms_ = 0;

    // save a instance of the ROBOT class to be used in the static functions
    static ROBOT* instance_;
    bool initialized = false;

    // Set in init() (see bootState() above); stays SETUP unless a boot-time
    // sub-mode button was held AND that sub-mode actually started.
    stateName boot_state_ = SETUP;

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

    // High-rate encoder/IMU/current snapshot, written by sampleEKF() -- see
    // SensorSnapshot's own comment. The single source of truth for both the
    // EKF's measurement[] (below) and "robot.sensors" telemetry.
    SensorSnapshot sensor_snapshot_;

    // ADC oneshot handles for cfg.current_a/current_b (DRV8251A current
    // sense), set up once by initCurrentSensors() -- same
    // adc_oneshot_io_to_channel()/adc_oneshot_new_unit()/
    // adc_oneshot_config_channel() idiom ArraySensor's constructor already
    // uses for its own "sig" pin, same attenuation/bitwidth
    // (ADC_ATTEN_DB_12/ADC_BITWIDTH_12) for consistency. Two independent
    // handles because the two pins do not necessarily share a physical ADC
    // unit (ESP32-S3: GPIO1-10 are ADC1, GPIO11-20 are ADC2) -- current_a
    // (GPIO14) lands on the same unit (ADC2) ArraySensor's "sig" pin (GPIO13)
    // already owns. initCurrentSensors() detects that and skips current_a
    // entirely rather than calling adc_oneshot_new_unit() on an
    // already-owned unit: on the bench that second call did NOT fail
    // cleanly, it left the ADC in a state where every later read (this
    // robot's own array_sensor reads included) ran pathologically long,
    // starving every other task on the core. A null handle here just means
    // readCurrentRaw() returns 0 for that channel, same fail-soft posture
    // ArraySensor::read() already has.
    adc_oneshot_unit_handle_t current_a_adc_handle_ = nullptr;
    adc_oneshot_unit_handle_t current_b_adc_handle_ = nullptr;
    adc_channel_t current_a_adc_channel_ = ADC_CHANNEL_0;
    adc_channel_t current_b_adc_channel_ = ADC_CHANNEL_0;
    // Sets up current_a_adc_handle_/current_b_adc_handle_ above from
    // settings.data().current_a/current_b. Called once from init(), right
    // after configurePinsFromSettings() resets those two pins to floating
    // analog input (same point ArraySensor's own ADC setup effectively runs,
    // via array_sensor.emplace()).
    void initCurrentSensors();
    // Reads one already-configured current-sense channel. Returns 0 when its
    // handle is null (setup failed/skipped) -- never blocks, never retries.
    uint16_t readCurrentRaw(adc_oneshot_unit_handle_t handle, adc_channel_t channel);

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

    // Bind one RobotSettings::ApplyFn per module that can meaningfully take
    // effect without a reboot ("timers", "logger", "ota", "ekf_noise",
    // "kinematics", "error"). Called once from init(), after startWrappers()
    // so the "settings" shell module already exists.
    //
    // Deliberately NOT registered for "sensor" or any pins_* group: ArraySensor
    // reconfigures ADC/GPIO from its constructor with no live reconfiguration
    // path, and the pin groups size hardware that is already wired up.
    // RobotSettings::apply() reports NoApplier ("requires a reboot") for those
    // on its own -- nothing to do here for them.
    void registerSettingsAppliers();

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

    // Register every shell module. Most modules are owned by the subsystem
    // they operate on (see each lib's own register_shell_commands); this
    // function is now just the composition root wiring those together,
    // plus the three modules below that stay here because their guards or
    // shared state span multiple private members with no single owning
    // subsystem.
    void startWrappers();

    // "motion": movement in the robot's own terms (arm/disarm/drive/stop/
    // brake/coast/limits). Stays here for the same reason "robot" does: it
    // drives the Flags_pwm/HBridge pair this composition owns, and the arming
    // gate is enforced in setOutputs(), which is also this class's.
    void registerMotionCommands();

    // Fase 4 coverage: commands added into modules the subsystems already
    // own (sensor/storage/ota/junkebox/debug/help), for capabilities whose
    // gate or context the owning library cannot see by itself.
    void registerCoverageCommands();

    // "job": time- and state-triggered shell commands, plus the SD script
    // runner. Stays here — JobScheduler is deliberately TinyShell-free so it
    // can be unit tested under env:native (see its class comment), and the
    // script runner needs both the SD card and the command queue, which are
    // this class's.
    void registerJobCommands();

    // "sys": machine identity, health and lifecycle (info/identity/health/
    // uptime/tasks/memory/temp/reset_reason/boot_mode/reboot/factory_reset).
    // Stays here — it crosses esp_system, esp_ota_ops, the BTP endpoint's
    // identity, SystemMonitor and StateMachine at once, so no single lib can
    // answer "who am I and how am I".
    void registerSystemCommands();

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

    // get the linear/angular speed of the robot based on the encoders
    // values. Reads each encoder's getCountDiff() exactly once -- it is a
    // stateful "diff since last call" getter (see Encoder::getCountDiff()),
    // so calling it a second time here (once for linear, once for angular)
    // would starve the second reading of almost every pulse.
    void getVelocitiesFromEncoders(float& linear_speed, float& angular_speed);

    // reset the flags: buttons, side sensors, leds and motors
    void resetFlags();

    // verify the condictions to change the state machine to the next state
    uint32_t stateMachineTimer = 0;
    void checkStateMachine();

    // Runs one shell line from inside the firmware (a job firing, a line of
    // an SD script) on BtpDevice's shell task (BtpDevice::run()). Returns
    // false when its queue is full (the caller decides whether to drop or
    // retry -- the job scheduler drops, the script runner retries), the line
    // does not fit, or there is no device.
    bool submitLocalCommand(const char* command_line);
    static bool submitLocalCommandStatic(void* context, const char* command_line);

    // ---- SD script runner ("job -run_file", and JOB_AUTOEXEC_FILE at boot) --
    //
    // Deliberately NOT part of JobScheduler: that library is pure C++ so it
    // can be unit tested, and reading a file needs the SD card.
    //
    // The whole file is read into script_buffer_ once and then fed one line
    // per routine() pass. Two reasons it is not streamed: SDCard allows a
    // single open stream at a time and Logger::flush_to_sd wants it too, and
    // a cursor over memory cannot leave a stream open across passes. One line
    // per pass is also what keeps a 50-line script from overflowing the
    // 10-deep command queue — the failure this replaced.
    static constexpr size_t kScriptMaxBytes = 2048U;
    char     script_buffer_[kScriptMaxBytes + 1U]{};
    size_t   script_size_ = 0U;
    size_t   script_cursor_ = 0U;
    uint16_t script_line_ = 0U;
    bool     script_active_ = false;
    bool     autoexec_done_ = false;

    /** @brief Load a script file and start feeding it. @return false when the
     *  card is unmounted, the file is missing, or it is larger than
     *  kScriptMaxBytes. */
    bool startScript(const char* path);
    /** @brief Feed at most one line. Called once per routine() pass. */
    void pollScript();
    /** @brief Drive JobScheduler from routine(); once per pass. */
    void pollJobs();

    // ---- Job persistence (JOB_SAVE_FILE) ----
    //
    // Interval and OnStateEnter jobs only. A Once job's delay_ms is relative
    // to the moment it was scheduled -- a reboot erases that reference point,
    // so restoring it after one would fire delay_ms after the NEW boot
    // instead, a silently different job. Skipped, not approximated.
    /** @brief Write every active Interval/OnStateEnter job to JOB_SAVE_FILE.
     *  @return false when the SD card is not mounted or the write failed. */
    bool saveJobs();
    /** @brief Re-schedule whatever JOB_SAVE_FILE holds, straight into
     *  JobScheduler -- never through TinyShell::run_command_line(), since
     *  these lines are not shell command text.
     *  @return false when the card is not mounted or there is no such file
     *  (both are normal: nothing was ever saved). */
    bool loadJobs();

    // ---- BtpDevice ---------------------------------------------------------
    // The link, the terminal, the key and the telemetry (src/btp/ holds the
    // configuration). Set by attachDevice(); nullptr before that, and on a
    // boot whose begin() failed (the robot still runs, just unreachable).
    BtpDevice* device_ = nullptr;
    // BtpDevice's shell, while registerShell() runs (the shell modules).
    TinyShell* shell_ = nullptr;
    // The link this boot uses (comm_mode, or TCP for a boot into OTA).
    btpdev::Link boot_link_ = btpdev::Link::EspNow;
    bool boot_enter_ota_ = false;
    bool boot_enter_storage_ = false;
    // "robot.state" is published on change (runStateMachine()).
    stateName last_published_state_ = NONE;
    // current_a/current_b and the array sensor into sensor_snapshot_, while
    // "robot.sensors" has a subscriber: 50 Hz, the topic's highest rate.
    // State-machine task (see SensorSnapshot).
    static constexpr uint64_t kSlowSensorPeriodUs = 20000ULL;
    uint64_t next_slow_sample_us_ = 0U;
    void sampleSlowSensors();
    // protocol.test's counter (src/btp/btp_topics.cpp).
    std::atomic<uint32_t> protocol_test_counter_{0U};
};

#endif
