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
#include <ManifestResponder.h>
#include <SubscriptionResponder.h>
#include <TerminalResponder.h>
#include <StatusReporter.h>
#include <TelemetryPublisher.h>
#include <TxScheduler.h>
#include <btp/node.hpp>
#include <bally_channels.h>
#include <OTAUpdater.h>
#include <RobotSettings.h>
#include <SDCard.h>
#include <KeyStore.h>
#include <RadioSeal.h>
#include <StateMachine.h>
#include <USBMassStorage.h>
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

class ROBOT;

// The btp::NodeConfig this robot's node_ is built on. btp::NodeConfig became
// an abstract class (BTP library 2.34.0): the identity is plain data, every
// external dependency a virtual method Node calls through. Held BY REFERENCE
// by node_, so it lives as a ROBOT member (protocol_link_), constructed once
// and never moved.
//
// This robot's node_ is receive-only over ESP-NOW: send() returns false
// (every real send still goes out through `protocol` / TxScheduler, wired in
// main.cpp), and open() is the AEAD-open path that used to be
// handleReceiveStatic's stage two (RadioSeal::open / open_e, fail-closed,
// classified by channel). seal() / terminal() / has_command() land here as
// the btp::Node adoption widens (serve_catalog, on_terminal, ...).
class RobotLink : public btp::NodeConfig {
public:
    explicit RobotLink(ROBOT& robot) noexcept : robot_(robot) {}

    bool send(const std::uint8_t* frame, std::size_t frame_size) override;

    bool has_open() const noexcept override { return true; }
    bool open(const btp::Header& header, std::uint16_t sealed_size,
              const std::uint8_t* sealed, std::uint8_t* out_plaintext) override;

private:
    ROBOT& robot_;
};

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
    // protocol_link_ (node_'s btp::NodeConfig) forwards open() into
    // ROBOT::protocolOpen -- the one copy of the classify-then-RadioSeal logic.
    friend class RobotLink;

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

    // Constructs node_ (needs TxScheduler already configured -- see node_'s
    // own comment) and points `protocol` at its btp::Endpoint, so every
    // producer sending through `protocol` and node_'s own receive share one
    // sequence counter for this robot's identity. Called once from main.cpp's
    // setup_system_callbacks(), right after tx_scheduler.configure() (public,
    // like protocol.set_send_callback() right next to it there -- init()
    // itself runs too early, before TxScheduler exists). Returns false on a
    // bad identity or storage (node_->begin() failing) -- a programming
    // error, checked once at boot, same rule as rx_router_.valid() used to
    // have.
    bool bindProtocolTransport();

    // routine to be executed in parallel processing
    static void routine(void *param);

    // dedicated radio task: drains the TX scheduler and re-publishes STATUS at
    // a steady rate, decoupled from routine()'s workload (see start_freertos_
    // tasks() in main.cpp for why). Never blocks.
    static void runComms(void *param);

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
    ManifestResponder manifest_responder;
    TelemetryPublisher telemetry;
    SubscriptionResponder subscription_responder;
    TerminalResponder terminal_responder;
    StatusReporter status_reporter;
    StateMachine machine;
    TinyShell shell;
    RobotSettings settings;
    // Constructed in init(), once pins are known from settings.load() —
    // ArraySensor's constructor configures GPIO/ADC immediately, so it
    // cannot run at static-init time like the other members below.
    std::optional<ArraySensor> array_sensor;
    SDCard sd_card;
    // The two channel keys (E, TraceView<->robot; L, dongle<->robot),
    // loaded once from bally.key in init(). Nothing encrypts against them
    // yet -- see the AEAD comment on handleReceiveStatic -- so this only
    // holds the keys for the day that lands; last_error()/verify_e()/
    // verify_l() are what a bench log reports meanwhile.
    KeyStore key_store;
    // Constructed in init() once cfg.bzr (settings) is known — same reason
    // as array_sensor above. Public (not alongside motor_left/imu below)
    // because state functions in src/robot/ call junkebox->play() directly,
    // the same way 03_Calibrate.cpp calls array_sensor->calibrate().
    std::optional<Junkebox> junkebox;
    USBMassStorage usb_storage;
    OTAUpdater ota;
    // Always built (the old ENABLE_SYSTEM_MONITOR build flag is gone):
    // observability that a commented-out line in platformio.ini can switch
    // off is observability the field build never has. To silence only the
    // periodic report, set timers.sysmon_freq_ms = 0 -- the on-demand
    // "sysmon"/"sys" commands keep working either way.
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

    // DEBUG when init() found button 3 / cfg.btn2 (OTA) or button 2 /
    // cfg.btn1 (USB storage) held at boot and actually managed to start that
    // sub-mode; SETUP otherwise. cfg.btn0 can't be used for this: it is
    // GPIO0, the boot strapping pin -- holding it at reset drops the chip
    // into ROM download mode instead of running the firmware.
    // Read once by main.cpp, right after init() returns, to pick the state
    // machine's initial state — this is the boot-time equivalent of the
    // existing DEBUG-state shell commands ("ota start" / "storage expose"),
    // which are unaffected and still work the normal way.
    stateName bootState() const { return boot_state_; }

    // Keep selected shell responses out of the retained PSRAM log.
    void sendNextShellOutputDirect();
    bool consumeDirectShellOutputRequest();

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

    // save a instance of the ROBOT class to be used in the static functions
    static ROBOT* instance_;
    bool initialized = false;

    // Set in init() (see bootState() above); stays SETUP unless a boot-time
    // sub-mode button was held AND that sub-mode actually started.
    stateName boot_state_ = SETUP;

    // Guards configureCommunication(): init() can be retried by app_main's
    // `while (!robot.init())` loop after a later step fails (e.g. motor),
    // and wifi/esp-now bring-up is not safe to redo (esp_wifi_init(),
    // esp_now_init(), esp_now_add_peer()... all error/warn on a second call).
    bool communication_configured_ = false;

    // 16-byte opaque identity handed to ManifestResponder (topico 16),
    // derived from base_mac in configureProtocolIdentity() -- this robot has
    // no HELLO/session concept over ESP-NOW, so there is no other source for
    // a stable "source_uuid" to put in MANIFEST_DATA.
    std::uint8_t protocol_uuid_[16]{};

    // MANIFEST_DATA source_info block (BTP/docs/commands.md section 3.12),
    // built once by buildSourceInfo() from esp_app_get_description(),
    // esp_chip_info(), the running OTA partition and RobotSettings. Entries
    // borrow their strings: app-descriptor fields (static), settings buffers
    // (re-read live, so "settings -set identity ..." needs no reboot), and
    // slices of source_info_scratch_ for the formatted numbers. Manifest
    // Responder drops an entry whose value is empty, so an unconfigured
    // name/description simply does not appear.
    SourceInfoEntry source_info_entries_[ManifestResponder::kMaxSourceInfoEntries]{};
    std::size_t source_info_count_ = 0U;
    char source_info_scratch_[96]{};
    void buildSourceInfo();

    // This boot's identity, computed once by configureProtocolIdentity() (MAC
    // + a fresh random boot_id from NVS) and consumed later by
    // bindProtocolTransport(), which is what actually builds node_ -- see
    // node_'s own comment for why the two cannot happen at the same time.
    std::uint32_t protocol_source_id_ = 0U;
    std::uint32_t protocol_boot_id_ = 0U;

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

    // configure the wifi and the esp-now settings for the robot
    bool configureCommunication();
    bool configureProtocolIdentity();
    void processCommandRequest(const btp::Header& header,
                               btp::ByteView payload, bally::Channel channel);
    void processManifestRequest(const btp::Header& header,
                                btp::ByteView payload);
    // Topico 17: CONTROL/SUBSCRIBE and CONTROL/UNSUBSCRIBE, routed the same
    // way processManifestRequest already is (see handleReceiveStatic's
    // object_id switch). `channel` is threaded through the same way
    // processCommandRequest's already is, since topico 31.2 widened these
    // two object_ids to channel B alongside COMMAND.
    void processSubscribeRequest(const btp::Header& header, btp::ByteView payload,
                                 bally::Channel channel);
    void processUnsubscribeRequest(const btp::Header& header, btp::ByteView payload,
                                   bally::Channel channel);
    void dispatchDecoded(const btp::Header& header, btp::ByteView payload,
                        bally::Channel channel);
    void sampleTelemetry();

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

    // "link"/"telemetry"/"sec": the radio, the protocol and the keys, read
    // from the shell. All three stay here for one reason: every library they
    // touch (TxScheduler, RxRouter, CommandProcessor, TelemetryPublisher,
    // KeyStore) is compiled by env:native for its unit suite, where TinyShell
    // does not exist -- see BallyRobotShell.cpp's header comment.
    void registerLinkCommands();
    void registerTelemetryCommands();
    void registerSecurityCommands();

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
        // Only meaningful when cache_slot == kTerminalCommandSlot: the
        // TERMINAL origin runShell() routes the captured output back to via
        // TerminalResponder::deliver_command_output().
        uint32_t terminal_source_id = 0U;
        uint32_t terminal_boot_id = 0U;
    };

    // cache_slot for a command that did not come from the radio: a job firing
    // or a line of an SD script. Deliberately outside CommandProcessor's
    // kCacheCapacity (16) so it can never collide with a real reservation.
    // runShell() uses it to skip the COMMAND_RESULT step — there is no
    // request to correlate a result with.
    static constexpr uint8_t kLocalCommandSlot = 0xFFU;
    // Same idea for a line typed at TraceView's terminal (topico 19bis):
    // no COMMAND_REQUEST to correlate, but its output is captured and mirrored
    // back to the origin as TERMINAL_OUT instead of only going out as LOG.
    static constexpr uint8_t kTerminalCommandSlot = 0xFEU;

    // Enqueue one shell line from inside the firmware. Returns false when the
    // queue is full (the caller decides whether to drop or retry — the job
    // scheduler drops, the script runner retries) or the line does not fit.
    bool submitLocalCommand(const char* command_line);
    static bool submitLocalCommandStatic(void* context, const char* command_line);

    // Enqueue one shell line from TraceView's terminal, tagged with the origin
    // so runShell() can hand its captured output back to TerminalResponder.
    bool submitTerminalCommand(uint32_t source_id, uint32_t boot_id,
                               const char* command_line);
    static bool submitTerminalCommandStatic(void* context, uint32_t source_id,
                                            uint32_t boot_id, const char* command_line);

    // The shell task's own handle, captured at the top of runShell(). Logger's
    // command-output capture is gated on it so only shell-task log lines
    // (i.e. the running command's own output) are mirrored to the terminal.
    TaskHandle_t shell_task_handle_ = nullptr;

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

    static constexpr size_t kCommandQueueLength = 10U;
    QueueHandle_t receivedDataQueue = nullptr;
    StaticQueue_t received_data_queue_control_{};
    uint8_t received_data_queue_storage_[
        kCommandQueueLength * sizeof(QueuedCommand)]{};

    // Receive pipeline: decode + CRC + reassembly + AEAD open, all now
    // btp::Node's job (library 2.16.0+; RxRouter's decode/reassemble half and
    // handleReceiveStatic's manual RadioSeal::open/open_e half both moved in
    // here). Node also lends its btp::Endpoint (node_->endpoint()) to
    // `protocol` via protocol.bind() -- see bindProtocolTransport() -- so
    // there is exactly one sequence counter for this robot's identity, shared
    // by every producer that sends through `protocol` and by whatever this
    // Node itself would send (it never does: cfg.send is left null, see
    // bindProtocolTransport()'s comment -- this Node exists to receive and to
    // own that one Endpoint, nothing else. No session (this robot has no
    // HELLO/session concept over ESP-NOW -- see SubscriptionResponder.h),
    // no served/learned catalogue (ManifestResponder keeps answering
    // MANIFEST_REQUEST by hand -- see its class comment: btp::Catalog cannot
    // carry a field's unit/description, which the wire manifest still needs).
    //
    // Deferred (std::optional, like array_sensor/junkebox/motor_left/... just
    // below): the identity node_ is built on is only known after
    // configureProtocolIdentity(), and bindProtocolTransport() (main.cpp's
    // setup_system_callbacks(), after init()) is what emplaces it. protocol_link_
    // itself is a plain member, constructed with the ROBOT -- node_ holds it by
    // reference, so it must outlive node_.
    //
    // Sizes match RxRouter's own (4 slots, 600 octets, 4000 ms) -- the two
    // ends of the radio link still need to tolerate the same loss and
    // reordering. SealBytes/ScratchBytes/Catalog* are still minimal here; the
    // btp::Node adoption (serve_catalog / publish) grows them.
    static constexpr std::size_t kNodeSlotCount = 4U;
    static constexpr std::size_t kNodeSlotBytes = 600U;
    static constexpr std::uint64_t kNodeReassemblyTimeoutMs = 4000U;
    RobotLink protocol_link_{*this};
    std::optional<btp::StaticNode<kNodeSlotCount, kNodeSlotBytes,
                                  /*SealBytes=*/16U, /*ScratchBytes=*/16U,
                                  /*CatalogTopics=*/1U, /*CatalogFields=*/1U,
                                  /*CatalogStringBytes=*/16U,
                                  /*MaxSubscriptions=*/1U, /*MaxCommands=*/1U,
                                  /*CommandBytes=*/16U>> node_;

    // Two tasks touch node_ once it exists, and it holds no lock: receive()
    // from the ESP-NOW receive callback, tick()'s reassembly sweep once a
    // second from routine() via publishStatus(). Same tolerated single-
    // writer-ish pattern RxRouter::Router documented; the worst case is
    // unchanged too.

    // Classifies one decoded header the same way for both protocolOpen()
    // (which key opens it) and dispatchDecoded() (which key answers it) --
    // one source of truth instead of two copies of channel_of_peer's call.
    // CONTROL/MANIFEST_REQUEST is deliberately NOT included: only the
    // dongle's own aggregation cache ever asks a robot for its manifest (see
    // ManifestResponder's class comment), so it always classifies C_Link.
    static bally::Channel classifyChannel(const btp::Header& header,
                                          std::uint32_t dongle_source_id) noexcept;

    // btp::NodeOpenFn bound to node_'s cfg.open: classifies by classifyChannel()
    // then opens under RadioSeal::open (C_Link) or open_e (B_Endpoint), both
    // fail-closed exactly as handleReceiveStatic's old stage two was. `ctx` is
    // the ROBOT singleton -- note_unauthorized() on a failed open is the one
    // side effect this callback has beyond returning false.
    static bool protocolOpen(void* ctx, const btp::Header& header,
                             std::uint16_t sealed_size, const std::uint8_t* sealed,
                             std::uint8_t* out_plaintext) noexcept;

    // The dongle's BTP source_id, derived once in configureProtocolIdentity()
    // from the same MAC_ADDR build flag handleReceiveStatic's radio prefilter
    // already uses, via the identical btp_command::source_id_from_mac() the
    // robot uses for its OWN identity. Never persisted, never a RobotSettings
    // field -- MAC_ADDR is already the one place this robot's build records
    // "who is my dongle". Stays 0U (a value intake() already rejects as an
    // invalid header field) when MAC_ADDR is not defined for this build.
    //
    // This is the only thing bally_channels.h::channel_of_peer(Vantage::Robot,
    // ...) needs beyond the header's own cleartext source_id, and is what
    // finally lets a COMMAND's plaintext be classified as channel B
    // (TraceView, key E) or channel C (dongle, key L) BEFORE RadioSeal::open
    // vs open_e is chosen -- see handleReceiveStatic.
    uint32_t dongle_source_id_ = 0U;

    // The state-machine task is the sole telemetry producer. The routine task
    // only consumes the publisher's bounded SPSC queue. The publish period is
    // no longer a fixed constant (topico 17): it comes from
    // TelemetryPublisher::topic_period_us(), which reflects whatever rate the
    // dongle's SUBSCRIBE actually granted (protocol.test's schema max is
    // still 50000 millihz = 20000us, see TelemetryPublisher.cpp's kSchemas).
    uint64_t next_protocol_test_us_ = 0U;
    uint64_t next_system_monitor_us_ = 0U;
    uint32_t protocol_test_counter_ = 0U;
    stateName last_telemetry_state_ = NONE;

    // CONTROL/STATUS (topico 17 PASSOS 8/9): a spontaneous message published
    // from the dedicated comms task (runComms()), decoupled from routine().
    // status_version=2 carries the per-topic block (subscriber count, aggregate
    // rate, bytes, drops).
    //
    // 500 ms, not 1 s: this frame is also the dongle's presence heartbeat for
    // hub.peers and its trigger to (re)prime the robot's manifest. Near the
    // motors ESP-NOW loses frames, so sending twice as often is what makes the
    // dongle reliably hear at least one per second and keeps a healthy robot
    // from flickering "offline". Still a small frame on a lightly used channel.
    static constexpr uint64_t kStatusPeriodUs = 500000ULL;
    uint64_t next_status_us_ = 0U;
    void publishStatus();

    // Zero point for "link -delta".
    //
    // The counters themselves are never reset, and there is deliberately no
    // reset_stats() on TxScheduler/btp::Receiver/CommandProcessor: commands.md
    // section 5 defines the STATUS counters as monotonic since boot, so
    // zeroing them would make any consumer computing a delta see a negative
    // one. A snapshot on this side gives the bench the same "since I started
    // watching" view without touching the wire contract.
    struct LinkStatsBaseline {
        bool     set = false;
        uint32_t uptime_ms = 0U;
        uint64_t frames_rx = 0U;
        uint64_t crc_errors = 0U;
        uint64_t decode_errors = 0U;
        TxScheduler::Stats       tx{};
        btp::Receiver::Stats     rx{};
        CommandProcessor::Stats  command{};
    };
    LinkStatsBaseline link_baseline_{};
    void captureLinkBaseline();

    // frames_rx (STATUS section 5): every octet stream the radio hands us is
    // one received frame attempt, counted here before the MAC prefilter or
    // node_->receive() even run -- neither is a Receiver::Stats counter, both
    // only see what actually reaches receive(). crc_errors/decode_errors/
    // reassembly_completed/timeouts/rejected all come from
    // node_->receiver().stats() now (read live in publishStatus() /
    // captureLinkBaseline()), so this is the only manually kept counter left.
    // Written only by the ESP-NOW receive callback (single writer), read by
    // publishStatus(); a relaxed atomic keeps the callback free of any lock.
    std::atomic<uint64_t> link_frames_rx_{0U};
};

#endif
