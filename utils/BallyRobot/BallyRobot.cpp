#include <BallyRobot.h>
#include <Settings.h>

// ESP-IDF Includes
#include <cstring>
#include <cstdlib>
#include <memory>
#include <new>
#include <esp_heap_caps.h>
#include <ctime>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "esp_chip_info.h"
#include "esp_ota_ops.h"
#include <cstdio>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <cmath>
#include <sys/time.h>
#ifndef PI
#define PI 3.14159265358979323846
#endif

// IMU I2C address (AD0 strapped low); gyro/accel unit conversions for the EKF.
static constexpr uint8_t IMU_I2C_ADDRESS = 0x68;
// Back to the ICM42688 driver's 400kHz default now that real pull-ups
// (1kOhm to 3.3V on SDA and SCL) are on the bus -- see the removed 100kHz
// mitigation this replaces: without them, the ESP32's own weak ~45kOhm
// internal pull-ups gave intermittent NACKs/garbled reads at 400kHz
// (scan_i2c: address ACKs but WHO_AM_I reads back 0xFF, or the whole bus
// goes briefly silent). If that symptom resurfaces on the bench, suspect
// the physical bus again before this constant.
static constexpr uint32_t IMU_I2C_CLOCK_HZ = 400'000;
// This retry margin predates the pull-up fix above (the bus used to
// occasionally drop the very first WHO_AM_I read/config write at boot too).
// Costs nothing at boot to keep, so it stays as a cushion against whatever
// transient noise the bus still sees before begin() declares the IMU absent.
static constexpr int      kImuInitAttempts    = 3;
static constexpr uint32_t kImuInitRetryDelayMs = 50;
static constexpr float   kDegToRad       = static_cast<float>(PI) / 180.0f;
static constexpr float   kGravityMss     = 9.81f;

// ==============================================================================
// STATIC MEMBER INITIALIZATION
// ==============================================================================

// active instance of the robot class
ROBOT* ROBOT::instance_ = nullptr;

Logger ROBOT::logger;

// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================

// ==============================================================================
// HARDWARE CONFIGURATION
// ==============================================================================

namespace {

constexpr ledc_mode_t kLedPwmMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kLedPwmTimer = LEDC_TIMER_2;
constexpr ledc_timer_bit_t kLedPwmResolution = LEDC_TIMER_8_BIT;
constexpr uint32_t kLedPwmFrequencyHz = 5000;
constexpr uint32_t kLedPwmMaxDuty = 255;

bool configureLedPwmTimer()
{
    ledc_timer_config_t timer = {};
    timer.speed_mode = kLedPwmMode;
    timer.timer_num = kLedPwmTimer;
    timer.duty_resolution = kLedPwmResolution;
    timer.freq_hz = kLedPwmFrequencyHz;
    timer.clk_cfg = LEDC_AUTO_CLK;

    return ledc_timer_config(&timer) == ESP_OK;
}

bool configureLedPwmChannel(uint8_t pin, uint8_t channel)
{
    const gpio_num_t gpio = static_cast<gpio_num_t>(pin);
    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio) ||
        channel >= static_cast<uint8_t>(LEDC_CHANNEL_MAX)) {
        return false;
    }

    ledc_channel_config_t config = {};
    config.speed_mode = kLedPwmMode;
    config.channel = static_cast<ledc_channel_t>(channel);
    config.timer_sel = kLedPwmTimer;
    config.gpio_num = pin;
    config.duty = 0;
    config.hpoint = 0;

    return ledc_channel_config(&config) == ESP_OK;
}

void setLedPwmDuty(uint8_t channel, float level)
{
    const uint32_t duty = static_cast<uint32_t>(level * kLedPwmMaxDuty + 0.5f);
    const ledc_channel_t ledc_channel = static_cast<ledc_channel_t>(channel);
    if (ledc_set_duty(kLedPwmMode, ledc_channel, duty) == ESP_OK) {
        ledc_update_duty(kLedPwmMode, ledc_channel);
    }
}

bool configureOutputPin(gpio_num_t pin, uint32_t initialLevel)
{
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
        return false;
    }

    if (gpio_reset_pin(pin) != ESP_OK) {
        return false;
    }

    if (gpio_set_direction(pin, GPIO_MODE_OUTPUT) != ESP_OK) {
        return false;
    }

    if (gpio_set_pull_mode(pin, GPIO_FLOATING) != ESP_OK) {
        return false;
    }

    if (gpio_set_level(pin, initialLevel) != ESP_OK) {
        return false;
    }

    return true;
}

bool configureInputPin(gpio_num_t pin, gpio_pull_mode_t pullMode)
{
    if (!GPIO_IS_VALID_GPIO(pin)) {
        return false;
    }

    if (gpio_reset_pin(pin) != ESP_OK) {
        return false;
    }

    if (gpio_set_direction(pin, GPIO_MODE_INPUT) != ESP_OK) {
        return false;
    }

    if (gpio_set_pull_mode(pin, pullMode) != ESP_OK) {
        return false;
    }

    return true;
}

} // namespace

bool ROBOT::configurePinsEarly()
{
    // O cartão SD deve iniciar desselecionado. CS é o único pino que
    // precisa existir antes de sd_card.begin() montar o cartão e permitir
    // a leitura de settings.conf, então continua fixo (include/Settings.h).
    return configureOutputPin(CS, 1);
}

bool ROBOT::configurePinsFromSettings()
{
    const SettingsData& cfg = settings.data();

    // --------------------------------------------------------
    // Saídas que devem iniciar desligadas/em nível baixo
    // --------------------------------------------------------
    const gpio_num_t out_pins[] = {
        static_cast<gpio_num_t>(cfg.led0),
        static_cast<gpio_num_t>(cfg.led1),
        static_cast<gpio_num_t>(cfg.led2),
        static_cast<gpio_num_t>(cfg.led3),

        static_cast<gpio_num_t>(cfg.bzr),

        // S0/S1/S2 (mux select) are intentionally not configured here —
        // ArraySensor's constructor owns them, together with the ADC
        // channel setup for cfg.sig, and runs right after this function.
        //
        // ain1/ain2/bin1/bin2 (H-bridge inputs) are likewise owned by
        // HBridge's constructor/init(), which configures them for LEDC PWM
        // right after motor_left/motor_right are constructed in init().
    };

    for (gpio_num_t pin : out_pins) {
        if (!configureOutputPin(pin, 0)) {
            return false;
        }
    }

    // --------------------------------------------------------
    // Entradas digitais
    // --------------------------------------------------------
    const gpio_num_t in_pins[] = {
        static_cast<gpio_num_t>(cfg.left),
        static_cast<gpio_num_t>(cfg.right),

        static_cast<gpio_num_t>(cfg.enc_a0),
        static_cast<gpio_num_t>(cfg.enc_a1),
        static_cast<gpio_num_t>(cfg.enc_b0),
        static_cast<gpio_num_t>(cfg.enc_b1)
    };

    for (gpio_num_t pin : in_pins) {
        if (!configureInputPin(pin, GPIO_FLOATING)) {
            return false;
        }
    }

    // --------------------------------------------------------
    // Botões
    // --------------------------------------------------------
    const gpio_num_t btn_pins[] = {
        static_cast<gpio_num_t>(cfg.btn1),
        static_cast<gpio_num_t>(cfg.btn2),
        static_cast<gpio_num_t>(cfg.btn0)
    };

    for (gpio_num_t pin : btn_pins) {
        if (!configureInputPin(pin, GPIO_PULLUP_ONLY)) {
            return false;
        }
    }

    // --------------------------------------------------------
    // Entradas analógicas
    // --------------------------------------------------------
    const gpio_num_t analog_pins[] = {
        static_cast<gpio_num_t>(cfg.sig),
        static_cast<gpio_num_t>(cfg.current_a),
        static_cast<gpio_num_t>(cfg.current_b)
    };

    for (gpio_num_t pin : analog_pins) {
        if (!configureInputPin(pin, GPIO_FLOATING)) {
            return false;
        }
    }

    // CH0..CH3 belong to the motors and CH4 to the buzzer. The three LED
    // channels share their own fixed-frequency timer; yellow stays on GPIO
    // because its calibrated gain is 1.0 and needs no PWM.
    if (!configureLedPwmTimer() ||
        !configureLedPwmChannel(cfg.led0, CH5) ||
        !configureLedPwmChannel(cfg.led1, CH6) ||
        !configureLedPwmChannel(cfg.led3, CH7)) {
        return false;
    }

    leds.setGain(LED_BLUE, 0.125f);
    leds.setGain(LED_GREEN, 0.125f);
    leds.setGain(LED_YELLOW, 1.0f);
    leds.setGain(LED_RED, 0.5f);

    return true;
}

// Sets up ADC oneshot reads for cfg.current_a/current_b -- see the member
// declaration comment in BallyRobot.h for why this is two independent
// handles and what happens when one fails to init. Same idiom
// ArraySensor::ArraySensor() already uses for its own "sig" pin.
//
// Must run AFTER array_sensor.emplace() (see its call site in init()):
// array_sensor's own ADC unit is resolved below (re-deriving the same
// mapping ArraySensor::ArraySensor() already made for cfg.sig) so a
// current-sense pin sharing that physical unit can be skipped entirely
// instead of attempting a second adc_oneshot_new_unit() on it. On the bench
// that second call did not fail cleanly the way a "unit already in use"
// error would -- it left the ADC in a state where every later read (this
// robot's own array_sensor reads included) ran pathologically long, which
// starved every other task on the core (every shell command hanging
// forever, even after a reboot). Skipping the call outright avoids the
// state entirely; sharing array_sensor's own handle for that channel is a
// possible follow-up, not attempted here.
void ROBOT::initCurrentSensors() {
    const SettingsData& cfg = settings.data();

    adc_unit_t array_sensor_unit{};
    adc_channel_t array_sensor_channel{};
    const bool has_array_sensor_unit =
        adc_oneshot_io_to_channel(static_cast<uint8_t>(cfg.sig), &array_sensor_unit,
                                  &array_sensor_channel) == ESP_OK;

    const struct { uint8_t pin; adc_oneshot_unit_handle_t* handle; adc_channel_t* channel; } kSensors[] = {
        {static_cast<uint8_t>(cfg.current_a), &current_a_adc_handle_, &current_a_adc_channel_},
        {static_cast<uint8_t>(cfg.current_b), &current_b_adc_handle_, &current_b_adc_channel_},
    };

    for (const auto& s : kSensors) {
        adc_unit_t unit{};
        if (adc_oneshot_io_to_channel(s.pin, &unit, s.channel) != ESP_OK) {
            continue;
        }

        if (has_array_sensor_unit && unit == array_sensor_unit) {
            ROBOT::logger.insert_logf(
                logType::WARN,
                "Current sense on GPIO%u: shares ADC unit %d with array_sensor's "
                "sig pin (GPIO%u) -- skipped, always reads 0",
                s.pin, static_cast<int>(unit), cfg.sig);
            *s.handle = nullptr;
            continue;
        }

        adc_oneshot_unit_init_cfg_t adc_config = {};
        adc_config.unit_id = unit;
        adc_config.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
        adc_config.ulp_mode = ADC_ULP_MODE_DISABLE;

        if (adc_oneshot_new_unit(&adc_config, s.handle) != ESP_OK) {
            ROBOT::logger.insert_logf(
                logType::WARN,
                "Current sense on GPIO%u: adc_oneshot_new_unit failed (unit %d)",
                s.pin, static_cast<int>(unit));
            *s.handle = nullptr;
            continue;
        }

        adc_oneshot_chan_cfg_t channel_config = {};
        channel_config.atten = ADC_ATTEN_DB_12;
        channel_config.bitwidth = ADC_BITWIDTH_12;
        if (adc_oneshot_config_channel(*s.handle, *s.channel, &channel_config) != ESP_OK) {
            adc_oneshot_del_unit(*s.handle);
            *s.handle = nullptr;
        }
    }
}

uint16_t ROBOT::readCurrentRaw(adc_oneshot_unit_handle_t handle, adc_channel_t channel) {
    if (handle == nullptr) return 0U;

    int raw_val = 0;
    if (adc_oneshot_read(handle, channel, &raw_val) != ESP_OK) {
        return 0U;
    }
    return static_cast<uint16_t>(raw_val);
}

// ==============================================================================
// CONTROL & LOGIC
// ==============================================================================

void ROBOT::setTimeLimit() {
    buttons.setTimeLimit(settings.data().delay_flags);
    sideSensors.setTimeLimit(settings.data().delay_flags);
}

void ROBOT::resetFlags() {
    buttons.checkFlagsDuration();
    sideSensors.checkFlagsDuration();
    leds.checkFlagsDuration();
    motors.checkFlagsDuration();
}

void ROBOT::setCoast(uint32_t duration_ms) {
    const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
    uint32_t deadline = now_ms + duration_ms;
    // 0 is the "not coasting" sentinel, so never land on it by accident.
    if (deadline == 0U) deadline = 1U;
    motors_coast_until_ms_.store(deadline, std::memory_order_release);
}

void ROBOT::clearCoast() {
    motors_coast_until_ms_.store(0U, std::memory_order_release);
}

void ROBOT::setOutputs() {
    int16_t left_pwm  = motors.getValue(MOTOR_LEFT_idx);
    int16_t right_pwm = motors.getValue(MOTOR_RIGHT_idx);

    // Arming is enforced here, not only where a command is accepted: whatever
    // is in the PWM flags, a disarmed robot outputs zero.
    if (!motors_armed_.load(std::memory_order_acquire)) {
        left_pwm  = 0;
        right_pwm = 0;
        clearCoast();
    }

    // Coast (both inputs low) is a third state PWM cannot express: applyPWM(0)
    // is an active brake. Honour the latch only while nothing is being driven
    // -- a real PWM command always wins -- and let it expire on its own.
    bool coasting = false;
    const uint32_t coast_until = motors_coast_until_ms_.load(std::memory_order_acquire);
    if (coast_until != 0U && left_pwm == 0 && right_pwm == 0) {
        const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
        // Signed subtraction keeps this valid across the 32-bit ms wrap, the
        // same way ScheduledDebugTest::poll() does it.
        if (static_cast<int32_t>(now_ms - coast_until) < 0) {
            coasting = true;
        } else {
            clearCoast();
        }
    }

    if (coasting) {
        motor_left->coast();
        motor_right->coast();
    } else {
        const esp_err_t left_result = motor_left->applyPWM(left_pwm);
        const esp_err_t right_result = motor_right->applyPWM(right_pwm);
        if (left_result != ESP_OK || right_result != ESP_OK) {
            logger.insert_logf(logType::ERRO,
                               "motor PWM failed left=0x%X right=0x%X values=%d,%d",
                               static_cast<unsigned>(left_result),
                               static_cast<unsigned>(right_result),
                               static_cast<int>(left_pwm),
                               static_cast<int>(right_pwm));
        }
    }

    // Green, red and blue are stronger than yellow on the board, so their
    // active flag level is attenuated by the per-output gain through PWM.
    // Yellow remains a regular full-level GPIO output.
    const SettingsData& cfg = settings.data();
    setLedPwmDuty(CH5, leds.getValue(LED_BLUE));
    setLedPwmDuty(CH6, leds.getValue(LED_GREEN));
    gpio_set_level(static_cast<gpio_num_t>(cfg.led2),
                   leds.getValue(LED_YELLOW) > 0.0f);
    setLedPwmDuty(CH7, leds.getValue(LED_RED));
}

bool ROBOT::canScheduleDebugTest() const {
    return StateMachine::current_state.load(std::memory_order_acquire) == DEBUG &&
           !usb_storage.is_active();
}

bool ROBOT::scheduleDebugTest(ScheduledDebugTest& test, uint32_t samples,
                              uint32_t interval_ms) {
    if (!canScheduleDebugTest()) return false;
    return test.schedule(samples, interval_ms);
}

bool ROBOT::anyDebugTestActive() const {
    return array_sensor_test_.active() || encoder_test_.active() ||
           imu_test_.active() || imu_i2c_test_.active();
}

void ROBOT::stopMotors() {
    motors.setValue(MOTOR_LEFT_idx, 0, settings.data().delay_flags);
    motors.setValue(MOTOR_RIGHT_idx, 0, settings.data().delay_flags);
}

void ROBOT::blinkErrorLeds() {
    const uint32_t half_period_ms = settings.data().error_blink_ms;
    const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);

    // On for one half-period, off for the other: only refresh the flags
    // during the "on" half, and let their own timeout (see Flags_out) turn
    // them off during the other half instead of clearing them here.
    if ((now_ms / half_period_ms) % 2 != 0) return;

    for (uint8_t i = 0; i < 4; ++i) leds.setFlag(i, half_period_ms);
}

void ROBOT::processDebug() {
    // DEBUG is a safe test state: keep both motor commands at zero on every
    // pass, while the normal outer task delay continues yielding the CPU.
    stopMotors();

    usb_storage.process(buttons.getFlags());
    if (usb_storage.is_active()) return;

    // T25b: ota.process()/updateTcpServerLifecycle() moved to routine(),
    // ticked every pass regardless of the current state -- comm_mode==TCP's
    // background connection (started once at boot, see ROBOT::init()) is
    // not something the operator "enters" from DEBUG, so DEBUG-only ticking
    // would leave it stalled for any robot that never opens DEBUG at all.
    // See routine()'s own comment for the rest of this reasoning.
    //
    // Only the gate stays here, and it changed from is_active() to
    // is_upload_session(): a real "ota start" upload session still blocks
    // DEBUG's own sensor tests below exactly as before (SD-card exclusivity,
    // no test running mid-upload), but comm_mode==TCP's background
    // connection -- which can sit at phase()==SERVING indefinitely -- no
    // longer permanently blocks them just by existing.
    if (ota.is_upload_session()) return;

    // Add one `if (test.poll()) { ... }` block per ScheduledDebugTest member
    // (H-bridge current, ...). Terminal-only, deliberately: these are live
    // feedback for whoever's watching the terminal session that started the
    // test (push_async_output addresses that same (source_id, boot_id)), not
    // a persisted record -- unlike kalman_log_test_ below (registerKalmanCommands'
    // "start_log", ROBOT::runEKF()), which IS meant to be retained/reviewed
    // later and stays LOG-only on purpose. Putting these in LOG too would
    // flood the retained ring (and TraceView's Robot Log widget) with
    // throwaway per-sample noise for the length of the test.
    if (array_sensor_test_.poll()) {
        const std::string text = array_sensor->debug();
        btpdev::printf("%s\n", text.c_str());
    }

    if (encoder_test_.poll()) {
        char text[96];
        // Fixed-width fields with a plain space separator, not '\t': same
        // reasoning as imu_test_ below -- a raw tab lands at whatever column
        // TraceView's terminal happens to be at (its 8-column tab-stop
        // expansion, serialterminalwidget.cpp), which moves every sample
        // because these counts change width (sign, digit count) as they
        // accumulate from one reading to the next.
        std::snprintf(text, sizeof(text), "Encoders: left=%+9lld right=%+9lld",
                      static_cast<long long>(encoder_left->getCount()),
                      static_cast<long long>(encoder_right->getCount()));
        btpdev::printf("%s\n", text);
    }

    if (imu_test_.poll()) {
        imu->getAGT();
        char text[192];
        // Fixed-width fields with a plain space separator, not '\t': a raw tab
        // lands at whatever column TraceView's terminal happens to be at (its
        // 8-column tab-stop expansion, serialterminalwidget.cpp), which moves
        // every sample because these values change width (sign, digit count)
        // from one reading to the next -- that's the "\t errado" misalignment.
        std::snprintf(text, sizeof(text),
                      "IMU: ax=%+7.2f ay=%+7.2f az=%+7.2f gx=%+7.2f gy=%+7.2f gz=%+7.2f t=%+7.2f",
                      imu->accX(), imu->accY(), imu->accZ(),
                      imu->gyrX(), imu->gyrY(), imu->gyrZ(), imu->temp());
        btpdev::printf("%s\n", text);
    }

    if (imu_i2c_test_.poll()) {
        const uint8_t who = imu->whoAmI();
        const bool    ok  = (who == 0x47);
        ok ? ++imu_i2c_ok_count_ : ++imu_i2c_fail_count_;
        const uint32_t total   = imu_i2c_ok_count_ + imu_i2c_fail_count_;
        const float    ok_pct  = total ? (100.0f * imu_i2c_ok_count_) / total : 0.0f;
        char text[144];
        std::snprintf(text, sizeof(text),
                      "IMU I2C check: %s (WHO_AM_I=0x%02X) | %lu ok / %lu fail (%.1f%% ok)",
                      ok ? "OK" : "FAIL", who,
                      static_cast<unsigned long>(imu_i2c_ok_count_),
                      static_cast<unsigned long>(imu_i2c_fail_count_), ok_pct);
        btpdev::printf("%s\n", text);
    }
}

void ROBOT::cancelDebugTests() {
    array_sensor_test_.cancel();
    encoder_test_.cancel();
    imu_test_.cancel();
    imu_i2c_test_.cancel();
    ota.cancel();
}

void ROBOT::sendNextShellOutputDirect() {
    direct_next_shell_output.store(true, std::memory_order_release);
}

bool ROBOT::consumeDirectShellOutputRequest() {
    return direct_next_shell_output.exchange(false,
                                             std::memory_order_acq_rel);
}

void ROBOT::flushLogsOnShutdown() {
    // instance_ is set at construction (well before app_main runs) and
    // esp_restart() can only be reached after that, so this is never null in
    // practice; the check is defensive, not load-bearing.
    if (instance_ == nullptr) return;

    // Best effort: there is nobody left to report a failure to by the time a
    // shutdown handler runs, and flush_to_sd() already declines safely on its
    // own (card not mounted, or currently owned by the USB host) -- see the
    // call site in init() for why that is enough here.
    char filename[SDFileInfo::MAX_NAME_LENGTH] = {};
    instance_->logger.flush_to_sd(instance_->sd_card, false, filename,
                                  sizeof(filename));
}

// ==============================================================================
// COMM_CONFIG menu (T25b, TAREFAS_TCP_BLE_ANDROID.txt: ETAPA 3B)
// ==============================================================================

namespace {

// Deliberately simpler than the interrupts task's own ANYEDGE ISR debounce
// (button_press_accepted() above, with its own timer array and reasoning
// comment): that one exists to keep a mechanical release from being
// misread as a fresh press system-wide, including while a button is
// driving the state machine's normal transitionTable. Here, a human is
// deliberately clicking through a menu one option at a time -- a single
// "already processed this edge" latch (was_low) plus a short minimum gap
// between ACCEPTED edges is enough to reject contact chatter without
// needing a second, release-side edge to re-arm anything.
constexpr uint32_t kCommConfigDebounceMs = 200U;
// ~30s with no button press exits COMM_CONFIG (without saving) and reboots,
// so the robot is never stuck in this menu if the operator walks away —
// same safety-net reasoning as OTA's own button-cancels-scan behaviour.
constexpr uint32_t kCommConfigInactivityTimeoutMs = 30000U;
// Refreshed every commConfigTick() call (see blinkErrorLeds()'s own "call
// every pass" pattern for LEDs driven this way) -- comfortably longer than
// one state-machine task pass so the selected LED reads as solid on, short
// enough that the un-refreshed other three visibly turn off within a
// fraction of a second of the selection changing.
constexpr uint32_t kCommConfigLedHoldMs = 200U;

// LED index for each comm_mode option, in the same 0..3 = ESP-NOW/TCP/BLE/
// serial order RobotSettings.h documents (not specified by T25b beyond "one
// of the 4 LEDs per option"): blue ESP-NOW, green TCP, yellow BLE, red
// serial (USB CDC).
constexpr uint8_t kCommConfigLedForOption[4] = {LED_BLUE, LED_GREEN,
                                                LED_YELLOW, LED_RED};

bool commConfigDebouncedPress(gpio_num_t pin, bool& was_low,
                              uint32_t& last_edge_ms, uint32_t now_ms) {
    const bool low = gpio_get_level(pin) == 0; // pull-up wiring: LOW = pressed
    bool accepted = false;
    if (low && !was_low && (now_ms - last_edge_ms) >= kCommConfigDebounceMs) {
        accepted = true;
        last_edge_ms = now_ms;
    }
    was_low = low;
    return accepted;
}

// Deferred restart, 500ms one-shot -- same shape as BallyRobotShell.cpp's
// own scheduleRestart() and OTAUpdater::finish_update()'s reboot timer,
// deliberately duplicated rather than shared: those two already document
// why (OTAUpdater must not depend on the ROBOT composition root, and a new
// library for twelve lines of esp_timer boilerplate costs more than it
// saves). Same reasoning applies here -- this one lives in BallyRobot.cpp
// because commConfigTick() does. If one of the three changes, change all.
void scheduleCommConfigRestart() {
    esp_timer_handle_t reboot_timer = nullptr;
    const esp_timer_create_args_t timer_args = {
        .callback = [](void*) { esp_restart(); },
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "comm_config_reboot",
        .skip_unhandled_events = false,
    };
    esp_timer_create(&timer_args, &reboot_timer);
    esp_timer_start_once(reboot_timer, 500000);
}

} // namespace

stateName ROBOT::commConfigTick() {
    const SettingsData& cfg = settings.data();
    const gpio_num_t btn0 = static_cast<gpio_num_t>(cfg.btn0);
    const gpio_num_t btn1 = static_cast<gpio_num_t>(cfg.btn1);
    const gpio_num_t btn2 = static_cast<gpio_num_t>(cfg.btn2);
    const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);

    if (!comm_config_active_) {
        comm_config_active_ = true;
        comm_config_restart_scheduled_ = false;
        comm_config_selected_ = (cfg.comm_mode <= 3) ? cfg.comm_mode : 0U;
        comm_config_last_activity_ms_ = now_ms;
        // Prime the edge trackers to the CURRENT level -- see the field
        // comments in BallyRobot.h for why.
        comm_config_btn0_was_low_ = gpio_get_level(btn0) == 0;
        comm_config_btn1_was_low_ = gpio_get_level(btn1) == 0;
        comm_config_btn2_was_low_ = gpio_get_level(btn2) == 0;
        logger.insert_logf(
            logType::INFO,
            "COMM_CONFIG: entered (btn1=next btn2=prev btn0=confirm), "
            "current comm_mode=%u",
            static_cast<unsigned>(comm_config_selected_));
    }

    // Keep the selected LED solid on; the other three are simply never
    // refreshed here and expire on their own (Flags_out::
    // checkFlagsDuration(), driven by resetFlags() every routine() pass).
    leds.setFlag(kCommConfigLedForOption[comm_config_selected_], kCommConfigLedHoldMs);

    // A reboot is already scheduled (confirm or timeout, below) -- nothing
    // left to do but keep the LED refreshed above until esp_restart() fires.
    if (comm_config_restart_scheduled_) return COMM_CONFIG;

    bool activity = false;

    if (commConfigDebouncedPress(btn1, comm_config_btn1_was_low_,
                                 comm_config_last_btn1_edge_ms_, now_ms)) {
        comm_config_selected_ = static_cast<uint8_t>((comm_config_selected_ + 1) % 4U);
        activity = true;
        logger.insert_logf(logType::INFO, "COMM_CONFIG: next -> %u",
                           static_cast<unsigned>(comm_config_selected_));
    }
    if (commConfigDebouncedPress(btn2, comm_config_btn2_was_low_,
                                 comm_config_last_btn2_edge_ms_, now_ms)) {
        comm_config_selected_ = static_cast<uint8_t>((comm_config_selected_ + 3U) % 4U);
        activity = true;
        logger.insert_logf(logType::INFO, "COMM_CONFIG: prev -> %u",
                           static_cast<unsigned>(comm_config_selected_));
    }
    if (commConfigDebouncedPress(btn0, comm_config_btn0_was_low_,
                                 comm_config_last_btn0_edge_ms_, now_ms)) {
        char value[4];
        std::snprintf(value, sizeof(value), "%u",
                      static_cast<unsigned>(comm_config_selected_));
        const bool set_ok = settings.set("comm", "comm_mode", value);
        const bool saved = set_ok && settings.save(sd_card);
        logger.insert_logf(
            logType::INFO,
            "COMM_CONFIG: confirmed comm_mode=%u set=%d saved=%d, rebooting to apply",
            static_cast<unsigned>(comm_config_selected_), set_ok ? 1 : 0,
            saved ? 1 : 0);
        comm_config_restart_scheduled_ = true;
        scheduleCommConfigRestart();
        return COMM_CONFIG;
    }

    if (activity) {
        comm_config_last_activity_ms_ = now_ms;
    } else if (now_ms - comm_config_last_activity_ms_ >= kCommConfigInactivityTimeoutMs) {
        logger.insert_log(
            logType::WARN,
            "COMM_CONFIG: inactivity timeout, rebooting without saving");
        comm_config_restart_scheduled_ = true;
        scheduleCommConfigRestart();
    }

    return COMM_CONFIG;
}

void ROBOT::checkStateMachine() {
    if ((uint32_t)(esp_timer_get_time() / 1000ULL) - instance_->stateMachineTimer > instance_->settings.data().delay_flags) {
        ROBOT::machine.next(ROBOT::buttons.getFlags());
        instance_->stateMachineTimer = (uint32_t)(esp_timer_get_time() / 1000ULL);
    }
}

namespace {
struct SoundTrigger {
    stateName from_state; // NONE means "from any state" — never a live state
                          // past boot (see StateMachine::current_state.store(SETUP, ...)
                          // in main.cpp), so it's safe to reuse as a wildcard.
    stateName to_state;
    BuiltinSound sound;
};

// FROM STATE (NONE = any) | TO STATE | SOUND
constexpr SoundTrigger kSoundTriggerTable[] = {
{ NONE, ERROR, BuiltinSound::Error },
};
} // namespace

// Diffs buttons/sideSensors/state against the previous routine() pass and
// triggers the matching Junkebox::BuiltinSound — see kSoundTriggerTable
// above for state transitions. previous_buttons_/previous_side_sensors_/
// previous_state_ (BallyRobot.h) are plain copies, not references: updating
// them here never touches buttons/sideSensors/StateMachine, they only exist
// to be diffed against on the next pass.
//
// Junkebox::play() always interrupts whatever is currently queued/playing,
// so if more than one of these fires in the same pass, only the last
// play() call below is actually heard — the order here (button, then side
// sensor, then state) is a deliberate priority, state being the most
// significant event.
void ROBOT::updateSoundFeedback() {
    const uint8_t current_buttons = buttons.getFlags();
    const uint8_t current_side_sensors = sideSensors.getFlags();
    const stateName current_state =
        static_cast<stateName>(StateMachine::current_state.load(std::memory_order_acquire));

    // Button press edge: any bit that just turned on since the last pass —
    // buttons already self-clears each flag after settings.delay_flags (see
    // resetFlags()/Flags_in::checkFlagsDuration), so a held button only
    // edges once.
    if ((current_buttons & ~previous_buttons_) != 0) {
        junkebox->play(BuiltinSound::Click);
    }

    // Side-sensor edge: same rising-edge logic as buttons, own sound —
    // side sensors can trigger a lot faster/more often on a line-following
    // pass, so Ping is deliberately shorter than Click.
    //if ((current_side_sensors & ~previous_side_sensors_) != 0) {
    //    junkebox->play(BuiltinSound::Ping);
    //}

    // State transition: first table row matching (from, to) wins.
    if (current_state != previous_state_) {
        for (const SoundTrigger& trigger : kSoundTriggerTable) {
            if (trigger.to_state == current_state &&
                (trigger.from_state == NONE || trigger.from_state == previous_state_)) {
                junkebox->play(trigger.sound);
                break;
            }
        }
    }

    previous_buttons_ = current_buttons;
    previous_side_sensors_ = current_side_sensors;
    previous_state_ = current_state;
}

// ==============================================================================
// EKF & SENSORS
// ==============================================================================

void ROBOT::initEKF() {
    const SettingsData& cfg = settings.data();

    float x0[3] = {0.0f, 0.0f, 0.0f};
    float P0[3][3] = {
        {cfg.initial_p, 0.0f, 0.0f},
        {0.0f, cfg.initial_p, 0.0f},
        {0.0f, 0.0f, cfg.initial_p}
    };
    // Q and R are handed to TinyEKF's constructor and become const for the
    // filter's lifetime (see TinyEKF.h): the only way to change ekf_noise.*
    // is to reconstruct the whole filter, which is what re-running this
    // function does. "settings -apply ekf_noise" calls this live (see
    // registerSettingsAppliers()), with the EKF task suspended for the
    // duration -- state resets to zero (x0 above) either way, at boot or live.
    const TinyEKF::StateMat Q{{
        {cfg.v_noise, 0.0f,  0.0f},
        {0.0f,  cfg.w_noise, 0.0f},
        {0.0f,  0.0f,  cfg.b_noise}
    }};
    const TinyEKF::MeasureMat R{{
        {cfg.enc_noise, 0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, cfg.enc_noise, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, cfg.gyro_noise, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, cfg.accel_noise, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f, cfg.accel_noise}
    }};

    EKF.emplace(x0, P0, Q, R);
}

void ROBOT::registerSettingsAppliers() {
    // "timers": sample_micros needs the running EKF timer restarted with the
    // new period; timezone needs setenv/tzset applied directly rather than
    // waiting for the next "logger -set_datetime" to happen to read it.
    // delay_flags needs nothing here -- checkStateMachine() already reads
    // settings.data() fresh on every pass, so it is live already.
    settings.register_applier("timers", [this](const SettingsData& cfg) -> bool {
        bool ok = true;
        if (ekf_timer_handle_ != nullptr) {
            if (esp_timer_restart(ekf_timer_handle_, cfg.sample_micros) != ESP_OK) {
                ok = false;
            }
        }
        // Empty is a real, if unusual, value here: setenv() would happily
        // clear TZ to "", which tzset() then reads as UTC. That silently
        // changes every future log timestamp's zone, so refuse it instead.
        if (cfg.timezone[0] != '\0') {
            setenv("TZ", cfg.timezone, 1);
            tzset();
        } else {
            ok = false;
        }
        return ok;
    });

    // "logger": mutex timeout is documented as unused (Logger hardcodes its
    // own in wait_for_mutex()), so only the flush pacing needs pushing in.
    settings.register_applier("logger", [this](const SettingsData& cfg) -> bool {
        logger.set_flush_limits(cfg.max_chunks_per_flush, cfg.block_size);
        return true;
    });

    // "ota": configure() is documented safe to call any time after begin()
    // (OTAUpdater.h) -- it only re-copies the tuning struct and re-snprintf's
    // the three string fields. One real subtlety: espnow_channel here only
    // changes what OTA restores the radio TO on cancel(); the channel
    // ESP-NOW is actually running on right now was fixed at boot by
    // configureCommunication() and does not move without a reboot.
    settings.register_applier("ota", [this](const SettingsData& cfg) -> bool {
        ota.configure(OtaTuning{
            .led_step_ms        = cfg.ota_led_step_ms,
            .led_hold_ms        = cfg.ota_led_hold_ms,
            .led_fail_hold_ms   = cfg.ota_led_fail_hold_ms,
            .connect_timeout_ms = cfg.ota_connect_timeout_ms,
            .retry_scan_ms      = cfg.ota_retry_scan_ms,
            .espnow_channel     = cfg.espnow_channel,
            .hostname           = cfg.ota_hostname,
            .instance_name      = cfg.ota_instance_name,
            .password           = cfg.ota_password,
        });
        return true;
    });

    // "ekf_noise": Q/R are const for TinyEKF's lifetime (see initEKF()), so
    // the only way to apply a change is to reconstruct the whole filter. The
    // EKF task is suspended for that window -- it only ever touches EKF from
    // inside runEKF(), so this is the one point of exclusion that matters.
    // A notification that arrives from sampleEKF() while suspended is not
    // lost (FreeRTOS records it against the task, not the ready queue), so
    // at worst one predict/update right after resume runs against the
    // instant the state reset to zero -- not a crash, one stale-looking
    // sample.
    settings.register_applier("ekf_noise", [this](const SettingsData&) -> bool {
        if (ekf_task_handle == nullptr) return false;
        vTaskSuspend(ekf_task_handle);
        initEKF();
        vTaskResume(ekf_task_handle);
        return true;
    });

    // "kinematics": encoder_ppr/wheel_radius are already read fresh on every
    // call (getVelocitiesFromEncoders(), "motion -limits") -- registered as a
    // no-op so "settings -apply kinematics" answers "applied", not the
    // misleading "requires a reboot" it would get with no applier at all.
    settings.register_applier("kinematics", [](const SettingsData&) -> bool {
        return true;
    });

    // "error": error_blink_ms is read fresh on every blinkErrorLeds() call.
    // Same no-op reasoning as "kinematics" above.
    settings.register_applier("error", [](const SettingsData&) -> bool {
        return true;
    });
}

void ROBOT::getVelocitiesFromEncoders(float& linear_speed, float& angular_speed) {
    const SettingsData& cfg = settings.data();
    // getCountDiff() consumes the delta since its last call -- read each
    // encoder exactly once here and derive both speeds from that same
    // reading (see the declaration comment in BallyRobot.h).
    const float left_speed = encoder_left->getCountDiff()/cfg.encoder_ppr * cfg.wheel_radius * 2.0f * PI * settings.freq_ekf();
    const float right_speed = encoder_right->getCountDiff()/cfg.encoder_ppr * cfg.wheel_radius * 2.0f * PI * settings.freq_ekf();
    linear_speed = (left_speed + right_speed) / 2.0f;
    angular_speed = (right_speed - left_speed) / EKF_WHEEL_BASE;
}

void ROBOT::sampleEKF(void *param) {
    // save the pwm values to the control input vector for the EKF (PWM is
    // not a "sensor" reading, it stays direct)
    instance_->control_input[0] = static_cast<float>(instance_->motors.getValue(MOTOR_RIGHT_idx));
    instance_->control_input[1] = static_cast<float>(instance_->motors.getValue(MOTOR_LEFT_idx));

    // Fill sensor_snapshot_ first -- the single source of truth both the EKF
    // (below) and "robot.sensors" telemetry (sampleTelemetry()) read from.
    SensorSnapshot& snap = instance_->sensor_snapshot_;

    float linear_speed = 0.0f;
    float angular_speed = 0.0f;
    instance_->getVelocitiesFromEncoders(linear_speed, angular_speed);
    snap.enc_linear_speed.store(linear_speed, std::memory_order_relaxed);
    snap.enc_angular_speed.store(angular_speed, std::memory_order_relaxed);

    // Skipped while the IMU never answered at boot (imu_ready_ == false):
    // retrying an I2C transaction against a disconnected sensor here would
    // block this same timer callback on an I2C timeout every tick.
    if (instance_->imu_ready_) {
        instance_->imu->getAGT();
        snap.gyro_z.store(instance_->imu->gyrZ() * kDegToRad, std::memory_order_relaxed);
        snap.accel_x.store(instance_->imu->accX() * kGravityMss, std::memory_order_relaxed);
        snap.accel_y.store(instance_->imu->accY() * kGravityMss, std::memory_order_relaxed);
    } else {
        snap.gyro_z.store(0.0f, std::memory_order_relaxed);
        snap.accel_x.store(0.0f, std::memory_order_relaxed);
        snap.accel_y.store(0.0f, std::memory_order_relaxed);
    }

    // Current sense and the array sensor are NOT read here on purpose: they
    // are plotting-only fields the EKF never touches (see SensorSnapshot's
    // own comment), and sampled instead from sampleTelemetry() at whatever
    // (much slower) rate "robot.sensors" is actually subscribed at -- doing
    // up to 10 ADC reads with mux-settle delays every single 1kHz tick here
    // caused this task to run long enough to starve every other task on the
    // core (observed on the bench as every shell command hanging forever).
    // See sampleTelemetry()'s own comment.

    // Build the EKF's own measurement vector from the snapshot just written
    // above -- same task, same instant, so a relaxed load back is exact (no
    // cross-task race to guard against here).
    instance_->measurement[0] = snap.enc_linear_speed.load(std::memory_order_relaxed);
    instance_->measurement[1] = snap.enc_angular_speed.load(std::memory_order_relaxed);
    instance_->measurement[2] = snap.gyro_z.load(std::memory_order_relaxed);
    instance_->measurement[3] = snap.accel_x.load(std::memory_order_relaxed);
    instance_->measurement[4] = snap.accel_y.load(std::memory_order_relaxed);

    // notify the EKF task that new measurements are available
    if (instance_->ekf_task_handle != nullptr)
        xTaskNotifyGive(instance_->ekf_task_handle);
}

void ROBOT::runEKF(void *param) {
    (void)param; // Suppress unused parameter warning

    // save the handle of this task to the robot instance to be able to wake it up from the sampleEKF function
    instance_->ekf_task_handle = xTaskGetCurrentTaskHandle();

    while (true) {
        // wait to be notified by the sampleEKF function that new measurements are available
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // EKF predict/update temporarily disabled: nothing outside this task
        // consumes get_state() except the debug log right below and a shell
        // status command (BallyRobotShell.cpp) -- motor control does not
        // depend on the filter's estimate today -- so this is safe to park
        // while the sensor-sampling rework (possibly I2C->I3C for the IMU)
        // and a revisit of the filter itself are pending. x[] simply stays
        // at its last (or initial) value until this is re-enabled.
        // instance_->EKF->predict(instance_->control_input);
        // instance_->EKF->update(instance_->control_input, instance_->measurement);

        // Read back x[] right after writing it, in this same task — no
        // cross-task race with whoever else might read the filter state.
        if (instance_->kalman_log_test_.poll()) {
            ROBOT::logger.insert_logf(
                logType::INFO,
                "KALMAN v=%.4f w=%.4f b=%.4f | meas enc_v=%.4f enc_w=%.4f gyro=%.4f accx=%.4f accy=%.4f | pwm R=%.0f L=%.0f",
                instance_->EKF->get_state(0), instance_->EKF->get_state(1), instance_->EKF->get_state(2),
                instance_->measurement[0], instance_->measurement[1], instance_->measurement[2],
                instance_->measurement[3], instance_->measurement[4],
                instance_->control_input[0], instance_->control_input[1]);
        }
    }
}

// ==============================================================================
// LOCAL COMMANDS, JOBS AND SD SCRIPTS
// ==============================================================================

bool ROBOT::submitLocalCommand(const char* command_line) {
    // Never blocks: this runs on the routine task. A full queue is a "no",
    // not a wait. The output goes to TraceView's log panel.
    return device_ != nullptr && device_->run(command_line);
}

bool ROBOT::submitLocalCommandStatic(void* context, const char* command_line) {
    ROBOT* self = static_cast<ROBOT*>(context);
    if (self == nullptr) return false;
    return self->submitLocalCommand(command_line);
}

void ROBOT::pollJobs() {
    jobs.poll(static_cast<uint64_t>(esp_timer_get_time() / 1000ULL),
              StateMachine::current_state.load(std::memory_order_acquire));
}

// Generous per-line estimate ("interval," + two uint32 fields + comma +
// kMaxCommandLength + newline), times the largest possible job count.
static constexpr size_t kJobSaveBufferBytes =
    JobScheduler::kMaxJobs * (JobScheduler::kMaxCommandLength + 48U);

bool ROBOT::saveJobs() {
    if (!sd_card.is_mounted()) return false;

    JobScheduler::JobView views[JobScheduler::kMaxJobs];
    const size_t count = jobs.list(views, JobScheduler::kMaxJobs);

    char buffer[kJobSaveBufferBytes];
    size_t offset = 0U;
    size_t persisted = 0U;

    for (size_t i = 0; i < count; ++i) {
        const JobScheduler::JobView& view = views[i];
        int written = 0;
        if (view.kind == JobScheduler::Kind::Interval) {
            written = snprintf(buffer + offset, sizeof(buffer) - offset,
                               "interval,%lu,%lu,%s\n",
                               static_cast<unsigned long>(view.interval_ms),
                               static_cast<unsigned long>(view.remaining),
                               view.command);
        } else if (view.kind == JobScheduler::Kind::OnStateEnter) {
            written = snprintf(buffer + offset, sizeof(buffer) - offset,
                               "on_state,%s,%s\n",
                               StateMachine::stateToString(view.state),
                               view.command);
        } else {
            // Kind::Once: see JOB_SAVE_FILE's comment in BallyRobot.h.
            continue;
        }
        if (written <= 0 ||
            static_cast<size_t>(written) >= sizeof(buffer) - offset) {
            break;  // Out of room: keep what already fit, not a torn line.
        }
        offset += static_cast<size_t>(written);
        ++persisted;
    }

    if (!sd_card.write_file(JOB_SAVE_FILE, buffer, offset)) return false;
    logger.insert_logf(logType::INFO, "saved %u/%u job(s) to " JOB_SAVE_FILE,
                       static_cast<unsigned>(persisted),
                       static_cast<unsigned>(count));
    return true;
}

bool ROBOT::loadJobs() {
    if (!sd_card.is_mounted() || !sd_card.file_exists(JOB_SAVE_FILE)) {
        return false;
    }

    char buffer[kJobSaveBufferBytes + 1U];
    size_t read_bytes = 0U;
    if (!sd_card.read_file(JOB_SAVE_FILE, buffer, sizeof(buffer) - 1U,
                           &read_bytes)) {
        return false;
    }
    buffer[read_bytes] = '\0';

    size_t restored = 0U;
    size_t cursor = 0U;
    while (cursor < read_bytes) {
        size_t end = cursor;
        while (end < read_bytes && buffer[end] != '\n' && buffer[end] != '\r') {
            ++end;
        }
        buffer[end] = '\0';
        char* line = buffer + cursor;
        cursor = end + 1U;
        if (line[0] == '\0') continue;

        uint8_t id = 0U;
        if (std::strncmp(line, "interval,", 9U) == 0) {
            char* rest = line + 9U;
            char* comma1 = std::strchr(rest, ',');
            char* comma2 = comma1 != nullptr ? std::strchr(comma1 + 1, ',') : nullptr;
            if (comma1 == nullptr || comma2 == nullptr) continue;
            *comma1 = '\0';
            *comma2 = '\0';
            const uint32_t interval_ms =
                static_cast<uint32_t>(std::strtoul(rest, nullptr, 10));
            const uint32_t remaining =
                static_cast<uint32_t>(std::strtoul(comma1 + 1, nullptr, 10));
            if (jobs.schedule_interval(interval_ms, remaining, comma2 + 1,
                                       &id) == JobScheduler::Error::Ok) {
                ++restored;
            }
        } else if (std::strncmp(line, "on_state,", 9U) == 0) {
            char* rest = line + 9U;
            char* comma = std::strchr(rest, ',');
            if (comma == nullptr) continue;
            *comma = '\0';
            const uint8_t state = StateMachine::stateFromString(rest);
            if (jobs.schedule_on_state(state, comma + 1, &id) ==
                JobScheduler::Error::Ok) {
                ++restored;
            }
        }
    }

    if (restored > 0U) {
        logger.insert_logf(logType::INFO, "restored %u job(s) from " JOB_SAVE_FILE,
                           static_cast<unsigned>(restored));
    }
    return true;
}

bool ROBOT::startScript(const char* path) {
    if (path == nullptr || path[0] == '\0') return false;
    if (!sd_card.is_mounted()) return false;
    if (!sd_card.file_exists(path)) return false;

    size_t read_bytes = 0;
    if (!sd_card.read_file(path, script_buffer_, kScriptMaxBytes, &read_bytes)) {
        return false;
    }

    // read_file stops at capacity without saying whether more was there, so a
    // file at exactly the cap is reported as truncated too. Erring toward the
    // warning is right: silently running the first 2 KB of a longer script is
    // the worse failure.
    if (read_bytes >= kScriptMaxBytes) {
        logger.insert_logf(logType::WARN,
                           "script %s reached the %u byte limit; it may be truncated",
                           path, static_cast<unsigned>(kScriptMaxBytes));
    }

    script_buffer_[read_bytes] = '\0';
    script_size_   = read_bytes;
    script_cursor_ = 0U;
    script_line_   = 0U;
    script_active_ = true;
    return true;
}

void ROBOT::pollScript() {
    if (!script_active_) return;

    // Skip blank lines and comments without spending a pass on each: only a
    // line that actually produces a command costs one routine() iteration.
    while (script_cursor_ < script_size_) {
        const size_t start = script_cursor_;

        size_t end = start;
        while (end < script_size_ && script_buffer_[end] != '\n' &&
               script_buffer_[end] != '\r') {
            ++end;
        }

        // Borrow the buffer for one line: terminate in place, then restore the
        // separator so the cursor arithmetic below stays valid.
        const char separator = script_buffer_[end];
        script_buffer_[end] = '\0';

        char* line = &script_buffer_[start];
        while (*line == ' ' || *line == '\t') ++line;

        char* tail = &script_buffer_[end];
        while (tail > line && (tail[-1] == ' ' || tail[-1] == '\t')) {
            *--tail = '\0';
        }

        const bool runnable = (line[0] != '\0') && (line[0] != '#');
        bool submitted = false;
        bool queue_full = false;

        if (runnable) {
            if (JobScheduler::is_job_command(line)) {
                // Same reason a job may not schedule jobs: a script that runs
                // scripts (or fills the job table) has no bound.
                logger.insert_logf(logType::WARN,
                                   "script line %u ignored: a script may not run job commands",
                                   static_cast<unsigned>(script_line_ + 1U));
            } else if (submitLocalCommand(line)) {
                submitted = true;
            } else {
                queue_full = true;
            }
        }

        script_buffer_[end] = separator;

        if (queue_full) {
            // Unlike a job, a script line is retried: a boot script that runs
            // three quarters of the way is worse than one that takes a few
            // extra passes. The cursor is left where it is.
            return;
        }

        ++script_line_;
        script_cursor_ = end;
        // Consume the line separator, CRLF included.
        if (script_cursor_ < script_size_ && script_buffer_[script_cursor_] == '\r') {
            ++script_cursor_;
        }
        if (script_cursor_ < script_size_ && script_buffer_[script_cursor_] == '\n') {
            ++script_cursor_;
        }

        if (submitted) return;  // one command per pass
    }

    script_active_ = false;
    logger.insert_logf(logType::INFO, "script finished lines=%u",
                       static_cast<unsigned>(script_line_));
}

// ==============================================================================
// SHELL REGISTRATION
// ==============================================================================
// startWrappers(), registerSystemCommands(), registerRobotIOCommands(),
// registerKalmanCommands() and registerDebugCommands() live in
// BallyRobotShell.cpp -- same class, separate translation unit. See the
// header comment there for why the shell side is deliberately ESP-IDF-only.

// ==============================================================================
// MAIN TASKS & INITIALIZATION
// ==============================================================================

// ------------------------------------------------------------------------------
// BUTTON DEBOUNCE (used by the three button ISRs in initInterruptions)
// ------------------------------------------------------------------------------
// Mechanical buttons chatter on release just as much as on press: letting go
// bounces the contact, and every bounce that pulls the line back down is
// another falling edge. On a plain NEGEDGE interrupt those were
// indistinguishable from a new press -- which is what made OTA quit the
// moment the finger came off the boot button (OTAUpdater::process() cancels
// on any button flag).
//
// So the interrupt is armed on ANY edge and filtered here instead: every
// edge, in either direction, restarts the guard window, and only a falling
// edge that arrives after a full quiet window counts as a press. That is
// what makes a release safe no matter how long the button was held -- its
// first bounce restarts the window from the rising edge, so the chatter
// behind it can never reach setFlag().
namespace {

struct ButtonIsr {
    gpio_num_t pin;
    uint8_t    bit;
};

// Only ever touched from the button ISRs themselves (one context per pin,
// and same-pin interrupts don't nest), so no locking is needed.
DRAM_ATTR ButtonIsr        button_isr_cfg[3]        = {};
DRAM_ATTR volatile int64_t button_last_edge_us[3]   = {};
DRAM_ATTR volatile int64_t button_debounce_us       = 0;

// True only for a genuine press. Always stamps the edge, so the window
// slides along with the chatter instead of expiring in the middle of it.
bool IRAM_ATTR button_press_accepted(const ButtonIsr& btn) {
    const int64_t now      = esp_timer_get_time();
    const int64_t previous = button_last_edge_us[btn.bit];
    button_last_edge_us[btn.bit] = now;

    // Pull-up wiring: LOW is pressed, so a HIGH level here means this edge
    // was the release (or a bounce of it) -- never a press.
    if (gpio_get_level(btn.pin) != 0) return false;

    return (now - previous) >= button_debounce_us;
}

} // namespace

void ROBOT::initInterruptions(void *param){
    (void)param; // Suppress unused parameter warning

    // Settings are already loaded by the time this task runs (it's started
    // in main.cpp after robot.init() returns).
    const SettingsData& cfg = instance_->settings.data();
    const gpio_num_t btn0 = static_cast<gpio_num_t>(cfg.btn0);
    const gpio_num_t btn1 = static_cast<gpio_num_t>(cfg.btn1);
    const gpio_num_t btn2 = static_cast<gpio_num_t>(cfg.btn2);
    const gpio_num_t left = static_cast<gpio_num_t>(cfg.left);
    const gpio_num_t right = static_cast<gpio_num_t>(cfg.right);

    // Debounce window: half of delay_flags, the period at which the flags
    // self-clear and the state machine samples them (see setFlags()/
    // runStateMachine()). Comfortably longer than the few ms a switch really
    // bounces for, and still only half a sampling period, so a deliberate
    // press is never swallowed.
    button_debounce_us = static_cast<int64_t>(cfg.delay_flags) * 1000 / 2;
    button_isr_cfg[BIT_0] = {btn0, BIT_0};
    button_isr_cfg[BIT_1] = {btn1, BIT_1};
    button_isr_cfg[BIT_2] = {btn2, BIT_2};

    // Start the window running: a button still held from boot (the OTA/USB
    // storage sub-mode select in init()) is about to be released, and its
    // release must land inside a window, not on a stale zero timestamp.
    const int64_t now_us = esp_timer_get_time();
    for (volatile int64_t& stamp : button_last_edge_us) stamp = now_us;

    // set the interrupt type for the buttons and side sensors,
    // and add the corresponding ISR handlers to set the flags when the interrupts are triggered
    const gpio_isr_t button_isr = [](void* arg) IRAM_ATTR {
        const ButtonIsr& btn = *static_cast<const ButtonIsr*>(arg);
        if (button_press_accepted(btn)) instance_->buttons.setFlag(btn.bit);
    };

    // ANYEDGE, not NEGEDGE: the handler needs to see the releases too, since
    // they are what re-arm the guard window (see button_press_accepted()).
    for (ButtonIsr& btn : button_isr_cfg) {
        gpio_set_intr_type(btn.pin, GPIO_INTR_ANYEDGE);
        gpio_isr_handler_add(btn.pin, button_isr, &btn);
    }

    gpio_set_intr_type(left, GPIO_INTR_POSEDGE); // RISING
    gpio_isr_handler_add(left, [](void* arg) IRAM_ATTR {
        instance_->sideSensors.setFlag(BIT_0);
    }, nullptr);

    gpio_set_intr_type(right, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(right, [](void* arg) IRAM_ATTR {
        instance_->sideSensors.setFlag(BIT_1);
    }, nullptr);

    // init a periodic timer to get the sensors
    const esp_timer_create_args_t timer_args = {
        .callback = &ROBOT::sampleEKF,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "kalman_trigger",
        .skip_unhandled_events = false
    };

    // set the timer to trigger the EKF at the defined sample rate. The handle
    // is stored on the instance, not dropped on this task's stack: this task
    // ends with vTaskDelete(NULL) a few lines below, and "settings apply
    // timers" needs the handle later to change the period without a reboot.
    esp_timer_create(&timer_args, &instance_->ekf_timer_handle_);
    esp_timer_start_periodic(instance_->ekf_timer_handle_, cfg.sample_micros);

    vTaskDelete(NULL);
}

void ROBOT::runStateMachine(void *param) {
    (void)param; // Suppress unused parameter warning

    // log the task
    logger.insert_log(logType::INFO, "State machine task started");

    while (true) {
        // run state machine
        instance_->machine.run();

        // Telemetry only raises flags here; BtpDevice's comms task sends.
        // "robot.state" is event-driven: one sample per transition.
        const stateName state = static_cast<stateName>(
            StateMachine::current_state.load(std::memory_order_acquire));
        if (instance_->device_ != nullptr) {
            if (state != instance_->last_published_state_) {
                instance_->last_published_state_ = state;
                instance_->device_->publish("robot.state");
            }
            if (instance_->device_->subscribed("robot.sensors")) instance_->sampleSlowSensors();
        }
		// need to add a small delay to avoid blocking the CPU and allow other tasks to run
        vTaskDelay(WDOG_TIMEOUT_TK);
    }
}

void ROBOT::routine(void *param){
    (void)param;

    while (!instance_->initialized)
        vTaskDelay((100));

    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log(logType::INFO, "Parallel processing initialized");
    #endif

    // The boot script, if the card has one. This is what makes the robot
    // configurable with nobody on the other end: write JOB_AUTOEXEC_FILE over
    // USB MSC or with "storage -write", reboot, and it applies its own policy.
    if (!instance_->autoexec_done_) {
        instance_->autoexec_done_ = true;
        if (instance_->startScript(JOB_AUTOEXEC_FILE)) {
            ROBOT::logger.insert_logf(logType::INFO, "running %s", JOB_AUTOEXEC_FILE);
        }
        // After the script, not before: JOB_SAVE_FILE holds jobs that were
        // ACTIVE at some past "job -save", and a fresh autoexec run is free
        // to schedule its own without the two colliding over job table slots
        // in an order that would depend on load timing.
        instance_->loadJobs();
    }

    // excute the loop to menage the robot
    while(true) {
        // OTA's own state machine, every pass regardless of the current
        // state: on an ESP-NOW boot it scans/connects for an "ota start"
        // session; on a TCP boot it waits for BtpDevice's connection.
        if (!instance_->usb_storage.is_active()) {
            instance_->ota.process(instance_->buttons.getFlags());
        }

        // An upload session holds the radio (on ESP-NOW, the target Wi-Fi's
        // channel): the log lines wait in PSRAM and drain once it ends.
        if (!instance_->ota.is_upload_session()) {
            ROBOT::logger.flush_logs();              // hand the retained log lines to BtpDevice
        }
        instance_->resetFlags();                    // reset the flags - buttons, side sensors, pwm...
        instance_->setOutputs();                    // set the output - leds, pwm...
        instance_->checkStateMachine();             // cheg the next state of the state machine
        instance_->updateSoundFeedback();           // react to the changes above with a Junkebox sound
        // After checkStateMachine(), so a job attached to a state sees the
        // entry on the same pass it happens rather than one pass later.
        instance_->pollJobs();                      // fire due time/state jobs
        instance_->pollScript();                    // feed at most one script line
        vTaskDelay(WDOG_TIMEOUT_TK); // delay for wathdog timer and to allow other tasks to run
    }
}

// ==============================================================================
// TELEMETRY (src/btp/btp_topics.cpp reads these on BtpDevice's comms task)
// ==============================================================================

void ROBOT::sampleSlowSensors() {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    if (now_us < next_slow_sample_us_) return;
    next_slow_sample_us_ = now_us + kSlowSensorPeriodUs;
    sensor_snapshot_.current_a.store(readCurrentRaw(current_a_adc_handle_, current_a_adc_channel_),
                                     std::memory_order_relaxed);
    sensor_snapshot_.current_b.store(readCurrentRaw(current_b_adc_handle_, current_b_adc_channel_),
                                     std::memory_order_relaxed);
    // ArraySensor channel i's raw ADC reading, downscaled >>4 to fit one
    // byte (0-255) -- uncalibrated, plotting only.
    for (size_t i = 0U; i < ArraySensor::MAX_LEN; ++i) {
        sensor_snapshot_.array[i].store(
            static_cast<uint8_t>(array_sensor->read_channel(static_cast<uint8_t>(i)) >> 4U),
            std::memory_order_relaxed);
    }
}

void ROBOT::readSensors(SensorsSample& out) {
    out.linear_speed = sensor_snapshot_.enc_linear_speed.load(std::memory_order_relaxed);
    out.angular_speed = sensor_snapshot_.enc_angular_speed.load(std::memory_order_relaxed);
    out.gyro_z = sensor_snapshot_.gyro_z.load(std::memory_order_relaxed);
    out.accel_x = sensor_snapshot_.accel_x.load(std::memory_order_relaxed);
    out.accel_y = sensor_snapshot_.accel_y.load(std::memory_order_relaxed);
    out.current_a = sensor_snapshot_.current_a.load(std::memory_order_relaxed);
    out.current_b = sensor_snapshot_.current_b.load(std::memory_order_relaxed);
    for (size_t i = 0U; i < ArraySensor::MAX_LEN; ++i) {
        out.array[i] = sensor_snapshot_.array[i].load(std::memory_order_relaxed);
    }
}

void ROBOT::readFlags(FlagsSample& out) {
    // Straight from the Flags_*/HBridge state this class already owns.
    out.buttons = buttons.getFlags();
    out.side_sensors = sideSensors.getFlags();
    out.leds = leds.getFlags();
    out.pwm_left = static_cast<int8_t>(motors.getValue(MOTOR_LEFT_idx));
    out.pwm_right = static_cast<int8_t>(motors.getValue(MOTOR_RIGHT_idx));
}

uint8_t ROBOT::currentState() const {
    return static_cast<uint8_t>(StateMachine::current_state.load(std::memory_order_acquire));
}

std::string ROBOT::systemReport(size_t max_size) {
    sysmon.update();
    return sysmon.getTelemetryReport(max_size);
}

// SDA/SCL short check — see the declaration comment in BallyRobot.h.
bool ROBOT::checkSdaSclShort(uint8_t sda_pin, uint8_t scl_pin) {
    const gpio_num_t sda = static_cast<gpio_num_t>(sda_pin);
    const gpio_num_t scl = static_cast<gpio_num_t>(scl_pin);

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << sda) | (1ULL << scl);
    io_conf.mode         = GPIO_MODE_INPUT_OUTPUT_OD;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    if (gpio_config(&io_conf) != ESP_OK) {
        ESP_LOGW("I2C_SCAN", "short check: failed to configure SDA=%u SCL=%u", sda_pin, scl_pin);
        return false;
    }

    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(200);  // let both settle HIGH before probing

    // Drive SDA low, leave SCL released: if shorted, SCL gets dragged down too.
    gpio_set_level(sda, 0);
    esp_rom_delay_us(200);
    const bool scl_follows_sda = (gpio_get_level(scl) == 0);
    gpio_set_level(sda, 1);
    esp_rom_delay_us(200);

    // Drive SCL low, leave SDA released: if shorted, SDA gets dragged down too.
    gpio_set_level(scl, 0);
    esp_rom_delay_us(200);
    const bool sda_follows_scl = (gpio_get_level(sda) == 0);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(200);

    gpio_reset_pin(sda);
    gpio_reset_pin(scl);

    const bool shorted = scl_follows_sda || sda_follows_scl;
    if (shorted) {
        ESP_LOGW("I2C_SCAN",
                "SDA(%u)/SCL(%u) SHORT DETECTED: driving SDA low pulls SCL %s; driving SCL low pulls SDA %s",
                sda_pin, scl_pin,
                scl_follows_sda ? "LOW too" : "stays HIGH",
                sda_follows_scl ? "LOW too" : "stays HIGH");
        logger.insert_logf(logType::WARN,
                           "I2C short check: SDA(%u)/SCL(%u) appear shorted together",
                           sda_pin, scl_pin);
    } else {
        ESP_LOGI("I2C_SCAN", "short check: no short between SDA=%u SCL=%u", sda_pin, scl_pin);
    }
    return shorted;
}

// Manual, bit-banged I2C scan — see the declaration comment in BallyRobot.h
// for why this exists alongside the hardware-peripheral scanI2CBus().
bool ROBOT::bitbangI2CScan(uint8_t sda_pin, uint8_t scl_pin) {
    const gpio_num_t   sda          = static_cast<gpio_num_t>(sda_pin);
    const gpio_num_t   scl          = static_cast<gpio_num_t>(scl_pin);
    const uint32_t      half_bit_us = 300;   // ~1.6kHz — deliberately far below the 100/400kHz a real bus targets
    const uint32_t      rise_wait_us = 5000; // how long to tolerate a slow/clock-stretched rising edge before giving up

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << sda) | (1ULL << scl);
    io_conf.mode         = GPIO_MODE_INPUT_OUTPUT_OD;  // open-drain: writing 1 releases the line to the pull-up, 0 drives it low
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    if (gpio_config(&io_conf) != ESP_OK) {
        ESP_LOGW("I2C_SCAN", "bitbang: failed to configure SDA=%u SCL=%u as open-drain", sda_pin, scl_pin);
        return false;
    }

    // Releases a line and waits for it to actually read HIGH (clock
    // stretching / slow-pullup tolerant), instead of assuming it rose the
    // instant we stopped driving it low.
    auto release_and_wait_high = [&](gpio_num_t pin) -> bool {
        gpio_set_level(pin, 1);
        uint32_t waited = 0;
        while (gpio_get_level(pin) == 0 && waited < rise_wait_us) {
            esp_rom_delay_us(10);
            waited += 10;
        }
        return gpio_get_level(pin) == 1;
    };

    // Idle both lines before starting.
    if (!release_and_wait_high(sda) || !release_and_wait_high(scl)) {
        ESP_LOGW("I2C_SCAN", "bitbang: SDA/SCL never settled HIGH before starting — bus genuinely stuck low");
        logger.insert_log(logType::WARN, "I2C bitbang: SDA/SCL stuck low, aborting");
        gpio_reset_pin(sda);
        gpio_reset_pin(scl);
        return false;
    }
    esp_rom_delay_us(half_bit_us);

    std::string found;
    char        addr_str[8];
    bool        scl_ever_stuck = false;

    for (uint8_t addr7 = 0x08; addr7 <= 0x77; ++addr7) {
        // START: SDA high->low while SCL is high.
        gpio_set_level(sda, 1);
        gpio_set_level(scl, 1);
        esp_rom_delay_us(half_bit_us);
        gpio_set_level(sda, 0);
        esp_rom_delay_us(half_bit_us);
        gpio_set_level(scl, 0);
        esp_rom_delay_us(half_bit_us);

        // 7-bit address + write bit, MSB first.
        const uint8_t byte = static_cast<uint8_t>(addr7 << 1);
        for (int8_t bit = 7; bit >= 0; --bit) {
            gpio_set_level(sda, (byte >> bit) & 0x01);
            esp_rom_delay_us(half_bit_us);
            if (!release_and_wait_high(scl)) scl_ever_stuck = true;
            esp_rom_delay_us(half_bit_us);
            gpio_set_level(scl, 0);
            esp_rom_delay_us(half_bit_us);
        }

        // ACK bit: release SDA, pulse SCL high, sample SDA while it's high.
        gpio_set_level(sda, 1);
        esp_rom_delay_us(half_bit_us);
        if (!release_and_wait_high(scl)) scl_ever_stuck = true;
        esp_rom_delay_us(half_bit_us);
        const bool acked = (gpio_get_level(sda) == 0);
        gpio_set_level(scl, 0);
        esp_rom_delay_us(half_bit_us);

        // STOP: SDA low->high while SCL is high.
        gpio_set_level(sda, 0);
        esp_rom_delay_us(half_bit_us);
        if (!release_and_wait_high(scl)) scl_ever_stuck = true;
        esp_rom_delay_us(half_bit_us);
        gpio_set_level(sda, 1);
        esp_rom_delay_us(half_bit_us);

        if (acked) {
            snprintf(addr_str, sizeof(addr_str), " 0x%02X", addr7);
            found += addr_str;
        }
    }

    gpio_reset_pin(sda);
    gpio_reset_pin(scl);

    if (scl_ever_stuck) {
        ESP_LOGW("I2C_SCAN", "bitbang: SCL failed to rise within %u us at least once during the sweep",
                (unsigned)rise_wait_us);
    }

    if (found.empty()) {
        ESP_LOGW("I2C_SCAN", "bitbang SDA=%u SCL=%u: no devices found", sda_pin, scl_pin);
        logger.insert_logf(logType::WARN, "I2C bitbang (SDA=%u SCL=%u): no devices found", sda_pin, scl_pin);
        return false;
    }

    ESP_LOGI("I2C_SCAN", "bitbang SDA=%u SCL=%u: found%s", sda_pin, scl_pin, found.c_str());
    logger.insert_logf(logType::INFO, "I2C bitbang (SDA=%u SCL=%u): found%s", sda_pin, scl_pin, found.c_str());
    return true;
}

// Probes every valid 7-bit address on sda_pin/scl_pin and logs which ones
// ACK. Must be called before anything else creates an I2C bus on those
// pins — see the declaration comment in BallyRobot.h.
bool ROBOT::scanI2CBus(uint8_t sda_pin, uint8_t scl_pin) {
    // Plain digital-input check, no I2C protocol involved: with only the
    // ESP32's own weak (~45kOhm) internal pull-up active, does each line
    // even settle HIGH at DC? If a line still reads LOW here, no amount of
    // I2C driver tuning fixes that — the internal pull-up isn't strong
    // enough to overcome whatever is holding that net down (missing
    // external pull-up on a board that needs one, a short, a miswired
    // ground, ...). This is exactly the "probe device timeout" symptom
    // (ESP_ERR_TIMEOUT, not ESP_ERR_NOT_FOUND) seen on every address: the
    // bus never reaches idle-high in the first place.
    {
        gpio_num_t sda_gpio = static_cast<gpio_num_t>(sda_pin);
        gpio_num_t scl_gpio = static_cast<gpio_num_t>(scl_pin);
        gpio_reset_pin(sda_gpio);
        gpio_reset_pin(scl_gpio);
        gpio_set_direction(sda_gpio, GPIO_MODE_INPUT);
        gpio_set_direction(scl_gpio, GPIO_MODE_INPUT);
        gpio_set_pull_mode(sda_gpio, GPIO_PULLUP_ONLY);
        gpio_set_pull_mode(scl_gpio, GPIO_PULLUP_ONLY);
        vTaskDelay(pdMS_TO_TICKS(2));  // let the weak pull-up settle

        int sda_level = gpio_get_level(sda_gpio);
        int scl_level = gpio_get_level(scl_gpio);
        ESP_LOGI("I2C_SCAN", "idle level (internal pull-up only): SDA(%u)=%s SCL(%u)=%s",
                sda_pin, sda_level ? "HIGH" : "LOW", scl_pin, scl_level ? "HIGH" : "LOW");
        logger.insert_logf(logType::INFO,
                           "I2C idle check: SDA(%u)=%s SCL(%u)=%s",
                           sda_pin, sda_level ? "HIGH" : "LOW", scl_pin, scl_level ? "HIGH" : "LOW");

        if (sda_level == 0 || scl_level == 0) {
            ESP_LOGW("I2C_SCAN",
                    "line reads LOW at idle even with the internal pull-up -- "
                    "this is a wiring/power problem (missing external pull-up, "
                    "short, bad ground, or unpowered/faulty IMU sinking the "
                    "line), not something firmware can fix");
        }

        // Hand the pins back undecorated; i2c_new_master_bus() below
        // configures them itself (including its own pull-up flag).
        gpio_reset_pin(sda_gpio);
        gpio_reset_pin(scl_gpio);
    }

    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port                     = -1;  // let the driver pick a free port
    bus_config.sda_io_num                   = static_cast<gpio_num_t>(sda_pin);
    bus_config.scl_io_num                   = static_cast<gpio_num_t>(scl_pin);
    bus_config.clk_source                   = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt            = 7;
    bus_config.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus_handle = nullptr;
    if (i2c_new_master_bus(&bus_config, &bus_handle) != ESP_OK) {
        ESP_LOGW("I2C_SCAN", "failed to open bus on SDA=%u SCL=%u", sda_pin, scl_pin);
        logger.insert_logf(logType::WARN,
                           "I2C scan: failed to open bus on SDA=%u SCL=%u",
                           sda_pin, scl_pin);
        return false;
    }

    // Every unanswered address makes i2c_master_probe() itself log an
    // ESP_LOGE "probe device timeout" line — silence that component for the
    // duration of the sweep so an (expected, common) empty/near-empty bus
    // doesn't spam the console with dozens of misleading error lines.
    esp_log_level_t prev_i2c_log_level = esp_log_level_get("i2c.master");
    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    std::string found;
    char        addr_str[8];
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        if (i2c_master_probe(bus_handle, addr, 30) == ESP_OK) {
            snprintf(addr_str, sizeof(addr_str), " 0x%02X", addr);
            found += addr_str;
        }
    }

    esp_log_level_set("i2c.master", prev_i2c_log_level);
    i2c_del_master_bus(bus_handle);

    // Sent both ways: ESP_LOG so it shows up immediately on a serial
    // monitor, and through the logger (ESP-NOW to the ground station, see
    // main.cpp's set_send_callback) so it's also visible there.
    if (found.empty()) {
        ESP_LOGW("I2C_SCAN", "SDA=%u SCL=%u: no devices found", sda_pin, scl_pin);
        logger.insert_logf(logType::WARN,
                           "I2C scan (SDA=%u SCL=%u): no devices found",
                           sda_pin, scl_pin);
        return false;
    }

    ESP_LOGI("I2C_SCAN", "SDA=%u SCL=%u: found%s", sda_pin, scl_pin, found.c_str());
    logger.insert_logf(logType::INFO,
                       "I2C scan (SDA=%u SCL=%u): found%s",
                       sda_pin, scl_pin, found.c_str());
    return true;
}

bool ROBOT::init() {
    if (initialized)
        return true;

    // general setup for gpio, i2c, and other peripherals
    gpio_install_isr_service(0);

    // Only CS needs to exist before the SD card can mount and
    // settings.conf can be read; the rest of the pin config comes after.
    if (!configurePinsEarly()) {
        ESP_LOGE("ROBOT_INIT", "Failed to configure early pins (CS)");
        return false;
    }

    logger.begin();

    // The BTP identity BtpDevice will use on the link, known before it
    // starts (btpdev::identity() counts this boot in NVS): the SD log files
    // and the wire agree on source/boot. Lines logged before this are
    // dropped (Logger needs the endpoint to number them).
    const btpdev::Identity& identity = btpdev::identity();
    protocol.configure(identity.source_id, identity.boot_id);
    logger.configure_btp(protocol);
    logger.insert_logf(logType::INFO, "BTP source=%08lx boot=%08lx",
                       static_cast<unsigned long>(identity.source_id),
                       static_cast<unsigned long>(identity.boot_id));

    // Why the chip is starting this boot -- previously only discoverable by
    // typing "sys reset_reason" right after it happened, so an unattended
    // brownout/panic/watchdog reset in the field went unrecorded unless
    // someone happened to ask at exactly the right moment. One line, every
    // boot, so it survives into the retained/radioed LOG history instead.
    const esp_reset_reason_t reset_reason = esp_reset_reason();
    logger.insert_logf(logType::INFO, "Boot: reset_reason=%d name=%s",
                       static_cast<int>(reset_reason), resetReasonName(reset_reason));

    // Persist whatever the retained PSRAM ring holds before an ORDERLY
    // restart wipes it -- esp_restart() (sys -reboot, factory_reset, the
    // post-OTA-upload restart) calls every registered shutdown handler first.
    // Same rationale as the one-shot flush TELEMETRY already does on its own
    // transition (src/robot/07_Telemetry.cpp): whatever has not yet made it
    // out over a possibly lossy/out-of-range radio link would otherwise be
    // lost the moment the ring is wiped.
    //
    // This deliberately does NOT run on a crash: esp_restart()'s shutdown
    // handlers are skipped entirely by a panic/abort, so a firmware fault
    // still loses this boot's unflushed log. Catching that case needs a
    // coredump partition and CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH, which is a
    // flash-layout change this firmware does not make yet -- it can only be
    // applied together with a planned cable reflash, never delivered by OTA,
    // so it is intentionally left for that occasion rather than folded in
    // here.
    //
    // Registered this early so it is armed for the rest of boot;
    // flush_to_sd() itself already refuses safely if the card never mounts,
    // or if USB currently owns it (SDCard::is_mounted() reports false then).
    esp_register_shutdown_handler(&ROBOT::flushLogsOnShutdown);

    // Initialize the card and give FAT ownership to the USB storage manager.
    // It starts mounted for the robot and only exposes it on a DEBUG command.
    if (!sd_card.begin()) {
        logger.insert_log(logType::ERRO, "Failed to initialize SD card");
        ESP_LOGE("ROBOT_INIT", "Failed to initialize SD card");
    } else if (!usb_storage.begin(sd_card, leds)) {
        logger.insert_log(logType::ERRO,
                          "Failed to mount SD card through USB storage manager");
        ESP_LOGE("ROBOT_INIT", "Failed to mount SD card through USB storage manager");
    }

    // Load settings.conf (or, if missing/unmounted, keep the compiled-in
    // defaults already in settings.data() and — when the card is mounted —
    // persist them as a fresh, complete file).
    uint16_t settings_skipped = 0;
    if (!settings.load(sd_card, &settings_skipped)) {
        logger.insert_log(logType::WARN,
                          "settings.conf unavailable; using compiled-in defaults");
    } else if (settings_skipped > 0) {
        logger.insert_logf(logType::WARN,
                           "settings.conf: %u unknown/invalid line(s) ignored",
                           settings_skipped);
    }
    logger.set_flush_limits(settings.data().max_chunks_per_flush,
                            settings.data().block_size);
    // The USB serial port is listed under the robot's name (the same one BLE
    // advertises), so TraceView can tell robots apart without opening ports.
    // Before anything below installs the USB device (storage boot, comm_mode
    // serial): descriptors are read once, at enumeration.
    usb_storage.set_product_name(settings.data().name);

    // bally.key is BtpDevice's now: it imports the file from the SD card into
    // NVS at begin() (see src/btp/btp_storage.h) and logs the public verify
    // tags. Not fatal when missing, same as settings.conf above: a robot
    // without a key boots, on the USB serial link only.

    // Configure the remaining pins now that settings are known, then build
    // the peripherals whose constructors need those pins (ArraySensor
    // touches GPIO/ADC immediately, so it cannot be built earlier).
    if (!configurePinsFromSettings()) {
        ESP_LOGE("ROBOT_INIT", "Failed to configure pins from settings");
        return false;
    }

    const SettingsData& cfg = settings.data();

    // Boot-time sub-mode select -- the ONE place every "hold a button at
    // power-up" mode is decided. Hold the button(s) BEFORE resetting and
    // keep them down until the LEDs show the mode:
    //
    //   btn1 + btn2 -> COMM_CONFIG (comm_mode menu, see commConfigTick())
    //   btn2 only   -> OTA          (same as "ota start" in DEBUG)
    //   btn1 only   -> USB storage  (same as "storage expose" in DEBUG)
    //   nothing     -> SETUP -> WAIT, normal boot
    //
    // Read right after configurePinsFromSettings() configured the button
    // pins with a pull-up (pressed reads LOW); acted on further down, once
    // OTA/USB storage themselves are ready (see ota.begin() below).
    //
    // cfg.btn0 is deliberately NOT used here: it defaults to GPIO0, the
    // ESP32-S3 boot strapping pin (see RobotSettings.h). Holding it while
    // the chip comes out of reset selects Joint Download Boot in ROM, so
    // the application never runs at all. COMM_CONFIG used to be entered by
    // pressing btn0 inside a short window after SETUP started, but init()
    // takes a variable, invisible amount of time, so in practice nobody
    // could hit that window -- hence the btn1+btn2 chord instead.
    const bool boot_btn1 = gpio_get_level(static_cast<gpio_num_t>(cfg.btn1)) == 0;
    const bool boot_btn2 = gpio_get_level(static_cast<gpio_num_t>(cfg.btn2)) == 0;
    const bool boot_enter_comm_config = boot_btn1 && boot_btn2;
    const bool boot_enter_ota = boot_btn2 && !boot_btn1;
    const bool boot_enter_storage = boot_btn1 && !boot_btn2;

    array_sensor.emplace(cfg.s0, cfg.s1, cfg.s2, cfg.sig, cfg.len_sensor);
    // After array_sensor above: if cfg.current_a shares a physical ADC unit
    // with array_sensor's own "sig" pin (see initCurrentSensors()'s comment),
    // array_sensor's ADC init has already succeeded by the time this runs.
    initCurrentSensors();
    encoder_left.emplace(cfg.enc_a0, cfg.enc_a1);
    encoder_right.emplace(cfg.enc_b0, cfg.enc_b1);
    // Two LEDC channels per motor (IN1/IN2); CH0..CH3 (Settings.h) are
    // reserved exactly for this pair of H-bridges.
    motor_left.emplace(cfg.ain1, cfg.ain2, CH0, CH1);
    motor_right.emplace(cfg.bin1, cfg.bin2, CH2, CH3);
    junkebox.emplace(cfg.bzr, CH4);

    // Boot-time bitbangI2CScan()/scanI2CBus() disabled for now (noisy IMU
    // bus under active hardware debugging -- see "debug scan_i2c"/"test_i2c"
    // for the on-demand equivalents, which reuse imu's own bus instead of
    // opening a second one on these pins). Re-enable once the wiring is
    // sorted out if the one-shot boot log is still wanted.
    imu.emplace(cfg.sda_pin, cfg.scl_pin, IMU_I2C_ADDRESS, IMU_I2C_CLOCK_HZ);

    sysmon.begin();
    sysmon.setLoggerCallback([]() { return ROBOT::logger.get_write_pct(); });

    // Jobs run their lines through BtpDevice's shell (submitLocalCommand()):
    // until main.cpp attaches the device, a job that fires is refused.
    jobs.configure(&ROBOT::submitLocalCommandStatic, this);

    // comm_mode picks this boot's link (settings.conf, "comm" module, which
    // the COMM_CONFIG menu writes): 0 ESP-NOW, 1 TCP, 2 BLE, 3 USB serial.
    // Anything out of range (a hand-edited or corrupted settings.conf) falls
    // back to ESP-NOW, the safest default. A boot into OTA uses TCP instead:
    // the update server rides BtpDevice's Wi-Fi connection, whatever the
    // robot normally talks over.
    uint8_t comm_mode = cfg.comm_mode;
    if (comm_mode > 3) {
        logger.insert_logf(logType::WARN,
                           "comm_mode=%u is not a valid value, falling back to ESP-NOW",
                           static_cast<unsigned>(comm_mode));
        comm_mode = 0U;
    }
    static constexpr btpdev::Link kCommModeLinks[4] = {
        btpdev::Link::EspNow, btpdev::Link::Tcp, btpdev::Link::Ble, btpdev::Link::Serial};
    boot_link_ = boot_enter_ota ? btpdev::Link::Tcp : kCommModeLinks[comm_mode];
    boot_enter_ota_ = boot_enter_ota;
    boot_enter_storage_ = boot_enter_storage;

    // The OTA server itself starts in attachDevice(), once BtpDevice owns
    // the radio (the event loop, and on TCP the Wi-Fi connection).
    ota.configure(OtaTuning{
        .led_step_ms        = cfg.ota_led_step_ms,
        .led_hold_ms        = cfg.ota_led_hold_ms,
        .led_fail_hold_ms   = cfg.ota_led_fail_hold_ms,
        .connect_timeout_ms = cfg.ota_connect_timeout_ms,
        .retry_scan_ms      = cfg.ota_retry_scan_ms,
        .espnow_channel     = cfg.espnow_channel,
        .hostname           = cfg.ota_hostname,
        .instance_name      = cfg.ota_instance_name,
        .password           = cfg.ota_password,
    });
    ota.set_external_wifi(boot_link_ == btpdev::Link::Tcp);

    // Buttons 2+3 held at reset: the comm_mode menu (it needs no link).
    if (boot_enter_comm_config) {
        boot_state_ = COMM_CONFIG;
        logger.insert_log(logType::INFO,
                          "Boot: buttons 2+3 held, entering COMM_CONFIG");
    }

    // USB serial: the native port is ONE composite device, CDC (BtpDevice's
    // link) + MSC (usb_storage's SD card), installed here, before BtpDevice
    // opens the CDC port (DeviceConfig::usb.install_driver = false -- see
    // src/btp/btp_device.cpp). Descriptors are read once, at enumeration.
    if (boot_link_ == btpdev::Link::Serial && !usb_storage.install_persistent_usb_device()) {
        logger.insert_log(logType::ERRO, "comm_mode=serial: failed to install the USB device");
    }

    if (motor_left->init() != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to initialize left motor");
        ESP_LOGE("ROBOT_INIT", "Failed to initialize left motor");
        return false;
    }
    if (motor_right->init() != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to initialize right motor");
        ESP_LOGE("ROBOT_INIT", "Failed to initialize right motor");
        return false;
    }

    if (!encoder_left->init()) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to initialize left encoder");
        ESP_LOGE("ROBOT_INIT", "Failed to initialize left encoder");
        return false;
    }
    if (!encoder_right->init()) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to initialize right encoder");
        ESP_LOGE("ROBOT_INIT", "Failed to initialize right encoder");
        return false;
    }

    // Retry a few times: on a real but flaky bus, a failed attempt leaves
    // the ICM42688's I2C bus/device handles open (see initI2CBus()), so a
    // bare re-call of begin() would just fail again with the pins already
    // claimed. Re-emplacing the object first runs ~ICM42688() and tears
    // those handles down, letting the retry claim the bus fresh.
    int imu_begin_ret = -1;
    for (int attempt = 1; attempt <= kImuInitAttempts; ++attempt) {
        if (attempt > 1) {
            vTaskDelay(pdMS_TO_TICKS(kImuInitRetryDelayMs));
            imu.emplace(cfg.sda_pin, cfg.scl_pin, IMU_I2C_ADDRESS, IMU_I2C_CLOCK_HZ);
        }
        imu_begin_ret = imu->begin();
        if (imu_begin_ret > 0) break;
    }
    imu_ready_ = imu_begin_ret > 0;
    if (!imu_ready_) {
        // begin()'s return codes (see ICM42688::begin()): -2 bus/device open
        // failed, -3 WHO_AM_I mismatch (something else is answering at this
        // address, or the chip isn't responding correctly), -4/-7/-8 a later
        // config/calibration step failed.
        ROBOT::logger.insert_logf(logType::ERRO,
                                  "IMU (ICM42688) not detected after %d attempt(s) (begin()=%d)",
                                  kImuInitAttempts, imu_begin_ret);
        ESP_LOGE("ROBOT_INIT", "Failed to initialize IMU after %d attempt(s) (begin()=%d)",
                kImuInitAttempts, imu_begin_ret);
        // Not fatal, same as the buzzer below: everything that reads the
        // IMU already checks imu_ready_ (sampleEKF(), the DEBUG shell
        // commands). Returning false here made app_main retry init()
        // forever -- init() is not re-entrant (GPIO ISR service, ADC unit
        // and OTA event handlers are already claimed on the second pass),
        // so a missing/flaky IMU left the robot dead in that loop and
        // unreachable even through OTA, USB storage or COMM_CONFIG.
    }

    // Not fatal: a buzzer that fails to configure just means no music, not
    // a robot that can't drive.
    const esp_err_t junkebox_err = junkebox->begin(sd_card);
    if (junkebox_err != ESP_OK) {
        ESP_LOGW("ROBOT_INIT", "Failed to initialize buzzer (Junkebox): 0x%x (pin=%u)",
                junkebox_err, cfg.bzr);
        ROBOT::logger.insert_logf(logType::WARN,
                                  "Failed to initialize buzzer (Junkebox): 0x%x; music disabled",
                                  junkebox_err);
    }

    setTimeLimit();
    initEKF();
    registerSettingsAppliers();

    ROBOT::logger.insert_log(logType::INFO, "Welcome! the car is starting...");

    initialized = true;
    return true;
}

void ROBOT::attachDevice(BtpDevice& device, bool ready) {
    if (ready) {
        device_ = &device;
        // The logger's link: each retained line goes out as a BTP LOG through
        // BtpDevice (sealed like everything else, with its original time); a
        // refused line stays in PSRAM for the next flush_logs(). While a
        // command runs, its lines are also the command's output in
        // TraceView's terminal.
        logger.set_radio_sink([](logType type, uint64_t timestamp_us, const uint8_t* text,
                                 size_t length) {
            return btpdev::log_record(static_cast<btpdev::LogLevel>(type), timestamp_us,
                                      reinterpret_cast<const char*>(text), length);
        });
        logger.set_echo([](const char* text, size_t length) {
            if (btpdev::in_command()) btpdev::printf("%.*s\n", static_cast<int>(length), text);
        });
    } else {
        logger.insert_log(logType::ERRO, "BtpDevice did not start: the robot runs without a link");
    }

    // OTA registers on the event loop BtpDevice's Wi-Fi created (ESP-NOW
    // and TCP boots; BLE and serial never start Wi-Fi).
    const bool wifi = ready && device.active_link() == boot_link_ &&
                      (boot_link_ == btpdev::Link::EspNow || boot_link_ == btpdev::Link::Tcp);
    if (wifi && !ota.begin(sd_card, leds, [](logType type, const char* msg) {
            ROBOT::logger.insert_log(type, msg);
        })) {
        logger.insert_log(logType::ERRO, "Failed to initialize OTA updater");
    }

    // Boot-time sub-modes (see bootState()): button 3 -> OTA, button 2 ->
    // USB storage. Only a sub-mode that actually started changes the state.
    if (boot_enter_ota_) {
        if (wifi && ota.start()) {
            boot_state_ = DEBUG;
            logger.insert_log(logType::INFO,
                              "Boot: button 3 held, entering OTA mode directly");
        } else {
            logger.insert_log(logType::ERRO,
                              "Boot: button 3 held but OTA could not be started "
                              "(no Wi-Fi: check wifi.txt on the SD card)");
        }
    } else if (boot_enter_storage_) {
        if (usb_storage.expose()) {
            boot_state_ = DEBUG;
            logger.insert_log(logType::INFO,
                              "Boot: button 2 held, entering USB storage mode directly");
        } else {
            logger.insert_log(logType::ERRO,
                              "Boot: button 2 held but USB storage could not be exposed");
        }
    }
}
