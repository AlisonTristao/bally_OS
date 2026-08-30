#include <BallyRobot.h>
#include <Settings.h>

// ESP-IDF Includes
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "esp_now.h"
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
// Dropped from the ICM42688 driver's 400kHz default: this bus currently has
// no external pull-ups (relies on the ESP32's own weak ~45kOhm ones), which
// shows up as intermittent NACKs/garbled reads at 400kHz (scan_i2c: address
// ACKs but WHO_AM_I reads back 0xFF, or the whole bus goes briefly silent).
// 100kHz gives the lines more time to slew and tolerates noise better.
// This is a mitigation, not a fix -- add real pull-ups (2.2-4.7kOhm to
// 3.3V on SDA and SCL) and this can go back to the default.
static constexpr uint32_t IMU_I2C_CLOCK_HZ = 100'000;
static constexpr float   kDegToRad       = static_cast<float>(PI) / 180.0f;
static constexpr float   kGravityMss     = 9.81f;

// ==============================================================================
// STATIC MEMBER INITIALIZATION
// ==============================================================================

static const char* TAG = "ROBOT_CORE"; // Tag para os logs nativos do ESP-IDF

// active instance of the robot class
ROBOT* ROBOT::instance_ = nullptr;

Logger ROBOT::logger;

// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================

// verify_e/verify_l are public tags, safe to log; this is the only place
// that turns them into text. Never call this on key_e()/key_l() themselves.
static void hexEncode(const uint8_t* data, size_t size, char* out) {
    static const char kHexDigits[] = "0123456789abcdef";
    for (size_t i = 0; i < size; ++i) {
        out[i * 2] = kHexDigits[data[i] >> 4];
        out[i * 2 + 1] = kHexDigits[data[i] & 0x0F];
    }
    out[size * 2] = '\0';
}

static void readMacAddress() {
    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
    if (ret == ESP_OK) {
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", 
                 baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
        
        ESP_LOGI(TAG, "MAC Address: %s", macStr); // Substitui Serial.println
        ROBOT::logger.insert_logf(logType::INFO, "MAC Address: %s", macStr);
    } else {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to read MAC address");
    }
}

// ==============================================================================
// HARDWARE CONFIGURATION
// ==============================================================================

namespace {

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

    return true;
}

bool ROBOT::configureProtocolIdentity() {
    if (protocol.source_id() != 0U && protocol.boot_id() != 0U) return true;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ret = nvs_flash_erase();
        if (ret == ESP_OK) ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return false;

    uint8_t base_mac[6]{};
    if (esp_efuse_mac_get_default(base_mac) != ESP_OK) return false;
    const uint32_t source_id = btp_command::source_id_from_mac(base_mac);

    nvs_handle_t handle = 0;
    ret = nvs_open("btp", NVS_READWRITE, &handle);
    if (ret != ESP_OK) return false;

    uint32_t previous_boot_id = 0U;
    ret = nvs_get_u32(handle, "last_boot", &previous_boot_id);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        return false;
    }

    uint32_t boot_id = 0U;
    do {
        boot_id = esp_random();
    } while (boot_id == 0U || boot_id == previous_boot_id);

    ret = nvs_set_u32(handle, "last_boot", boot_id);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    if (ret != ESP_OK || !protocol.configure(source_id, boot_id)) return false;

    // Stable, opaque 16-byte identity for MANIFEST_DATA's source_uuid: the
    // MAC (6 bytes, already used for source_id) plus a fixed non-zero
    // suffix, same construction t_dongle_develop's SerialSession uses for
    // its own peer_uuid (topico 13 RESULTADO) since neither side has a
    // persisted UUID store yet.
    std::memcpy(protocol_uuid_, base_mac, 6U);
    static constexpr uint8_t kUuidSuffix[10] = {0xB0, 0xB1, 0xB2, 0xB3, 0xB4,
                                                0xB5, 0xB6, 0xB7, 0xB8, 0xB9};
    std::memcpy(protocol_uuid_ + 6U, kUuidSuffix, sizeof(kUuidSuffix));

    // Binds RadioSeal to key_store regardless of load order: seal()/open()
    // below read the KeyStore's state at call time, and both already fail
    // closed on !loaded() -- see RadioSeal.h. Safe to do before
    // key_store.load_from_card() runs later in init().
    RadioSeal::configure(key_store);

    // Same MAC->source_id conversion as this robot's own identity two lines
    // above, applied to the dongle's known MAC (MAC_ADDR, the same build
    // flag handleReceiveStatic's radio prefilter reads) instead of this
    // device's efuse MAC. Left 0U -- an invalid source_id intake() already
    // rejects -- when this build has no MAC_ADDR at all, matching
    // handleReceiveStatic's own "no MAC_ADDR -> reject everything" branch.
#ifdef MAC_ADDR
    {
        static constexpr uint8_t kDongleMac[6] = {MAC_ADDR};
        dongle_source_id_ = btp_command::source_id_from_mac(kDongleMac);
    }
#endif

    // LOG is channel B (bally_channels.h), same as TELEMETRY: the dongle
    // relays it and never reads it. Sealed with key E over the radio, or not
    // sent -- the SD flush path stays cleartext (local diagnostic files).
    logger.configure_btp(protocol, RadioSeal::seal_e, nullptr);
    // COMMAND_RESULT is sealed with whichever channel's key opened the
    // request it answers (see CommandProcessor::configure's comment):
    // seal_link for channel C (dongle, key L), seal_endpoint for channel B
    // (TraceView, key E, topico 31).
    command_processor.configure(protocol, RadioSeal::seal, nullptr,
                                RadioSeal::seal_e, nullptr);
    // MANIFEST_DATA is channel C always (bally_channels.h: only the dongle's
    // own ManifestCache legitimately asks a robot for its manifest), so it
    // takes the single-key seal STATUS already uses below -- topico 31.3:
    // without this, bally_dongle's own reply to its own priming request
    // never authenticates, so it can never be told apart from a forged one
    // and ManifestCache never learns this robot's schema while a desktop is
    // attached (see bally_channels.h's dongle_consumes comment).
    manifest_responder.configure(protocol, protocol_uuid_, RadioSeal::seal, nullptr);
    // TELEMETRY is channel B (bally_channels.h): TraceView holds key E, the
    // dongle relays the samples and never reads them. Sealed with E so a
    // desktop that has the robot's password is the only thing that can plot
    // them -- fail-closed, no cleartext fallback if key E is missing.
    telemetry.configure(protocol, RadioSeal::seal_e, nullptr);
    // SUBSCRIBE_RESULT/UNSUBSCRIBE_RESULT are sealed the same way
    // COMMAND_RESULT is (topico 31.2): seal_link for channel C, seal_endpoint
    // for channel B -- see handleReceiveStatic's widened classification
    // above.
    subscription_responder.configure(protocol, telemetry, RadioSeal::seal, nullptr,
                                      RadioSeal::seal_e, nullptr);
    // STATUS (heartbeat) is channel C by definition (bally_channels.h), so
    // it seals for real too.
    status_reporter.configure(protocol, telemetry, RadioSeal::seal, nullptr);
    // TERMINAL_OUT is channel B (topico 19bis): TraceView's terminal widget
    // holds key E, the dongle relays the stream verbatim. Sealed with E or not
    // sent, same as TELEMETRY/LOG. Tab completion runs against this robot's
    // real TinyShell catalog, exactly like bally_dongle's console does.
    terminal_responder.configure(
        protocol, RadioSeal::seal_e, nullptr,
        [this](const std::string& input, std::string* out, size_t max_out) -> size_t {
            if (out == nullptr || max_out == 0U) return 0U;
            const std::vector<std::string> matches = shell.complete_line(input, max_out);
            size_t written = 0U;
            for (const std::string& match : matches) {
                if (written >= max_out) break;
                out[written++] = match;
            }
            return written;
        },
        &ROBOT::submitTerminalCommandStatic, this, "bally> ");
    logger.insert_logf(
        logType::INFO,
        "BTP v%u, bally_protocol %u.%u.%u, source=%08lx boot=%08lx",
        btp::kV1Version, btp::kLibraryVersionMajor, btp::kLibraryVersionMinor,
        btp::kLibraryVersionPatch, static_cast<unsigned long>(source_id),
        static_cast<unsigned long>(boot_id));
    return true;
}

bool ROBOT::configureCommunication() {
    if (communication_configured_) return true;

    esp_netif_init();
    esp_event_loop_create_default();

    // Must exist before esp_wifi_start() below: it's what wires the STA
    // netif to WIFI_EVENT_STA_START/CONNECTED/DISCONNECTED and IP_EVENT so
    // the DHCP client actually runs after a connect. Creating it later (as
    // OTAUpdater::begin() used to) missed the STA_START event that already
    // fired here, leaving the netif never marked "up" — Wi-Fi would
    // associate fine but esp_netif_dhcpc_start() never ran, so DHCP never
    // even got attempted and the OTA connect just sat there until timeout.
    if (esp_netif_create_default_wifi_sta() == nullptr) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to create the Wi-Fi STA netif");
        ESP_LOGE("ROBOT_INIT", "Failed to create the Wi-Fi STA netif");
        return false;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK || esp_wifi_start() != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to configure WiFi mode");
        ESP_LOGE("ROBOT_INIT", "Failed to configure WiFi mode");
        return false;
    }

    // Default modem sleep (WIFI_PS_MIN_MODEM) lets the radio doze between
    // beacons; both ESP-NOW latency and the OTA sub-mode's AP association
    // need it awake — otherwise the DHCP offer/ACK after a successful
    // connect can be missed, association stays up, and OTA just times out
    // waiting for an IP that was already sent.
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Explicit channel so ESP-NOW has a known home to return to after the
    // OTA sub-mode (in DEBUG) associates with an access point and leaves.
    // Runtime setting (RobotSettings, module "ota") so this and
    // OTAUpdater::cancel()'s restore can't drift into two different values.
    esp_wifi_set_channel(settings.data().espnow_channel, WIFI_SECOND_CHAN_NONE);

    vTaskDelay(50 / portTICK_PERIOD_MS);

    // initialize ESP-NOW
    esp_err_t err = esp_now_init();
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to initialize ESP-NOW");
        ESP_LOGE("ROBOT_INIT", "Failed to initialize ESP-NOW (0x%x)", err);
        return false;
    }

    // configure the ESP-NOW callbacks
    err = esp_now_register_recv_cb(handleReceiveStatic);
    if (err != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to register receive callback");
        ESP_LOGE("ROBOT_INIT", "Failed to register receive callback (0x%x)", err);
        return false;
    }

    err = esp_now_register_send_cb(handleSendStatic);
    if (err != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to register send callback");
        ESP_LOGE("ROBOT_INIT", "Failed to register send callback (0x%x)", err);
        return false;
    }

    readMacAddress();

    #ifdef MAC_ADDR
        uint8_t peer_addr[6] = {MAC_ADDR};
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, peer_addr, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        peerInfo.ifidx = WIFI_IF_STA;
        err = esp_now_add_peer(&peerInfo);
        if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
            ROBOT::logger.insert_log(logType::ERRO, "Failed to add ESP-NOW peer");
            ESP_LOGE("ROBOT_INIT", "Failed to add ESP-NOW peer (0x%x)", err);
            return false;
        }
    #else
        #warning "MAC_ADDR not defined; ESP-NOW peer not added"
    #endif

    communication_configured_ = true;
    return true;
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
        motor_left->applyPWM(left_pwm);
        motor_right->applyPWM(right_pwm);
    }

    // turn on the leds according to the BITS of the arr_stats variable
    uint8_t arr_stats = leds.getFlags();
    const SettingsData& cfg = settings.data();
    gpio_set_level(static_cast<gpio_num_t>(cfg.led0), (arr_stats & (1 << LED0_idx)));
    gpio_set_level(static_cast<gpio_num_t>(cfg.led1), (arr_stats & (1 << LED1_idx)));
    gpio_set_level(static_cast<gpio_num_t>(cfg.led2), (arr_stats & (1 << LED2_idx)));
    gpio_set_level(static_cast<gpio_num_t>(cfg.led3), (arr_stats & (1 << LED3_idx)));
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

    ota.process(buttons.getFlags());
    if (ota.is_active()) return;

    // Add one `if (test.poll()) { ... }` block per ScheduledDebugTest member
    // (H-bridge current, ...).
    if (array_sensor_test_.poll()) {
        logger.insert_log(logType::INFO, array_sensor->debug().c_str());
    }

    if (encoder_test_.poll()) {
        logger.insert_logf(
            logType::INFO, "Encoders: left=%lld right=%lld",
            static_cast<long long>(encoder_left->getCount()),
            static_cast<long long>(encoder_right->getCount()));
    }

    if (imu_test_.poll()) {
        imu->getAGT();
        logger.insert_logf(
            logType::INFO,
            "IMU: ax=%.2f\tay=%.2f\taz=%.2f\tgx=%.2f\tgy=%.2f\tgz=%.2f\tt=%.2f",
            imu->accX(), imu->accY(), imu->accZ(),
            imu->gyrX(), imu->gyrY(), imu->gyrZ(), imu->temp());
    }

    if (imu_i2c_test_.poll()) {
        const uint8_t who = imu->whoAmI();
        const bool    ok  = (who == 0x47);
        ok ? ++imu_i2c_ok_count_ : ++imu_i2c_fail_count_;
        const uint32_t total   = imu_i2c_ok_count_ + imu_i2c_fail_count_;
        const float    ok_pct  = total ? (100.0f * imu_i2c_ok_count_) / total : 0.0f;
        logger.insert_logf(
            logType::INFO,
            "IMU I2C check: %s (WHO_AM_I=0x%02X) | %u ok / %u fail (%.1f%% ok)",
            ok ? "OK" : "FAIL", who, imu_i2c_ok_count_, imu_i2c_fail_count_, ok_pct);
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
// COMMUNICATION CALLBACKS
// ==============================================================================

// The receive pipeline, in this order and no other:
//
//     decode -> reassemble -> route
//
// It used to be decode -> route by type -> reassemble, with reassembly
// reachable only from the COMMAND and CONTROL branches. That worked while
// the dongle was the only peer and delivered whole messages. Behind a hub it
// does not: TraceView now speaks to this robot end to end and the dongle
// relays fragment by fragment, verbatim, by design -- reassembly is the
// endpoint's job. A type filter placed before reassembly judges the header
// of a fragment, and throws away pieces of messages it would have accepted
// whole. So reassembly became a stage every frame crosses (RxRouter), and
// the type filter moved behind it, where it only ever sees complete
// messages. Topico 31 slots aead_open() into the same gap, for the same
// reason: a tag computed over the whole logical payload cannot be checked
// against a piece of it.
void ROBOT::handleReceiveStatic(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    // Partial or malformed radio payloads are rejected by btp::decode inside
    // the router, before any field is read. Only the COMMAND_REQUEST route
    // below can enqueue work for TinyShell.
    if (instance_ == nullptr || recv_info == nullptr || incomingData == nullptr ||
        len <= 0 || instance_->receivedDataQueue == nullptr) return;

    // STATUS section 5 counters: every octet stream the radio hands us is one
    // received frame attempt.
    instance_->link_frames_rx_.fetch_add(1U, std::memory_order_relaxed);

    // Stage zero: cheap radio filter, and NOT authentication -- read the
    // comment on btp_command::authorized_source. It no longer binds the
    // frame's source_id to the sender's MAC, because behind a hub every frame
    // legitimately arrives from the dongle's MAC carrying somebody else's
    // source_id. The real authorization is the AEAD tag opened in stage two
    // below (RadioSeal::open / open_e, fail-closed): a spoofed MAC clears this
    // memcmp but cannot forge a tag, so it never reaches the shell.
    //
    // It runs before btp::decode now, where it used to run after: a frame
    // from a radio that is not our peer no longer moves the CRC or
    // decode-error counters, which describe our own link.
#ifdef MAC_ADDR
    static constexpr uint8_t expected_peer[6] = {MAC_ADDR};
    if (!btp_command::authorized_source(expected_peer, recv_info->src_addr)) {
        instance_->command_processor.note_unauthorized();
        return;
    }
#else
    instance_->command_processor.note_unauthorized();
    return;
#endif

    // Stage one: decode + reassemble. Returns Routed only for a COMPLETE
    // logical message. CRC and decode failures stay counted separately (a
    // frame rejected by CRC is never also counted as a decode error).
    const RxRouter::Outcome outcome = instance_->rx_router_.submit(
        incomingData, static_cast<size_t>(len),
        static_cast<uint64_t>(esp_timer_get_time() / 1000ULL),
        &instance_->rx_routed_);
    switch (outcome) {
        case RxRouter::Outcome::Routed:
            break;
        case RxRouter::Outcome::FragmentAccepted:
        case RxRouter::Outcome::DuplicateFragment:
            // Not an error, and not a message yet. A byte-identical retry is
            // absorbed by the reassembler without disturbing the slot.
            return;
        case RxRouter::Outcome::DroppedCrc:
            instance_->link_crc_errors_.fetch_add(1U, std::memory_order_relaxed);
            return;
        case RxRouter::Outcome::DroppedDecode:
        case RxRouter::Outcome::DroppedInvalidArgument:
            instance_->link_decode_errors_.fetch_add(1U, std::memory_order_relaxed);
            return;
        case RxRouter::Outcome::DroppedReassembly:
            instance_->link_reassembly_rejected_.fetch_add(1U, std::memory_order_relaxed);
            instance_->command_processor.note_drop();
            return;
    }

    if (instance_->rx_routed_.reassembled) {
        instance_->link_reassembly_completed_.fetch_add(1U, std::memory_order_relaxed);
    }

    // Stage two: classify, then open. header.source_id sits in the clear at
    // a fixed offset even inside a sealed frame (bally_channels.h), so which
    // key opens this frame can be decided before opening it. COMMAND and
    // CONTROL/SUBSCRIBE/UNSUBSCRIBE are classified by peer (topico 31.2
    // widened these two alongside COMMAND, once SubscriptionResponder's own
    // replies became channel-aware -- see its configure()); CONTROL/
    // MANIFEST_REQUEST stays forced to channel C, because only the dongle's
    // own aggregation cache legitimately sends one -- a TraceView hub-child
    // asks the DONGLE for a robot's manifest, never the robot directly (see
    // ManifestCache in bally_dongle), so there is nothing to widen there.
    //
    // This is the robot's REAL authorization now (see the long comment on
    // btp_command::authorized_source): the MAC check above is a cheap radio
    // prefilter a forged sender clears in one line, an AEAD tag is not.
    // RadioSeal::open()/open_e() fail closed on every branch (no key loaded,
    // ENCRYPTED not set, a cipher other than AES-128-GCM, or a tag that does
    // not verify); there is no fallback to reading the still-sealed bytes as
    // plaintext.
    const btp::Header& header = instance_->rx_routed_.header;
    const std::size_t ciphertext_size = instance_->rx_routed_.payload_size;
    const bool classify_by_peer =
        header.type == btp::MessageType::Command ||
        // TERMINAL_IN comes from TraceView's terminal widget through the hub,
        // sealed with key E (channel B) exactly like a COMMAND_REQUEST does.
        header.type == btp::MessageType::Terminal ||
        (header.type == btp::MessageType::Control &&
         (header.object_id == SubscriptionResponder::kSubscribeObjectId ||
          header.object_id == SubscriptionResponder::kUnsubscribeObjectId));
    const bally::Channel channel =
        classify_by_peer
            ? bally::channel_of_peer(bally::Vantage::Robot, header.source_id,
                                     instance_->dongle_source_id_)
            : bally::Channel::C_Link;
    const bool size_ok =
        ciphertext_size >= RadioSeal::kTagSize &&
        ciphertext_size - RadioSeal::kTagSize <= sizeof(instance_->rx_plaintext_);
    const bool opened =
        size_ok &&
        (channel == bally::Channel::B_Endpoint
             ? RadioSeal::open_e(header, static_cast<uint16_t>(ciphertext_size),
                                 instance_->rx_routed_.payload,
                                 instance_->rx_plaintext_)
             : RadioSeal::open(header, static_cast<uint16_t>(ciphertext_size),
                               instance_->rx_routed_.payload,
                               instance_->rx_plaintext_));
    if (!opened) {
        instance_->command_processor.note_unauthorized();
        return;
    }
    const std::size_t plaintext_size = ciphertext_size - RadioSeal::kTagSize;

    // Stage three: route. Explicit MessageType filter -- no other channel can
    // fall through to the shell path, even if its payload happens to look
    // like text. A type the robot has no handler for (TELEMETRY and LOG
    // today) is dropped HERE rather than before reassembly, so a fragmented
    // one costs a slot until it completes or times out. That is the price of
    // the ordering above, bounded by RxRouter::kSlotCount and its 4000 ms
    // timeout; filtering earlier is what loses real messages.
    switch (header.type) {
        case btp::MessageType::Command:
            if (header.object_id != btp_command::kCommandRequestObjectId) {
                instance_->command_processor.note_drop();
                return;
            }
            break;
        case btp::MessageType::Control:
            if (header.object_id != ManifestResponder::kManifestRequestObjectId &&
                header.object_id != SubscriptionResponder::kSubscribeObjectId &&
                header.object_id != SubscriptionResponder::kUnsubscribeObjectId) {
                instance_->command_processor.note_drop();
                return;
            }
            break;
        case btp::MessageType::Terminal:
            // TERMINAL_IN is the interactive shell channel (topico 19bis):
            // TraceView -> hub -> here, server-side line editing in
            // TerminalResponder, output mirrored back as TERMINAL_OUT.
            if (header.object_id != TerminalResponder::kTerminalInObjectId) {
                instance_->command_processor.note_drop();
                return;
            }
            break;
        case btp::MessageType::Telemetry:
        case btp::MessageType::Log:
        case btp::MessageType::Invalid:
            instance_->command_processor.note_drop();
            return;
    }

    instance_->dispatchDecoded(
        header, btp::ByteView{instance_->rx_plaintext_, plaintext_size},
        channel);
}

// Stage three of the pipeline above: hands one complete Command/Control
// message to the matching handler by object_id. handleReceiveStatic's switch
// already rejected any object_id not listed here, so the final else is
// unreachable in practice but kept as a safe no-op rather than an assert.
void ROBOT::dispatchDecoded(const btp::Header& header, btp::ByteView payload,
                           bally::Channel channel) {
    if (header.type == btp::MessageType::Terminal) {
        // Runs on the Wi-Fi RX task: TerminalResponder only buffers the bytes
        // here; runComms() drives the editor and runShell() the command.
        terminal_responder.on_terminal_in(header, payload);
        return;
    }
    if (header.type == btp::MessageType::Control) {
        if (header.object_id == ManifestResponder::kManifestRequestObjectId) {
            processManifestRequest(header, payload);
        } else if (header.object_id == SubscriptionResponder::kSubscribeObjectId) {
            processSubscribeRequest(header, payload, channel);
        } else if (header.object_id == SubscriptionResponder::kUnsubscribeObjectId) {
            processUnsubscribeRequest(header, payload, channel);
        }
    } else {
        processCommandRequest(header, payload, channel);
    }
}

void ROBOT::processCommandRequest(const btp::Header& header,
                                  btp::ByteView payload,
                                  bally::Channel channel) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    const CommandProcessor::Intake intake =
        command_processor.intake(header, payload, now_us, channel);
    if (intake.kind == CommandProcessor::IntakeKind::ResultReady) {
        command_processor.send_result(intake.result);
        return;
    }
    if (intake.kind != CommandProcessor::IntakeKind::Ready) return;

    QueuedCommand command{};
    command.cache_slot = intake.work.cache_slot;
    std::memcpy(command.text, intake.work.command, sizeof(command.text));
    if (xQueueSend(receivedDataQueue, &command, 0) != pdTRUE) {
        CommandProcessor::ResultView result{};
        if (command_processor.reject_busy(command.cache_slot, now_us,
                                          &result)) {
            command_processor.send_result(result);
        }
    }
}

void ROBOT::processManifestRequest(const btp::Header& header,
                                   btp::ByteView payload) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    manifest_responder.handle_request(header, payload, now_us);
}

void ROBOT::processSubscribeRequest(const btp::Header& header, btp::ByteView payload,
                                    bally::Channel channel) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    subscription_responder.handle_subscribe(header, payload, now_us, channel);
}

void ROBOT::processUnsubscribeRequest(const btp::Header& header, btp::ByteView payload,
                                      bally::Channel channel) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    subscription_responder.handle_unsubscribe(header, payload, now_us, channel);
}

void ROBOT::handleSendStatic(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    (void)tx_info;
    if (instance_ != nullptr) {
        instance_->tx_scheduler.on_delivery(status == ESP_NOW_SEND_SUCCESS);
    }
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
    // delay_flags and sysmon_freq_ms need nothing here -- checkStateMachine()
    // and the system-monitor task in main.cpp already read settings.data()
    // fresh on every pass, so they are live already.
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
    // save the pwm values to the control input vector for the EKF
    instance_->control_input[0] = static_cast<float>(instance_->motors.getValue(MOTOR_RIGHT_idx));
    instance_->control_input[1] = static_cast<float>(instance_->motors.getValue(MOTOR_LEFT_idx));

    instance_->getVelocitiesFromEncoders(instance_->measurement[0],
                                         instance_->measurement[1]);

    // Skipped while the IMU never answered at boot (imu_ready_ == false):
    // retrying an I2C transaction against a disconnected sensor here would
    // block this same timer callback on an I2C timeout every tick.
    if (instance_->imu_ready_) {
        instance_->imu->getAGT();
        instance_->measurement[2] = instance_->imu->gyrZ() * kDegToRad;
        instance_->measurement[3] = instance_->imu->accX() * kGravityMss;
        instance_->measurement[4] = instance_->imu->accY() * kGravityMss;
    } else {
        instance_->measurement[2] = 0.0f;
        instance_->measurement[3] = 0.0f;
        instance_->measurement[4] = 0.0f;
    }

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

        // run the EKF prediction and update steps with the latest control input and measurement
        instance_->EKF->predict(instance_->control_input);
        instance_->EKF->update(instance_->control_input, instance_->measurement);

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


// Snapshot for "link -delta". Deliberately not a reset: see the comment on
// LinkStatsBaseline in BallyRobot.h for why the counters themselves must stay
// monotonic since boot.
void ROBOT::captureLinkBaseline() {
    link_baseline_.set = true;
    link_baseline_.uptime_ms =
        static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
    link_baseline_.frames_rx = link_frames_rx_.load(std::memory_order_relaxed);
    link_baseline_.crc_errors = link_crc_errors_.load(std::memory_order_relaxed);
    link_baseline_.decode_errors =
        link_decode_errors_.load(std::memory_order_relaxed);
    link_baseline_.tx = tx_scheduler.stats();
    link_baseline_.rx = rx_router_.stats();
    link_baseline_.command = command_processor.stats();
}

// ==============================================================================
// LOCAL COMMANDS, JOBS AND SD SCRIPTS
// ==============================================================================

bool ROBOT::submitLocalCommand(const char* command_line) {
    if (command_line == nullptr || command_line[0] == '\0') return false;
    if (receivedDataQueue == nullptr) return false;

    const size_t length = std::strlen(command_line);
    if (length > btp_command::kMaxShellCommandSize) return false;

    QueuedCommand command{};
    command.cache_slot = kLocalCommandSlot;
    std::memcpy(command.text, command_line, length);
    command.text[length] = '\0';

    // Never block: this runs on the routine task, which also drains telemetry
    // and pumps the TX scheduler. A full queue is a "no", not a wait.
    return xQueueSend(receivedDataQueue, &command, 0) == pdTRUE;
}

bool ROBOT::submitLocalCommandStatic(void* context, const char* command_line) {
    ROBOT* self = static_cast<ROBOT*>(context);
    if (self == nullptr) return false;
    return self->submitLocalCommand(command_line);
}

bool ROBOT::submitTerminalCommand(uint32_t source_id, uint32_t boot_id,
                                  const char* command_line) {
    if (command_line == nullptr || command_line[0] == '\0') return false;
    if (receivedDataQueue == nullptr) return false;

    const size_t length = std::strlen(command_line);
    if (length > btp_command::kMaxShellCommandSize) return false;

    QueuedCommand command{};
    command.cache_slot = kTerminalCommandSlot;
    command.terminal_source_id = source_id;
    command.terminal_boot_id = boot_id;
    std::memcpy(command.text, command_line, length);
    command.text[length] = '\0';

    // Never block (comms task): a full queue is a "no" that TerminalResponder
    // reports back to the operator as "! busy, command dropped".
    return xQueueSend(receivedDataQueue, &command, 0) == pdTRUE;
}

bool ROBOT::submitTerminalCommandStatic(void* context, uint32_t source_id,
                                        uint32_t boot_id, const char* command_line) {
    ROBOT* self = static_cast<ROBOT*>(context);
    if (self == nullptr) return false;
    return self->submitTerminalCommand(source_id, boot_id, command_line);
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

void ROBOT::runShell(void *param) {
    (void)param; // Suppress unused parameter warning

    // Logger's terminal-output capture is gated on this handle, so only this
    // task's log lines (a running command's own output) are mirrored back to
    // TraceView's terminal.
    instance_->shell_task_handle_ = xTaskGetCurrentTaskHandle();

    while (instance_->receivedDataQueue == nullptr)
        vTaskDelay(100 / portTICK_PERIOD_MS); // wait

    // log the task
    logger.insert_log(logType::INFO, "Shell task started and ready to receive commands");

    // run the task
    while (true) {
        // delay for wathdog timer and to allow other tasks to run
        // in the begin of the loop to wait when no one command is received
        vTaskDelay(WDOG_TIMEOUT_TK);

        QueuedCommand received_command{};
        if (xQueueReceive(instance_->receivedDataQueue, &received_command, 0) != pdTRUE)
            continue;

        const bool from_terminal =
            received_command.cache_slot == kTerminalCommandSlot;

        // A terminal command's output is mirrored back to its origin as
        // TERMINAL_OUT (topico 19bis), so capture every log line it emits --
        // both the shell's own output_callback text and the insert_log()
        // calls inside the command bodies.
        if (from_terminal) {
            logger.begin_capture(instance_->shell_task_handle_);
        }

        // Execute exactly once, then publish the final correlated result. A
        // repeated request is answered from CommandProcessor's boot-lifetime
        // cache and never reaches TinyShell again.
        const uint8_t shell_status =
            instance_->shell.run_command_line(received_command.text);

        if (from_terminal) {
            std::string captured;
            logger.end_capture(captured);
            instance_->terminal_responder.deliver_command_output(
                received_command.terminal_source_id,
                received_command.terminal_boot_id, captured.c_str(), shell_status);
            continue;
        }

        // A job firing or a script line has no COMMAND_REQUEST behind it, so
        // there is nothing to correlate a COMMAND_RESULT with and no cache
        // entry to complete. Its output still goes out as LOG like any other.
        if (received_command.cache_slot == kLocalCommandSlot) continue;

        CommandProcessor::ResultView result{};
        if (instance_->command_processor.complete(
                received_command.cache_slot, shell_status,
                static_cast<uint64_t>(esp_timer_get_time()), &result)) {
            instance_->command_processor.send_result(result);
        }
    }
}

void ROBOT::runStateMachine(void *param) {
    (void)param; // Suppress unused parameter warning

    // log the task
    logger.insert_log(logType::INFO, "State machine task started");

    while (true) {
        // run state machine
        instance_->machine.run();

        // Sampling only copies a compact payload into a pre-allocated queue;
        // radio I/O happens later in routine().
        instance_->sampleTelemetry();
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
        // OTA holds the radio on the target Wi-Fi's channel, so ESP-NOW
        // frames sent while it's active never reach the peer; skip the
        // flush and let logs pile up in PSRAM instead of retrying/losing
        // them, then drain everything once cancel() gives the channel back.
        if (!instance_->ota.is_active()) {
            // Producers only enqueue encoded frames; the "comms" task drains
            // the scheduler and re-publishes STATUS (see runComms() below), so
            // a stall here never silences the link. This task still feeds the
            // scheduler with telemetry and log frames.
            instance_->telemetry.flush(4U);
            ROBOT::logger.flush_logs();              // send the logger messagens to output
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

// Dedicated radio task. Only two jobs, neither of which ever blocks:
//   - pump() the TX scheduler: it starts at most one ESP-NOW send per call and
//     returns immediately (the delivery callback lands on the Wi-Fi task), so
//     this loop's rate is what sets radio throughput. On routine() it was
//     throttled to routine()'s slowest pass;
//   - publishStatus(): self-gated to kStatusPeriodUs, so calling it every pass
//     costs nothing until one is actually due.
//
// The result: the dongle keeps hearing STATUS on channel C (hub.peers stays
// "online", MANIFEST_REQUEST priming keeps getting answered) and queued
// COMMAND_RESULT / MANIFEST_DATA leave promptly, regardless of what routine(),
// the logger or an SD flush are doing.
void ROBOT::runComms(void *param) {
    (void)param;

    while (!instance_->initialized)
        vTaskDelay(100 / portTICK_PERIOD_MS);

    while (true) {
        // Same OTA guard routine() uses: while OTA holds the radio on the
        // target Wi-Fi channel, ESP-NOW frames never reach the peer.
        if (!instance_->ota.is_active()) {
            // Drive the terminal editors before pumping the scheduler so a
            // keystroke echo / command output queued now leaves on this pass.
            instance_->terminal_responder.pump(
                static_cast<uint64_t>(esp_timer_get_time()));
            instance_->tx_scheduler.pump(
                static_cast<uint64_t>(esp_timer_get_time() / 1000ULL));
            instance_->publishStatus();
        }
        vTaskDelay(WDOG_TIMEOUT_TK);
    }
}

// CONTROL/STATUS, status_version=2 (commands.md sections 5/5.1).
// Spontaneous publication, no response expected, one per kStatusPeriodUs.
void ROBOT::publishStatus() {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    if (now_us < next_status_us_) return;
    next_status_us_ = now_us + kStatusPeriodUs;

    // READ, never sweep. The sweep happens inside RxRouter::submit(), on the
    // ESP-NOW receive callback -- doing it from here as well would mean two
    // tasks mutating the same slot table with nothing between them. See the
    // concurrency note in RxRouter.h.
    const uint32_t timeouts = rx_router_.stats().reassembly_timeouts;
    link_reassembly_timeouts_.store(static_cast<uint64_t>(timeouts),
                                    std::memory_order_relaxed);

    const TxScheduler::Stats tx = tx_scheduler.stats();
    const TelemetryPublisher::Stats telemetry_stats = telemetry.stats();
    const CommandProcessor::Stats command_stats = command_processor.stats();

    StatusReporter::Counters counters{};
    counters.frames_rx = link_frames_rx_.load(std::memory_order_relaxed);
    // "frames_tx": frames actually handed to the radio. "frames_dropped":
    // valid frames discarded by queues/capacity, which on this side means
    // the scheduler's own drops plus the publisher's full-queue drops.
    counters.frames_tx = tx.accepted;
    counters.frames_dropped =
        static_cast<uint64_t>(tx.dropped) + telemetry_stats.dropped_full;
    counters.crc_errors = link_crc_errors_.load(std::memory_order_relaxed);
    counters.decode_errors = link_decode_errors_.load(std::memory_order_relaxed);
    counters.reassembly_completed =
        link_reassembly_completed_.load(std::memory_order_relaxed);
    counters.reassembly_timeouts =
        link_reassembly_timeouts_.load(std::memory_order_relaxed);
    counters.reassembly_rejected =
        link_reassembly_rejected_.load(std::memory_order_relaxed);
    counters.command_duplicates = command_stats.duplicates;
    // A sample dropped by the full telemetry queue contributes both here and
    // to that topic's samples_dropped_total (section 5.1 allows exactly
    // that), but never twice inside the same counter.
    counters.telemetry_dropped = telemetry_stats.dropped_full;

    status_reporter.publish(0U, now_us, counters, now_us);
}

void ROBOT::sampleTelemetry() {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());

    // Topico 17 (assinaturas e controle de taxa): a lease that nobody
    // renewed falls back to "not publishing" here, before the gate below is
    // even checked -- this is the robot-side half of PASSO 6's disconnect
    // behavior (the dongle-side half clears its own aggregate and stops
    // sending SUBSCRIBE upstream; either one alone is enough to eventually
    // stop this topic).
    telemetry.expire_subscriptions(now_us);

    // protocol.test is periodic and now gated by an active subscription
    // (topic_period_us() returns 0 -- "don't publish" -- until the dongle
    // has forwarded at least one SUBSCRIBE for it). robot.state below stays
    // ungated: it is event-driven, not periodic, and section 3.3 of
    // commands.md defines max_rate_millihz=0 to mean exactly
    // that ("not periodic").
    const uint64_t period_us = telemetry.topic_period_us(TelemetryPublisher::kProtocolTestTopicId);
    if (period_us != 0U && now_us >= next_protocol_test_us_) {
        // IEEE-754 bits 3f 0d 0a 00 become PACKED_LE bytes 00 0a 0d 3f.
        // Every test sample therefore proves that NUL, LF and CR are data.
        constexpr uint32_t kEdgeFloatBits = 0x3F0D0A00U;
        float edge_value = 0.0f;
        std::memcpy(&edge_value, &kEdgeFloatBits, sizeof(edge_value));
        telemetry.publish_protocol_test(protocol_test_counter_++, edge_value,
                                        now_us);
        next_protocol_test_us_ = now_us + period_us;
    } else if (period_us == 0U) {
        // No active subscriber: keep the schedule pinned to "now" so the
        // first sample after a fresh SUBSCRIBE is emitted promptly (within
        // one sampleTelemetry() tick) instead of waiting out whatever
        // interval was in flight before the last subscriber went away.
        next_protocol_test_us_ = now_us;
    }

    const stateName current_state = static_cast<stateName>(
        machine.current_state.load(std::memory_order_acquire));
    if (current_state != last_telemetry_state_) {
        telemetry.publish_robot_state(static_cast<uint8_t>(current_state),
                                      now_us);
        last_telemetry_state_ = current_state;
    }
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
    shell.begin();

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

    if (!configureProtocolIdentity()) {
        ESP_LOGE("ROBOT_INIT", "Failed to configure BTP identity");
        return false;
    }

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

    // bally.key (lib/KeyStore): the two channel keys, already derived from
    // the two passwords by scripts/provision_key.py -- the robot never runs
    // PBKDF2 itself. Not fatal when missing or wrong, same as settings.conf
    // above: no channel is encrypted yet (see the AEAD comment on
    // handleReceiveStatic), so a robot without a card must still boot and
    // run on the bench. Only verify_e/verify_l are logged -- they are public
    // by construction; key_e()/key_l() must never be.
    if (key_store.load_from_card(sd_card)) {
        char verify_e_hex[KeyStore::kVerifyLength * 2U + 1U];
        char verify_l_hex[KeyStore::kVerifyLength * 2U + 1U];
        hexEncode(key_store.verify_e(), KeyStore::kVerifyLength, verify_e_hex);
        hexEncode(key_store.verify_l(), KeyStore::kVerifyLength, verify_l_hex);
        logger.insert_logf(logType::INFO,
                           "bally.key loaded: verify_e=%s verify_l=%s",
                           verify_e_hex, verify_l_hex);
    } else {
        logger.insert_logf(logType::WARN, "bally.key not loaded: %s",
                           KeyStore::error_field(key_store.last_error()));
    }

    // Configure the remaining pins now that settings are known, then build
    // the peripherals whose constructors need those pins (ArraySensor
    // touches GPIO/ADC immediately, so it cannot be built earlier).
    if (!configurePinsFromSettings()) {
        ESP_LOGE("ROBOT_INIT", "Failed to configure pins from settings");
        return false;
    }

    const SettingsData& cfg = settings.data();

    // Boot-time sub-mode select, in addition to the DEBUG-state shell
    // commands ("ota start" / "storage expose") which still work the normal
    // way. Read right after configurePinsFromSettings() configured the
    // button pins with a pull-up (pressed reads LOW) and before anything
    // else can move; actually acted on further down, once OTA/USB storage
    // themselves are ready (see ota.begin()/usb_storage.begin() below).
    //
    // cfg.btn0 is deliberately NOT used here: it defaults to GPIO0, the
    // ESP32-S3 boot strapping pin (see RobotSettings.h). Holding it while
    // the chip comes out of reset selects Joint Download Boot in ROM, so
    // the application never runs at all and this code is never reached --
    // the boot-time sub-mode select only works on non-strapping buttons.
    // OTA (cfg.btn2) wins if both are held.
    const bool boot_enter_ota = gpio_get_level(static_cast<gpio_num_t>(cfg.btn2)) == 0;
    const bool boot_enter_storage = !boot_enter_ota &&
        gpio_get_level(static_cast<gpio_num_t>(cfg.btn1)) == 0;

    array_sensor.emplace(cfg.s0, cfg.s1, cfg.s2, cfg.sig, cfg.len_sensor);
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
    sysmon.setOutputCallback([](const std::string& data) {
        if (!data.empty()) ROBOT::logger.insert_log(logType::DEBG, data.c_str());
    });
    sysmon.setLoggerCallback([]() { return ROBOT::logger.get_write_pct(); });

    receivedDataQueue = xQueueCreateStatic(
        kCommandQueueLength, sizeof(QueuedCommand),
        received_data_queue_storage_, &received_data_queue_control_);
    if (receivedDataQueue == nullptr) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to create receive queue");
        return false;
    }

    // Bind the job executor as soon as the queue it submits into exists, and
    // before startWrappers() can register the "job" commands: otherwise a
    // command arriving in that window would be refused with NotConfigured.
    jobs.configure(&ROBOT::submitLocalCommandStatic, this);

    // rx_router_ wires its own slots and storage in its constructor, so the
    // only thing that can be wrong here is that wiring, which is a
    // programming error -- checked once at boot and never again.
    if (!rx_router_.valid()) {
        ROBOT::logger.insert_log(logType::ERRO,
                                 "Failed to initialize BTP reassembly");
        return false;
    }

    // Must run before ota.begin(): it creates the default event loop and
    // the Wi-Fi STA netif (needed for the DHCP client — see the comment on
    // esp_netif_create_default_wifi_sta() inside configureCommunication())
    // that ota.begin()'s own event handler registration builds on top of.
    if (!configureCommunication())
        return false;

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
    if (!ota.begin(sd_card, leds, [](logType type, const char* msg) {
            ROBOT::logger.insert_log(type, msg);
        })) {
        logger.insert_log(logType::ERRO, "Failed to initialize OTA updater");
    }

    // Act on the boot-time sub-mode select read earlier: only takes effect
    // once the corresponding sub-mode actually starts, so a stray held
    // button with no stored Wi-Fi network (OTA) or no SD card (storage)
    // just boots normally instead of stranding the robot in DEBUG.
    if (boot_enter_ota) {
        if (ota.start()) {
            boot_state_ = DEBUG;
            logger.insert_log(logType::INFO,
                              "Boot: button 3 held, entering OTA mode directly");
        } else {
            logger.insert_log(logType::ERRO,
                              "Boot: button 3 held but OTA could not be started");
        }
    } else if (boot_enter_storage) {
        if (usb_storage.expose()) {
            boot_state_ = DEBUG;
            logger.insert_log(logType::INFO,
                              "Boot: button 2 held, entering USB storage mode directly");
        } else {
            logger.insert_log(logType::ERRO,
                              "Boot: button 2 held but USB storage could not be exposed");
        }
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

    // Not fatal: the IMU is optional hardware. Missing/unpowered, EKF just
    // keeps running on encoder-only measurements (see sampleEKF()).
    const int imu_begin_ret = imu->begin();
    imu_ready_              = imu_begin_ret > 0;
    if (!imu_ready_) {
        // begin()'s return codes (see ICM42688::begin()): -2 bus/device open
        // failed, -3 WHO_AM_I mismatch (something else is answering at this
        // address, or the chip isn't responding correctly), -4/-7/-8 a later
        // config/calibration step failed.
        ROBOT::logger.insert_logf(logType::WARN,
                                  "IMU (ICM42688) not detected (begin()=%d); EKF running on encoders only",
                                  imu_begin_ret);
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
    startWrappers();
    initEKF();
    registerSettingsAppliers();

    ROBOT::logger.insert_log(logType::INFO, "Welcome! the car is starting...");

    initialized = true;
    return true;
}
