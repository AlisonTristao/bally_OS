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

    logger.configure_btp(protocol);
    command_processor.configure(protocol);
    manifest_responder.configure(protocol, protocol_uuid_);
    telemetry.configure(protocol);
    subscription_responder.configure(protocol, telemetry);
    status_reporter.configure(protocol, telemetry);
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

void ROBOT::setOutputs() {
    motor_left->applyPWM(motors.getValue(MOTOR_LEFT_idx));
    motor_right->applyPWM(motors.getValue(MOTOR_RIGHT_idx));
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

void ROBOT::handleReceiveStatic(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    // Partial or malformed radio payloads are rejected by btp::decode before
    // any field is read. Only the authorized COMMAND_REQUEST route below can
    // enqueue work for TinyShell.
    if (instance_ == nullptr || recv_info == nullptr || incomingData == nullptr ||
        len <= 0 || instance_->receivedDataQueue == nullptr) return;

    // STATUS section 8 counters: every octet stream the radio hands us is one
    // received frame attempt; CRC and decode failures are counted separately
    // (a frame rejected by CRC is never also counted as a decode error).
    instance_->link_frames_rx_.fetch_add(1U, std::memory_order_relaxed);

    btp::DecodedFrame decoded{};
    const btp::Error decode_error = btp::decode(
        incomingData, static_cast<size_t>(len), btp::TransportProfile::EspNow,
        &decoded);
    if (decode_error != btp::Error::Ok) {
        if (decode_error == btp::Error::CrcMismatch) {
            instance_->link_crc_errors_.fetch_add(1U, std::memory_order_relaxed);
        } else {
            instance_->link_decode_errors_.fetch_add(1U, std::memory_order_relaxed);
        }
        return;
    }

#ifdef MAC_ADDR
    static constexpr uint8_t expected_peer[6] = {MAC_ADDR};
    if (!btp_command::authorized_source(expected_peer, recv_info->src_addr,
                                        decoded.header.source_id)) {
        instance_->command_processor.note_unauthorized();
        return;
    }
#else
    instance_->command_processor.note_unauthorized();
    return;
#endif

    // Explicit MessageType router. No other channel can fall through to the
    // shell path, even if its payload happens to look like text.
    switch (decoded.header.type) {
        case btp::MessageType::Command:
            if (decoded.header.object_id !=
                btp_command::kCommandRequestObjectId) {
                instance_->command_processor.note_drop();
                return;
            }
            break;
        case btp::MessageType::Control:
            if (decoded.header.object_id != ManifestResponder::kManifestRequestObjectId &&
                decoded.header.object_id != SubscriptionResponder::kSubscribeObjectId &&
                decoded.header.object_id != SubscriptionResponder::kUnsubscribeObjectId) {
                instance_->command_processor.note_drop();
                return;
            }
            break;
        case btp::MessageType::Telemetry:
        case btp::MessageType::Log:
        case btp::MessageType::Terminal:
        case btp::MessageType::Invalid:
            instance_->command_processor.note_drop();
            return;
    }

    if ((decoded.header.flags & btp::kFlagFragmented) == 0U) {
        instance_->dispatchDecoded(decoded.header, decoded.payload);
        return;
    }

    if (!instance_->command_reassembler_.has_value()) return;
    btp::ReassembledMessage completed{};
    const auto event = instance_->command_reassembler_->push(
        decoded, static_cast<uint64_t>(esp_timer_get_time() / 1000ULL),
        &completed);
    if (event == btp::ReassemblyEvent::Complete) {
        instance_->link_reassembly_completed_.fetch_add(1U, std::memory_order_relaxed);
        instance_->dispatchDecoded(completed.header, completed.payload);
        instance_->command_reassembler_->release(completed.slot_index);
    } else if (event != btp::ReassemblyEvent::Accepted &&
               event != btp::ReassemblyEvent::Duplicate) {
        instance_->link_reassembly_rejected_.fetch_add(1U, std::memory_order_relaxed);
        instance_->command_processor.note_drop();
    }
}

// Shared by the unfragmented and reassembled paths above: routes a fully
// decoded Command/Control frame to the matching handler by object_id.
// handleReceiveStatic's switch already rejected any object_id not listed
// here, so the final else is unreachable in practice but kept as a safe
// no-op rather than an assert.
void ROBOT::dispatchDecoded(const btp::Header& header, btp::ByteView payload) {
    if (header.type == btp::MessageType::Control) {
        if (header.object_id == ManifestResponder::kManifestRequestObjectId) {
            processManifestRequest(header, payload);
        } else if (header.object_id == SubscriptionResponder::kSubscribeObjectId) {
            processSubscribeRequest(header, payload);
        } else if (header.object_id == SubscriptionResponder::kUnsubscribeObjectId) {
            processUnsubscribeRequest(header, payload);
        }
    } else {
        processCommandRequest(header, payload);
    }
}

void ROBOT::processCommandRequest(const btp::Header& header,
                                  btp::ByteView payload) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    const CommandProcessor::Intake intake =
        command_processor.intake(header, payload, now_us);
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

void ROBOT::processSubscribeRequest(const btp::Header& header,
                                    btp::ByteView payload) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    subscription_responder.handle_subscribe(header, payload, now_us);
}

void ROBOT::processUnsubscribeRequest(const btp::Header& header,
                                      btp::ByteView payload) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    subscription_responder.handle_unsubscribe(header, payload, now_us);
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
    // filter's lifetime (see TinyEKF.h) — changing ekf_noise.* settings only
    // takes effect after the next reboot re-runs this function.
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

float ROBOT::getSpeedFromEncoders() {
    const SettingsData& cfg = settings.data();
    float left_speed = encoder_left->getCountDiff()/cfg.encoder_ppr * cfg.wheel_radius * 2.0f * PI * settings.freq_ekf();
    float right_speed = encoder_right->getCountDiff()/cfg.encoder_ppr * cfg.wheel_radius * 2.0f * PI * settings.freq_ekf();
    return (left_speed + right_speed) / 2.0f;
}

float ROBOT::getOmegaFromEncoders() {
    const SettingsData& cfg = settings.data();
    float left_speed = encoder_left->getCountDiff()/cfg.encoder_ppr * cfg.wheel_radius * 2.0f * PI * settings.freq_ekf();
    float right_speed = encoder_right->getCountDiff()/cfg.encoder_ppr * cfg.wheel_radius * 2.0f * PI * settings.freq_ekf();
    return (right_speed - left_speed) / EKF_WHEEL_BASE;
}

void ROBOT::sampleEKF(void *param) {
    // save the pwm values to the control input vector for the EKF
    instance_->control_input[0] = static_cast<float>(instance_->motors.getValue(MOTOR_RIGHT_idx));
    instance_->control_input[1] = static_cast<float>(instance_->motors.getValue(MOTOR_LEFT_idx));

    instance_->measurement[0] = instance_->getSpeedFromEncoders();
    instance_->measurement[1] = instance_->getOmegaFromEncoders();

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

void ROBOT::startWrappers() {
    // Modules owned by ROBOT itself — see the declarations in BallyRobot.h
    // for why each one stays here instead of moving to a subsystem lib.
    registerRobotIOCommands();
    registerKalmanCommands();
    registerDebugCommands();

    // Every other module is owned by the subsystem it operates on. "sensor"
    // and "storage" are each split across two owners below (array sensor vs.
    // encoders; SD file management vs. USB ownership) — both register into
    // the same TinyShell module name, which TinyShell allows.
    array_sensor->register_shell_commands(shell, logger, settings);
    Encoder::register_shell_commands(shell, logger, *encoder_left, *encoder_right);
    junkebox->register_shell_commands(shell, logger);

    usb_storage.register_shell_commands(
        shell, logger, sd_card,
        [this]() { return anyDebugTestActive(); },
        [this]() { sendNextShellOutputDirect(); });
    sd_card.register_shell_commands(shell, logger);

    logger.register_shell_commands(
        shell, sd_card, settings,
        [this]() { sendNextShellOutputDirect(); });

    ota.register_shell_commands(
        shell, logger, sd_card, usb_storage,
        [this]() { return anyDebugTestActive(); },
        [this]() { sendNextShellOutputDirect(); });

    // "set"/"reset"/"reset_all" only change the in-memory copy; "save"
    // persists it, and every change needs a reboot to actually take effect
    // (pins, timers and EKF init are all applied once, at boot).
    settings.register_shell_commands(
        shell, logger, sd_card,
        [this]() { sendNextShellOutputDirect(); });

#ifdef ENABLE_SYSTEM_MONITOR
    sysmon.register_shell_commands(shell, logger);
#endif
}

void ROBOT::registerRobotIOCommands() {
    // Raw actuator/virtual-input I/O: motors, LEDs, virtual button/side-sensor triggers.
    shell.create_module("robot", "Raw actuator and virtual-input commands");

    shell.add([](uint8_t btn_idx) -> uint8_t {
        // set the flag of the button with the given index
        if (btn_idx >= Flags_in::MAX_FLAGS)
            return RESULT_ERROR;
        instance_->buttons.setFlag(btn_idx);
        return RESULT_OK;
    }, "btn", "Virtually trigger a button", "robot");

    shell.add([](uint8_t ssr_idx) -> uint8_t {
        // set the flag of the side sensor with the given index
        if (ssr_idx >= Flags_in::MAX_FLAGS)
            return RESULT_ERROR;
        instance_->sideSensors.setFlag(ssr_idx);
        return RESULT_OK;
    }, "ssr", "Virtually trigger a side sensor", "robot");

    shell.add([](uint8_t led_idx, int8_t pwm_value, uint32_t time) -> uint8_t {
        // set the PWM value for the motor with the given index (-100..100)
        if (led_idx >= Flags_in::MAX_FLAGS)
            return RESULT_ERROR;
        instance_->motors.setValue(led_idx, pwm_value, time);
        return RESULT_OK;
    }, "set_pwm", "Set PWM value for a motor (0 for left, 1 for right)", "robot");

    shell.add([](int8_t left_pwm, int8_t right_pwm, uint32_t time) -> uint8_t {
        instance_->motors.setValue(MOTOR_LEFT_idx, left_pwm, time);
        instance_->motors.setValue(MOTOR_RIGHT_idx, right_pwm, time);
        return RESULT_OK;
    }, "set_pwm_pair", "Set PWM values for both motors at once", "robot");

    shell.add([](uint8_t pin, uint32_t time) -> uint8_t {
        instance_->leds.setFlag(pin, time);
        return RESULT_OK;
    }, "set_led", "turn on a LED", "robot");

    // Not a Flags_pwm command like the ones above: the buzzer needs a
    // frequency in Hz, not a -100..100 duty percentage, so it doesn't fit
    // that struct's semantics. Junkebox::play_tone() gives the same
    // "one-shot, auto-stops after `time`" safety directly instead.
    shell.add([](uint32_t freq_hz, uint32_t time) -> uint8_t {
        if (!instance_->junkebox->play_tone(static_cast<uint16_t>(freq_hz), time)) {
            ROBOT::logger.insert_log(logType::ERRO, "Junkebox is not initialized yet");
            return RESULT_ERROR;
        }
        return RESULT_OK;
    }, "test_buzzer", "Play a raw tone on the buzzer, bypassing note parsing: freq_hz,duration_ms", "robot");
}

void ROBOT::registerKalmanCommands() {
    // EKF (Kalman) state and periodic tuning log. Unlike the "debug" module
    // below, this is available in any state (including RUN) since tuning
    // the filter means watching it while the robot is actually driving.
    shell.create_module("kalman", "EKF state and periodic tuning log");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_logf(logType::INFO, "EKF state: v=%.4f w=%.4f b=%.4f",
                                  instance_->EKF->get_state(0),
                                  instance_->EKF->get_state(1),
                                  instance_->EKF->get_state(2));
        return RESULT_OK;
    }, "state", "Read the current EKF state (v, w, gyro bias)", "kalman");

    shell.add([](uint32_t interval_ms) -> uint8_t {
        if (interval_ms == 0 ||
            !instance_->kalman_log_test_.schedule(UINT32_MAX, interval_ms)) {
            ROBOT::logger.insert_log(logType::ERRO,
                                     "Invalid interval; kalman log not started");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(logType::INFO,
                                  "Kalman log started: every %u ms", interval_ms);
        return RESULT_OK;
    }, "start_log", "Start periodic EKF state + measurement logging: interval_ms", "kalman");

    shell.add([]() -> uint8_t {
        instance_->kalman_log_test_.cancel();
        ROBOT::logger.insert_log(logType::INFO, "Kalman log stopped");
        return RESULT_OK;
    }, "stop_log", "Stop periodic EKF logging", "kalman");
}

void ROBOT::registerDebugCommands() {
    // DEBUG sensor tests: each schedules periodic, non-blocking sampling
    // (see ScheduledDebugTest/processDebug). Add one command per sensor
    // here — IMU and H-bridge current are planned next.
    shell.create_module("debug", "Safe non-blocking sensor tests");

    shell.add([](uint32_t samples, uint32_t interval_ms) -> uint8_t {
        if (!instance_->scheduleDebugTest(instance_->array_sensor_test_,
                                          samples, interval_ms)) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "Array test requires DEBUG state, USB inactive and samples > 0");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(
            logType::INFO,
            "Array sensor test scheduled: %u samples every %u ms",
            samples, interval_ms);
        return RESULT_OK;
    }, "test_arr_sensor", "Print sensor array: samples,interval_ms", "debug");

    shell.add([](uint32_t samples, uint32_t interval_ms) -> uint8_t {
        if (!instance_->scheduleDebugTest(instance_->encoder_test_,
                                          samples, interval_ms)) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "Encoder test requires DEBUG state, USB inactive and samples > 0");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(
            logType::INFO,
            "Encoder test scheduled: %u samples every %u ms",
            samples, interval_ms);
        return RESULT_OK;
    }, "test_encoder", "Print left/right encoder counts: samples,interval_ms", "debug");

    shell.add([](uint32_t samples, uint32_t interval_ms) -> uint8_t {
        // Same reasoning as sampleEKF()'s imu_ready_ guard: without it, a
        // sensor that never answered at boot would retry (and block on) an
        // I2C timeout every sample instead of failing once, up front.
        if (!instance_->imu_ready_) {
            ROBOT::logger.insert_log(logType::ERRO,
                                     "IMU not ready; nothing was detected at boot");
            return RESULT_ERROR;
        }

        if (!instance_->scheduleDebugTest(instance_->imu_test_,
                                          samples, interval_ms)) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "IMU test requires DEBUG state, USB inactive and samples > 0");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(
            logType::INFO,
            "IMU test scheduled: %u samples every %u ms",
            samples, interval_ms);
        return RESULT_OK;
    }, "test_imu", "Print IMU accel/gyro/temp readings: samples,interval_ms", "debug");

    shell.add([]() -> uint8_t {
        // Reuses the bus imu->begin() already opened on cfg.sda_pin/scl_pin
        // instead of opening a second i2c_master_bus on the same pins (see
        // ICM42688::scanBus()'s comment) -- so this works on demand, without
        // a reboot, even when imu_ready_ is false.
        if (!instance_->imu->busOpen()) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "IMU I2C bus never opened (begin() failed before that point); nothing to scan");
            return RESULT_ERROR;
        }

        std::string found;
        const bool  any_found = instance_->imu->scanBus(found);
        ROBOT::logger.insert_logf(
            logType::INFO,
            "I2C scan (IMU bus): %s | WHO_AM_I=0x%02X (expect 0x47)",
            any_found ? found.c_str() : "no devices found",
            instance_->imu->whoAmI());
        return RESULT_OK;
    }, "scan_i2c", "Probe the IMU I2C bus and read WHO_AM_I; works even if the IMU wasn't detected at boot", "debug");

    shell.add([](uint32_t samples, uint32_t interval_ms) -> uint8_t {
        // Same reuse-the-existing-bus reasoning as scan_i2c: no imu_ready_
        // gate here on purpose -- the whole point is to keep re-checking
        // even after a failed boot detection, since the bus can (and does,
        // on a noisy/pull-up-less setup) recover a moment later.
        if (!instance_->imu->busOpen()) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "IMU I2C bus never opened (begin() failed before that point); nothing to check");
            return RESULT_ERROR;
        }

        instance_->imu_i2c_ok_count_   = 0;
        instance_->imu_i2c_fail_count_ = 0;
        if (!instance_->scheduleDebugTest(instance_->imu_i2c_test_,
                                          samples, interval_ms)) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "I2C stability test requires DEBUG state, USB inactive and samples > 0");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(
            logType::INFO,
            "I2C stability test scheduled: %u checks every %u ms",
            samples, interval_ms);
        return RESULT_OK;
    }, "test_i2c", "Re-check WHO_AM_I on a timer and tally ok/fail, to watch bus stability live: samples,interval_ms", "debug");
}

// ==============================================================================
// MAIN TASKS & INITIALIZATION
// ==============================================================================

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

    // set the interrupt type for the buttons and side sensors,
    // and add the corresponding ISR handlers to set the flags when the interrupts are triggered
    gpio_set_intr_type(btn0, GPIO_INTR_NEGEDGE); // FALLING
    gpio_isr_handler_add(btn0, [](void* arg) IRAM_ATTR {
        instance_->buttons.setFlag(BIT_0);
    }, nullptr);

    gpio_set_intr_type(btn1, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(btn1, [](void* arg) IRAM_ATTR {
        instance_->buttons.setFlag(BIT_1);
    }, nullptr);

    gpio_set_intr_type(btn2, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(btn2, [](void* arg) IRAM_ATTR {
        instance_->buttons.setFlag(BIT_2);
    }, nullptr);

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

    // set the timer to trigger the EKF at the defined sample rate
    esp_timer_handle_t timer;
    esp_timer_create(&timer_args, &timer);
    esp_timer_start_periodic(timer, cfg.sample_micros);

    vTaskDelete(NULL);
}

void ROBOT::runShell(void *param) {
    (void)param; // Suppress unused parameter warning

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
        
        // Execute exactly once, then publish the final correlated result. A
        // repeated request is answered from CommandProcessor's boot-lifetime
        // cache and never reaches TinyShell again.
        const uint8_t shell_status =
            instance_->shell.run_command_line(received_command.text);
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

    // excute the loop to menage the robot
    while(true) {
        // OTA holds the radio on the target Wi-Fi's channel, so ESP-NOW
        // frames sent while it's active never reach the peer; skip the
        // flush and let logs pile up in PSRAM instead of retrying/losing
        // them, then drain everything once cancel() gives the channel back.
        if (!instance_->ota.is_active()) {
            // Producers only enqueue encoded frames. The scheduler drains one
            // callback-correlated ESP-NOW attempt at a time, always selecting
            // COMMAND_RESULT before status, critical logs, telemetry and debug.
            instance_->telemetry.flush(4U);
            ROBOT::logger.flush_logs();              // send the logger messagens to output
            instance_->tx_scheduler.pump(
                static_cast<uint64_t>(esp_timer_get_time() / 1000ULL));
            // Topico 17 PASSOS 8/9. Runs on the comms task, after the
            // telemetry drain, so the control loop and the state-machine
            // task never see it.
            instance_->publishStatus();
        }
        instance_->resetFlags();                    // reset the flags - buttons, side sensors, pwm...
        instance_->setOutputs();                    // set the output - leds, pwm...
        instance_->checkStateMachine();             // cheg the next state of the state machine
        instance_->updateSoundFeedback();           // react to the changes above with a Junkebox sound
        vTaskDelay(WDOG_TIMEOUT_TK); // delay for wathdog timer and to allow other tasks to run
    }
}

// CONTROL/STATUS, status_version=2 (COMMANDS_AND_ACTIONS.md sections 8/8.1).
// Spontaneous publication, no response expected, one per kStatusPeriodUs.
void ROBOT::publishStatus() {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    if (now_us < next_status_us_) return;
    next_status_us_ = now_us + kStatusPeriodUs;

    // Nothing else in the firmware sweeps abandoned reassembly slots, and a
    // timed-out partial message is exactly what reassembly_timeouts counts.
    if (command_reassembler_.has_value()) {
        const std::size_t expired =
            command_reassembler_->expire(now_us / 1000ULL);
        if (expired != 0U) {
            link_reassembly_timeouts_.fetch_add(
                static_cast<uint64_t>(expired), std::memory_order_relaxed);
        }
    }

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
    // to that topic's samples_dropped_total (section 8.1 allows exactly
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
    // ungated: it is event-driven, not periodic, and section 6.2 of
    // COMMANDS_AND_ACTIONS.md defines max_rate_millihz=0 to mean exactly
    // that ("nao periodico").
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

    if (!configureProtocolIdentity()) {
        ESP_LOGE("ROBOT_INIT", "Failed to configure BTP identity");
        return false;
    }

    // Initialize the card and give FAT ownership to the USB storage manager.
    // It starts mounted for the robot and only exposes it on a DEBUG command.
    if (!sd_card.begin()) {
        logger.insert_log(logType::ERRO, "Failed to initialize SD card");
        ESP_LOGE("ROBOT_INIT", "Failed to initialize SD card");
    } else if (!usb_storage.begin(sd_card)) {
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

    // Configure the remaining pins now that settings are known, then build
    // the peripherals whose constructors need those pins (ArraySensor
    // touches GPIO/ADC immediately, so it cannot be built earlier).
    if (!configurePinsFromSettings()) {
        ESP_LOGE("ROBOT_INIT", "Failed to configure pins from settings");
        return false;
    }

    const SettingsData& cfg = settings.data();
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

#ifdef ENABLE_SYSTEM_MONITOR
    sysmon.begin();
    sysmon.setOutputCallback([](const std::string& data) {
        if (!data.empty()) ROBOT::logger.insert_log(logType::DEBG, data.c_str());
    });
    sysmon.setLoggerCallback([]() { return ROBOT::logger.get_write_pct(); });
#endif

    receivedDataQueue = xQueueCreateStatic(
        kCommandQueueLength, sizeof(QueuedCommand),
        received_data_queue_storage_, &received_data_queue_control_);
    if (receivedDataQueue == nullptr) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to create receive queue");
        return false;
    }

    for (size_t index = 0U; index < kCommandReassemblySlots; ++index) {
        command_reassembly_storage_[index] = {
            command_reassembly_buffers_[index],
            sizeof(command_reassembly_buffers_[index]),
        };
    }
    command_reassembler_.emplace(
        command_reassembly_slots_, command_reassembly_storage_,
        kCommandReassemblySlots, 2000U);
    if (!command_reassembler_->valid()) {
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

    ROBOT::logger.insert_log(logType::INFO, "Welcome! the car is starting...");

    initialized = true;
    return true;
}
