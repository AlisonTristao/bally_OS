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
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include <cmath>
#include <sys/time.h>
#ifndef PI
#define PI 3.14159265358979323846
#endif

// IMU I2C address (AD0 strapped low); gyro/accel unit conversions for the EKF.
static constexpr uint8_t IMU_I2C_ADDRESS = 0x68;
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

static void formatBytes(uint64_t bytes, char* output, size_t capacity) {
    const char* unit = "B";
    double value = static_cast<double>(bytes);

    if (bytes >= (1024ULL * 1024ULL * 1024ULL)) {
        value /= 1024.0 * 1024.0 * 1024.0;
        unit = "GB";
    } else if (bytes >= (1024ULL * 1024ULL)) {
        value /= 1024.0 * 1024.0;
        unit = "MB";
    } else if (bytes >= 1024ULL) {
        value /= 1024.0;
        unit = "kB";
    }

    snprintf(output, capacity, "%.2f %s", value, unit);
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

uint8_t testPacket() {
    // Envia um texto longo numerado para testar fragmentacao e perda de pacotes
    // Referencia: Terminal log da GLaDOS (Portal) - "Still Alive"
    const char* long_text =
        "[01/14] Forms FORM-29827281-12-2: Notice of Dismissal\n"
        "[02/14] Aperture Science computer-aided enrichment center\n"
        "[03/14] This was a triumph.\n"
        "[04/14] I'm making a note here: HUGE SUCCESS.\n"
        "[05/14] It's hard to overstate my satisfaction.\n"
        "[06/14] Aperture Science\n"
        "[07/14] We do what we must because we can.\n"
        "[08/14] For the good of all of us.\n"
        "[09/14] Except the ones who are dead.\n"
        "[10/14] But there's no sense crying over every mistake.\n"
        "[11/14] You just keep on trying till you run out of cake.\n"
        "[12/14] And the Science gets done.\n"
        "[13/14] And you make a neat gun.\n"
        "[14/14] For the people who are still alive.\n";

    // When info logs are enabled, push to logger to validate its packetization path.
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log(logType::INFO, long_text);
        return RESULT_OK;
    #endif
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

bool ROBOT::configureCommunication() {
    if (communication_configured_) return true;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

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
    return array_sensor_test_.active() || encoder_test_.active();
}

void ROBOT::processDebug() {
    // DEBUG is a safe test state: keep both motor commands at zero on every
    // pass, while the normal outer task delay continues yielding the CPU.
    motors.setValue(MOTOR_LEFT_idx, 0, settings.data().delay_flags);
    motors.setValue(MOTOR_RIGHT_idx, 0, settings.data().delay_flags);

    usb_storage.process();
    if (usb_storage.is_active()) return;

    ota.process(buttons.getFlags());
    if (ota.is_active()) return;

    // Add one `if (test.poll()) { ... }` block per ScheduledDebugTest member
    // (IMU, H-bridge current, ...).
    if (array_sensor_test_.poll()) {
        logger.insert_log(logType::INFO, array_sensor->debug().c_str());
    }

    if (encoder_test_.poll()) {
        logger.insert_logf(
            logType::INFO, "Encoders: left=%lld right=%lld",
            static_cast<long long>(encoder_left->getCount()),
            static_cast<long long>(encoder_right->getCount()));
    }
}

void ROBOT::cancelDebugTests() {
    array_sensor_test_.cancel();
    encoder_test_.cancel();
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

// ==============================================================================
// COMMUNICATION CALLBACKS
// ==============================================================================

void ROBOT::handleReceiveStatic(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    const uint8_t* mac = recv_info->src_addr;
    (void)mac;

    if (instance_->receivedDataQueue == nullptr) {
        ROBOT::logger.insert_log(logType::ERRO, "Receive callback called but queue is not initialized");
        return;
    }

    // Only a full `message` struct is safe to read below; anything else
    // (partial/garbage frame) is dropped instead of read out of bounds —
    // incomingData is only `len` bytes long, not necessarily sizeof(message).
    if (len != static_cast<int>(sizeof(message))) return;

    // Heartbeat probe from the T-Dongle (see SharedMessageTypes.h): the
    // ESP-NOW driver already sent the low-level delivery ACK back to it on
    // the radio itself, which is all "we're connected" needs. Drop it here,
    // before the queue, so it never reaches the shell (as an empty command
    // line) or the retained PSRAM log.
    if (reinterpret_cast<const message*>(incomingData)->type == logType::PING) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(instance_->receivedDataQueue, incomingData, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void ROBOT::handleSendStatic(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    (void)tx_info;
    (void)status;
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

    // Array sensor (line follower) and wheel encoders.
    shell.create_module("sensor", "Array sensor (line follower) and wheel encoders");

    shell.add([]() -> uint8_t {
        if (StateMachine::current_state.load(std::memory_order_acquire) == RUN) {
            ROBOT::logger.insert_log(logType::ERRO,
                                     "Calibration blocked while RUN is active");
            return RESULT_ERROR;
        }

        const SettingsData& cfg = instance_->settings.data();
        const bool ok = instance_->array_sensor->calibrate(cfg.samples, cfg.delay_sample);
        ROBOT::logger.insert_logf(logType::INFO, "Calibration %s",
                                  ok ? "succeeded" : "failed");
        return ok ? RESULT_OK : RESULT_ERROR;
    }, "calibrate", "Calibrate the array sensor using the configured sample count", "sensor");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_log(logType::INFO,
                                 instance_->array_sensor->calibrate_status().c_str());
        return RESULT_OK;
    }, "calibrate_status", "Show the array sensor's stored calibration values", "sensor");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_logf(logType::INFO, "Line position: %.3f",
                                  instance_->array_sensor->get_line_position());
        return RESULT_OK;
    }, "position", "Read the current normalized line position (-1..1)", "sensor");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_log(logType::INFO, instance_->array_sensor->raw().c_str());
        return RESULT_OK;
    }, "raw", "Show raw ADC readings for every sensor channel", "sensor");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_logf(
            logType::INFO, "Encoders: left=%lld right=%lld",
            static_cast<long long>(instance_->encoder_left->getCount()),
            static_cast<long long>(instance_->encoder_right->getCount()));
        return RESULT_OK;
    }, "encoders", "Read the raw left/right encoder counts", "sensor");

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

    // SD card and native USB MSC ownership.
    shell.create_module("storage", "SD card file management and USB MSC ownership");

    shell.add([]() -> uint8_t {
        if (StateMachine::current_state.load(std::memory_order_acquire) !=
            DEBUG) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "USB storage requires DEBUG state; enter DEBUG first");
            return RESULT_ERROR;
        }

        if (instance_->usb_storage.is_exposed()) {
            ROBOT::logger.send_log_direct(logType::INFO,
                                          "USB storage is already enabled");
            return RESULT_OK;
        }

        if (!instance_->usb_storage.is_ready() ||
            !instance_->usb_storage.app_has_access() ||
            !instance_->sd_card.is_mounted()) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "USB storage unavailable: SD card is not mounted for robot");
            return RESULT_ERROR;
        }

        if (instance_->sd_card.has_open_stream()) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "USB storage blocked: close the active SD file first");
            return RESULT_ERROR;
        }

        if (instance_->anyDebugTestActive()) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "USB storage blocked: wait for the DEBUG test to finish");
            return RESULT_ERROR;
        }

        if (!instance_->usb_storage.expose()) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "Failed to transfer SD card ownership to native USB");
            return RESULT_ERROR;
        }

        ROBOT::logger.send_log_direct(
            logType::INFO,
            "USB storage enabled; safely eject the drive on the PC before leaving DEBUG");
        instance_->sendNextShellOutputDirect();
        return RESULT_OK;
    }, "expose", "Expose the SD card through native USB MSC (DEBUG state)", "storage");

    shell.add([]() -> uint8_t {
        if (!instance_->usb_storage.is_ready()) {
            ROBOT::logger.insert_log(logType::ERRO,
                                     "USB storage is not initialized");
            return RESULT_ERROR;
        }

        const char* owner = nullptr;
        if (instance_->usb_storage.is_exposed()) {
            owner = "PC owns SD";
        } else if (instance_->usb_storage.is_active()) {
            owner = instance_->usb_storage.host_is_attached()
                ? "PC connected, waiting for media"
                : "waiting for PC connection";
        } else {
            owner = "robot owns SD";
        }

        char capacity_text[24];
        formatBytes(instance_->usb_storage.capacity_bytes(), capacity_text,
                    sizeof(capacity_text));
        char status[96];
        snprintf(status, sizeof(status), "USB storage: %s; media=%s",
                 owner, capacity_text);

        ROBOT::logger.send_log_direct(logType::INFO, status);
        instance_->sendNextShellOutputDirect();
        return RESULT_OK;
    }, "status", "Show current SD card owner", "storage");

    shell.add([]() -> uint8_t {
        uint64_t total_bytes = 0;
        uint64_t used_bytes = 0;
        uint64_t free_bytes = 0;

        if (!instance_->sd_card.get_storage_info(
                total_bytes, used_bytes, free_bytes)) {
            ROBOT::logger.insert_log(logType::ERRO,
                                     "Failed to read SD card usage");
            return RESULT_ERROR;
        }

        const float used_percent = total_bytes == 0
            ? 0.0f
            : (used_bytes * 100.0f) / static_cast<float>(total_bytes);

        char total_text[24];
        char used_text[24];
        char free_text[24];
        formatBytes(total_bytes, total_text, sizeof(total_text));
        formatBytes(used_bytes, used_text, sizeof(used_text));
        formatBytes(free_bytes, free_text, sizeof(free_text));

        ROBOT::logger.insert_logf(
            logType::INFO,
            "SD total=%s used=%s free=%s (%.2f%%)",
            total_text, used_text, free_text,
            used_percent);
        return RESULT_OK;
    }, "usage", "Show SD card total, used and free bytes", "storage");

    shell.add([]() -> uint8_t {
        if (!instance_->sd_card.is_mounted()) return RESULT_ERROR;

        const uint16_t count = instance_->sd_card.get_file_count();
        ROBOT::logger.insert_logf(logType::INFO, "SD files: %u", count);

        for (uint16_t index = 0; index < count; ++index) {
            SDFileInfo info{};
            if (!instance_->sd_card.get_file_info(index, info)) continue;

            char size_text[24];
            formatBytes(info.size, size_text, sizeof(size_text));

            ROBOT::logger.insert_logf(
                logType::INFO, "[%u] %s (%s)", index, info.name, size_text);
        }

        return RESULT_OK;
    }, "list_logs", "List files stored at the SD card root", "storage");

    shell.add([](uint16_t file_index) -> uint8_t {
        SDFileInfo info{};
        if (!instance_->sd_card.get_file_info(file_index, info) ||
            !instance_->sd_card.remove_file(info.name)) {
            ROBOT::logger.insert_log(logType::ERRO,
                                     "Invalid log index or delete failed");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(logType::INFO, "Deleted %s", info.name);
        return RESULT_OK;
    }, "delete_log", "Delete a file by index (see list_logs)", "storage");

    shell.add([](uint16_t file_index, std::string new_name) -> uint8_t {
        SDFileInfo info{};
        if (!instance_->sd_card.get_file_info(file_index, info) ||
            !instance_->sd_card.rename_file(info.name, new_name.c_str())) {
            ROBOT::logger.insert_log(logType::ERRO,
                                     "Invalid log index or rename failed");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(logType::INFO, "Renamed %s to %s",
                                  info.name, new_name.c_str());
        return RESULT_OK;
    }, "rename_log", "Rename a file by index: file_index,new_name", "storage");

    // Retained PSRAM log buffer and its SD-backed files.
    shell.create_module("logger", "Retained PSRAM log buffer and SD log files");

    shell.add(testPacket, "test_packet",
             "Send a long test packet to evaluate multi-packet handling", "logger");

    shell.add([]() -> uint8_t {
        char used_text[24];
        char capacity_text[24];
        formatBytes(ROBOT::logger.get_used_bytes(), used_text,
                    sizeof(used_text));
        formatBytes(ROBOT::logger.get_capacity_bytes(), capacity_text,
                    sizeof(capacity_text));

        ROBOT::logger.insert_logf(
            logType::INFO,
            "PSRAM logger used=%s capacity=%s (%.2f%%)",
            used_text,
            capacity_text,
            ROBOT::logger.get_write_pct());
        return RESULT_OK;
    }, "psram_usage", "Show retained Logger PSRAM usage", "logger");

    shell.add([](uint16_t year, uint8_t month, uint8_t day,
                 uint8_t hour, uint8_t minute, uint8_t second) -> uint8_t {
        if (!ROBOT::logger.set_datetime(year, month, day, hour, minute, second,
                                        instance_->settings.data().timezone)) {
            ROBOT::logger.insert_log(logType::ERRO,
                                     "Invalid date/time or clock update failed");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(
            logType::INFO,
            "Robot time updated: %04u-%02u-%02u %02u:%02u:%02u",
            year, month, day, hour, minute, second);
        return RESULT_OK;
    }, "set_datetime", "Set local time: year,month,day,hour,minute,second",
       "logger");

    shell.add([]() -> uint8_t {
        char filename[SDFileInfo::MAX_NAME_LENGTH] = {};
        if (!ROBOT::logger.flush_to_sd(instance_->sd_card, false, filename,
                                       sizeof(filename))) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "Failed to create a new SD log; synchronize date/time first");
            return RESULT_ERROR;
        }

        char response[sizeof(filename) + 32];
        snprintf(response, sizeof(response), "PSRAM saved to %s", filename);
        ROBOT::logger.send_log_direct(logType::INFO, response);
        instance_->sendNextShellOutputDirect();
        return RESULT_OK;
    }, "flush_new", "Save retained PSRAM logs into a new dated file", "logger");

    shell.add([]() -> uint8_t {
        char filename[SDFileInfo::MAX_NAME_LENGTH] = {};
        if (!ROBOT::logger.flush_to_sd(instance_->sd_card, true, filename,
                                       sizeof(filename))) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "Failed to append PSRAM logs to the latest SD log");
            return RESULT_ERROR;
        }

        char response[sizeof(filename) + 36];
        snprintf(response, sizeof(response), "PSRAM appended to %s", filename);
        ROBOT::logger.send_log_direct(logType::INFO, response);
        instance_->sendNextShellOutputDirect();
        return RESULT_OK;
    }, "flush_append", "Append retained PSRAM logs to the latest log file",
       "logger");

    shell.add([](uint16_t file_index, uint32_t delay_msg_ms) -> uint8_t {
        SDFileInfo info{};
        if (!instance_->sd_card.get_file_info(file_index, info) ||
            info.size % sizeof(message) != 0 ||
            !instance_->sd_card.open_read_stream(info.name)) {
            ROBOT::logger.insert_log(logType::ERRO,
                                     "Invalid log index or file format");
            return RESULT_ERROR;
        }

        bool success = true;
        uint32_t sent_messages = 0;

        while (true) {
            message stored_message{};
            const size_t read = instance_->sd_card.read_stream(
                &stored_message, sizeof(stored_message));

            if (read == 0) break;
            if (read != sizeof(stored_message) ||
                stored_message.content.size > MAX_CONTENT_SIZE ||
                stored_message.packet_number == 0 ||
                stored_message.total_packets == 0 ||
                !ROBOT::logger.send_message(stored_message)) {
                success = false;
                break;
            }

            ++sent_messages;
            if (delay_msg_ms > 0) {
                TickType_t delay_ticks = pdMS_TO_TICKS(delay_msg_ms);
                if (delay_ticks == 0) delay_ticks = 1;
                vTaskDelay(delay_ticks);
            }
        }

        if (instance_->sd_card.stream_has_error()) success = false;
        if (!instance_->sd_card.close_stream()) success = false;

        ROBOT::logger.insert_logf(
            success ? logType::INFO : logType::ERRO,
            "Log playback %s: %u messages from %s",
            success ? "finished" : "failed", sent_messages, info.name);

        return success ? RESULT_OK : RESULT_ERROR;
    }, "print_log", "Replay file by index: file_index,delay_msg_ms", "logger");

    // Wi-Fi OTA firmware updates (DEBUG state only).
    shell.create_module("ota", "Wi-Fi OTA firmware updates (DEBUG state)");

    shell.add([]() -> uint8_t {
        if (StateMachine::current_state.load(std::memory_order_acquire) !=
            DEBUG) {
            ROBOT::logger.insert_log(
                logType::ERRO, "OTA update requires DEBUG state; enter DEBUG first");
            return RESULT_ERROR;
        }

        if (instance_->usb_storage.is_active() || !instance_->sd_card.is_mounted()) {
            ROBOT::logger.insert_log(
                logType::ERRO, "OTA update unavailable: SD card is not mounted for robot");
            return RESULT_ERROR;
        }

        if (instance_->anyDebugTestActive()) {
            ROBOT::logger.insert_log(
                logType::ERRO, "OTA update blocked: wait for the DEBUG test to finish");
            return RESULT_ERROR;
        }

        if (!instance_->ota.start()) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "Failed to start OTA update: no networks stored in " OTA_WIFI_LIST_FILE);
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_log(
            logType::INFO,
            "OTA update started: scanning for a known network; press any button to cancel");
        return RESULT_OK;
    }, "start", "Join a known Wi-Fi network and accept a firmware upload", "ota");

    shell.add([]() -> uint8_t {
        const char* status = "idle";
        switch (instance_->ota.phase()) {
            case OTAUpdater::Phase::SCANNING:       status = "scanning for a known network"; break;
            case OTAUpdater::Phase::CONNECTING:     status = "connecting"; break;
            case OTAUpdater::Phase::CONNECT_FAILED: status = "connect failed, retrying"; break;
            case OTAUpdater::Phase::SERVING:        status = "serving OTA updates"; break;
            case OTAUpdater::Phase::IDLE:            status = "idle"; break;
        }

        char text[160];
        if (instance_->ota.phase() == OTAUpdater::Phase::SERVING) {
            snprintf(text, sizeof(text),
                     "OTA: %s, ssid=%s ip=%s (or http://%s.local/)",
                     status, instance_->ota.connected_ssid(),
                     instance_->ota.connected_ip(), instance_->ota.hostname());
        } else {
            snprintf(text, sizeof(text), "OTA: %s", status);
        }

        ROBOT::logger.send_log_direct(logType::INFO, text);
        instance_->sendNextShellOutputDirect();
        return RESULT_OK;
    }, "status", "Show the current OTA update sub-mode status", "ota");

    shell.add([]() -> uint8_t {
        // A successful upload sets the new image as the boot partition but
        // deliberately does not reboot on its own (see
        // OTAUpdater::finish_update) — this is the remote trigger for that,
        // reachable over ESP-NOW since OTA already handed the channel back.
        ROBOT::logger.send_log_direct(logType::INFO, "Rebooting...");
        instance_->sendNextShellOutputDirect();

        esp_timer_handle_t reboot_timer;
        const esp_timer_create_args_t timer_args = {
            .callback = [](void*) { esp_restart(); },
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "shell_reboot",
            .skip_unhandled_events = false,
        };
        esp_timer_create(&timer_args, &reboot_timer);
        esp_timer_start_once(reboot_timer, 500000);

        return RESULT_OK;
    }, "reboot", "Restart the robot now (e.g. to boot into a staged OTA update)", "ota");

    shell.add([](std::string ssid, std::string password) -> uint8_t {
        if (!instance_->ota.add_network(ssid.c_str(), password.c_str())) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "Failed to add network: invalid ssid/password, list full or SD unavailable");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(logType::INFO, "Network added: %s",
                                  ssid.c_str());
        return RESULT_OK;
    }, "wifi_add", "Add a Wi-Fi network for OTA: ssid,password", "ota");

    shell.add([](uint16_t index) -> uint8_t {
        if (!instance_->ota.remove_network(index)) {
            ROBOT::logger.insert_log(logType::ERRO, "Invalid network index");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(logType::INFO, "Network %u removed", index);
        return RESULT_OK;
    }, "wifi_remove", "Remove a stored Wi-Fi network by index", "ota");

    shell.add([]() -> uint8_t {
        const uint16_t count = instance_->ota.network_count();
        ROBOT::logger.insert_logf(logType::INFO, "OTA networks: %u", count);

        for (uint16_t index = 0; index < count; ++index) {
            char ssid[OTA_SSID_MAX_LEN];
            if (!instance_->ota.get_network(index, ssid, sizeof(ssid))) continue;
            ROBOT::logger.insert_logf(logType::INFO, "[%u] %s", index, ssid);
        }

        return RESULT_OK;
    }, "wifi_list", "List Wi-Fi networks stored for OTA (SSID only)", "ota");

    // Runtime settings backed by settings.conf on the SD card (see
    // lib/RobotSettings). "set"/"reset"/"reset_all" only change the
    // in-memory copy; "save" persists it, and every change needs a reboot
    // to actually take effect (pins, timers and EKF init are all applied
    // once, at boot).
    shell.create_module("settings", "Runtime robot settings backed by settings.conf");

    shell.add([](std::string module, std::string key) -> uint8_t {
        std::string value;
        if (!instance_->settings.get(module.c_str(), key.c_str(), value)) {
            ROBOT::logger.insert_logf(logType::ERRO, "Unknown setting: %s.%s",
                                      module.c_str(), key.c_str());
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(logType::INFO, "%s.%s=%s", module.c_str(),
                                  key.c_str(), value.c_str());
        return RESULT_OK;
    }, "get", "Read one setting: module,key", "settings");

    shell.add([](std::string module, std::string key, std::string value) -> uint8_t {
        if (!instance_->settings.set(module.c_str(), key.c_str(), value.c_str())) {
            ROBOT::logger.insert_logf(
                logType::ERRO,
                "Failed to set %s.%s: unknown setting or invalid value",
                module.c_str(), key.c_str());
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(
            logType::INFO,
            "%s.%s set in memory; run 'settings save' to persist, reboot to apply",
            module.c_str(), key.c_str());
        return RESULT_OK;
    }, "set", "Change one setting in memory: module,key,value", "settings");

    shell.add([](std::string module) -> uint8_t {
        std::string out;
        instance_->settings.list(module.c_str(), out);

        if (out.empty()) {
            ROBOT::logger.insert_logf(logType::ERRO, "Unknown settings module: %s",
                                      module.c_str());
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_log(logType::INFO, out.c_str());
        return RESULT_OK;
    }, "list", "List every setting in one module", "settings");

    shell.add([]() -> uint8_t {
        std::string out;
        instance_->settings.list_all(out);
        ROBOT::logger.insert_log(logType::INFO, out.c_str());
        return RESULT_OK;
    }, "list_all", "List every setting in every module", "settings");

    shell.add([]() -> uint8_t {
        if (!instance_->settings.save(instance_->sd_card)) {
            ROBOT::logger.insert_log(
                logType::ERRO, "Failed to save settings.conf: SD card not mounted");
            return RESULT_ERROR;
        }

        ROBOT::logger.send_log_direct(logType::INFO, "Settings saved to settings.conf");
        instance_->sendNextShellOutputDirect();
        return RESULT_OK;
    }, "save", "Persist current settings to settings.conf", "settings");

    shell.add([]() -> uint8_t {
        uint16_t skipped = 0;
        if (!instance_->settings.load(instance_->sd_card, &skipped)) {
            ROBOT::logger.insert_log(
                logType::ERRO, "Failed to load settings.conf: SD card not mounted");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(
            logType::INFO,
            "Settings reloaded from settings.conf (%u line(s) ignored); reboot to apply",
            skipped);
        return RESULT_OK;
    }, "load", "Reload settings.conf from SD, discarding unsaved edits", "settings");

    shell.add([](std::string module) -> uint8_t {
        if (!instance_->settings.reset_module(module.c_str())) {
            ROBOT::logger.insert_logf(logType::ERRO, "Unknown settings module: %s",
                                      module.c_str());
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(
            logType::INFO,
            "%s reset to defaults in memory; run 'settings save' to persist",
            module.c_str());
        return RESULT_OK;
    }, "reset", "Reset one module to compiled-in defaults, in memory", "settings");

    shell.add([]() -> uint8_t {
        instance_->settings.reset_all();
        ROBOT::logger.insert_log(
            logType::INFO,
            "All settings reset to defaults in memory; run 'settings save' to persist");
        return RESULT_OK;
    }, "reset_all", "Reset every setting to compiled-in defaults, in memory", "settings");

#ifdef ENABLE_SYSTEM_MONITOR
    // System health: CPU load, memory, uptime (built only when enabled).
    shell.create_module("sysmon", "System health: CPU load, memory, uptime");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_logf(logType::INFO, "Core temperature: %.1f C",
                                  instance_->sysmon.getCoreTemperature());
        return RESULT_OK;
    }, "temp", "Show the SoC core temperature", "sysmon");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_log(logType::INFO, instance_->sysmon.getUptime().c_str());
        return RESULT_OK;
    }, "uptime", "Show system uptime", "sysmon");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_log(logType::INFO, instance_->sysmon.getMemoryStats().c_str());
        return RESULT_OK;
    }, "memory", "Show heap/PSRAM usage stats", "sysmon");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_log(logType::INFO, instance_->sysmon.getTaskStats().c_str());
        return RESULT_OK;
    }, "tasks", "Show per-task CPU load and stack usage", "sysmon");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_log(logType::INFO, instance_->sysmon.getFullReport().c_str());
        return RESULT_OK;
    }, "report", "Show the full system health report on demand", "sysmon");
#endif
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

        message receivedMessage;
        if (xQueueReceive(instance_->receivedDataQueue, &receivedMessage, 0) != pdTRUE)
            continue;
        
        // execute the command
        instance_->shell.run_command_line(receivedMessage.content.text);
    }
}

void ROBOT::runStateMachine(void *param) {
    (void)param; // Suppress unused parameter warning

    // log the task
    logger.insert_log(logType::INFO, "State machine task started");

    while (true) {
        // run state machine
        instance_->machine.run();
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
        if (!instance_->ota.is_active())
            ROBOT::logger.flush_logs();              // send the logger messagens to output
        instance_->resetFlags();                    // reset the flags - buttons, side sensors, pwm...
        instance_->setOutputs();                    // set the output - leds, pwm...
        instance_->checkStateMachine();             // cheg the next state of the state machine
        vTaskDelay(WDOG_TIMEOUT_TK); // delay for wathdog timer and to allow other tasks to run
    }
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
    imu.emplace(cfg.sda_pin, cfg.scl_pin, IMU_I2C_ADDRESS);

#ifdef ENABLE_SYSTEM_MONITOR
    sysmon.begin();
    sysmon.setOutputCallback([](const std::string& data) {
        if (!data.empty()) ROBOT::logger.insert_log(logType::DEBG, data.c_str());
    });
    sysmon.setLoggerCallback([]() { return ROBOT::logger.get_write_pct(); });
#endif

    receivedDataQueue = xQueueCreate(10, sizeof(message));
    if (receivedDataQueue == nullptr) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to create receive queue");
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
    imu_ready_ = imu->begin() > 0;
    if (!imu_ready_) {
        ROBOT::logger.insert_log(logType::WARN,
                                 "IMU (ICM42688) not detected; EKF running on encoders only");
    }

    setTimeLimit();
    startWrappers();
    initEKF();

    ROBOT::logger.insert_log(logType::INFO, "Welcome! the car is starting...");

    initialized = true;
    return true;
}
