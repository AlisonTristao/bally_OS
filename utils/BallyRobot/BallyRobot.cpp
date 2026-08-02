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

bool ROBOT::configurePins()
{
    const auto configureOutput =
        [](gpio_num_t pin, uint32_t initialLevel) -> bool
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
    };

    const auto configureInput =
        [](gpio_num_t pin, gpio_pull_mode_t pullMode) -> bool
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
    };

    // --------------------------------------------------------
    // Saídas que devem iniciar desligadas/em nível baixo
    // --------------------------------------------------------
    const gpio_num_t out_pins[] = {
        LED0,
        LED1,
        LED2,
        LED3, 

        AIN1,
        AIN2,
        BIN1,
        BIN2,

        BZR,

        S0,
        S1,
        S2
    };

    for (gpio_num_t pin : out_pins) {
        if (!configureOutput(pin, 0)) {
            return false;
        }
    }

    // O cartão SD deve iniciar desselecionado.
    if (!configureOutput(CS, 1)) {
        return false;
    }

    // --------------------------------------------------------
    // Entradas digitais
    // --------------------------------------------------------
    const gpio_num_t in_pins[] = {
        LEFT,
        RIGHT,

        ENC_A0,
        ENC_A1,
        ENC_B0,
        ENC_B1
    };

    for (gpio_num_t pin : in_pins) {
        if (!configureInput(pin, GPIO_FLOATING)) {
            return false;
        }
    }

    // --------------------------------------------------------
    // Botões
    // --------------------------------------------------------
    const gpio_num_t btn_pins[] = {
        BTN1,
        BTN2,
        BTN0
    };

    for (gpio_num_t pin : btn_pins) {
        if (!configureInput(pin, GPIO_PULLUP_ONLY)) {
            return false;
        }
    }

    // --------------------------------------------------------
    // Entradas analógicas
    // --------------------------------------------------------
    const gpio_num_t analog_pins[] = {
        SIG,
        CURRENT_A,
        CURRENT_B
    };

    for (gpio_num_t pin : analog_pins) {
        if (!configureInput(pin, GPIO_FLOATING)) {
            return false;
        }
    }

    return true;
}

bool ROBOT::configureCommunication() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK || esp_wifi_start() != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to configure WiFi mode");
        return false;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);

    // initialize ESP-NOW
    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to initialize ESP-NOW");
        return false;
    }

    // configure the ESP-NOW callbacks
    err = esp_now_register_recv_cb(handleReceiveStatic);
    if (err != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to register receive callback");
        return false;
    }

    err = esp_now_register_send_cb(handleSendStatic);
    if (err != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to register send callback");
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
            return false;
        }
    #else
        #warning "MAC_ADDR not defined; ESP-NOW peer not added"
    #endif

    return true;
}

// ==============================================================================
// CONTROL & LOGIC
// ==============================================================================

void ROBOT::setTimeLimit() {
    buttons.setTimeLimit(DELAY_FLAGS);
    sideSensors.setTimeLimit(DELAY_FLAGS);
}

void ROBOT::resetFlags() {
    buttons.checkFlagsDuration();
    sideSensors.checkFlagsDuration();
    leds.checkFlagsDuration();
    motors.checkFlagsDuration();
}

void ROBOT::setOutputs() {
    //motor_left.applyPWM(motors.getValue(MOTOR_LEFT_idx));
    //motor_right.applyPWM(motors.getValue(MOTOR_RIGHT_idx));
    // turn on the leds according to the BITS of the arr_stats variable
    uint8_t arr_stats = leds.getFlags();
    gpio_set_level(LED0, (arr_stats & (1 << LED0_idx)));
    gpio_set_level(LED1, (arr_stats & (1 << LED1_idx)));
    gpio_set_level(LED2, (arr_stats & (1 << LED2_idx)));
    gpio_set_level(LED3, (arr_stats & (1 << LED3_idx)));
}

bool ROBOT::startArraySensorTest(uint32_t samples, uint32_t interval_ms) {
    if (samples == 0 ||
        StateMachine::current_state.load(std::memory_order_acquire) != DEBUG ||
        usb_storage.is_active()) {
        return false;
    }

    array_sensor_test_interval_ms.store(interval_ms,
                                        std::memory_order_relaxed);
    array_sensor_test_next_ms.store(
        static_cast<uint32_t>(esp_timer_get_time() / 1000ULL),
        std::memory_order_relaxed);
    array_sensor_test_remaining.store(samples, std::memory_order_release);
    return true;
}

void ROBOT::processDebug() {
    // DEBUG is a safe test state: keep both motor commands at zero on every
    // pass, while the normal outer task delay continues yielding the CPU.
    motors.setValue(MOTOR_LEFT_idx, 0, DELAY_FLAGS);
    motors.setValue(MOTOR_RIGHT_idx, 0, DELAY_FLAGS);

    usb_storage.process();
    if (usb_storage.is_active()) return;

    const uint32_t remaining =
        array_sensor_test_remaining.load(std::memory_order_acquire);
    if (remaining == 0) return;

    const uint32_t now_ms = static_cast<uint32_t>(
        esp_timer_get_time() / 1000ULL);
    const uint32_t next_ms =
        array_sensor_test_next_ms.load(std::memory_order_relaxed);

    // Signed subtraction keeps the comparison valid across millis overflow.
    if (static_cast<int32_t>(now_ms - next_ms) < 0) return;

    logger.insert_log(logType::INFO, array_sensor.debug().c_str());
    array_sensor_test_remaining.store(remaining - 1,
                                      std::memory_order_release);
    array_sensor_test_next_ms.store(
        now_ms + array_sensor_test_interval_ms.load(std::memory_order_relaxed),
        std::memory_order_relaxed);
}

void ROBOT::cancelDebugTests() {
    array_sensor_test_remaining.store(0, std::memory_order_release);
}

void ROBOT::sendNextShellOutputDirect() {
    direct_next_shell_output.store(true, std::memory_order_release);
}

bool ROBOT::consumeDirectShellOutputRequest() {
    return direct_next_shell_output.exchange(false,
                                             std::memory_order_acq_rel);
}

void ROBOT::checkStateMachine() {
    if ((uint32_t)(esp_timer_get_time() / 1000ULL) - instance_->stateMachineTimer > DELAY_FLAGS) {
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
    (void)len;

    if (instance_->receivedDataQueue == nullptr) {
        ROBOT::logger.insert_log(logType::ERRO, "Receive callback called but queue is not initialized");
        return;
    }

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
    float x0[3] = {0.0f, 0.0f, 0.0f};
    float P0[3][3] = {
        {INITIAL_P, 0.0f, 0.0f},
        {0.0f, INITIAL_P, 0.0f},
        {0.0f, 0.0f, INITIAL_P}
    };
    float Q[3][3] = {
        {V_NOISE, 0.0f,  0.0f},  
        {0.0f,  W_NOISE, 0.0f},   
        {0.0f,  0.0f,  B_NOISE} 
    };
    float R[5][5] = {
        {ENC_NOISE, 0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, ENC_NOISE, 0.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, GYRO_NOISE, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f, ACCEL_NOISE, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f, ACCEL_NOISE}
    };

    EKF.init(x0, P0, Q, R);
}

float ROBOT::getSpeedFromEncoders() {
    float left_speed = encoder_left.getCountDiff()/ENCODER_PPR * WHEEL_RADIUS * 2.0f * PI * FREQ_EKF;
    float right_speed = encoder_right.getCountDiff()/ENCODER_PPR * WHEEL_RADIUS * 2.0f * PI * FREQ_EKF;
    return (left_speed + right_speed) / 2.0f;
}

float ROBOT::getOmegaFromEncoders() {
    float left_speed = encoder_left.getCountDiff()/ENCODER_PPR * WHEEL_RADIUS * 2.0f * PI * FREQ_EKF;
    float right_speed = encoder_right.getCountDiff()/ENCODER_PPR * WHEEL_RADIUS * 2.0f * PI * FREQ_EKF;
    return (right_speed - left_speed) / EKF_WHEEL_BASE;
}

void ROBOT::sampleEKF(void *param) {
    // save the pwm values to the control input vector for the EKF
    instance_->control_input[0] = static_cast<float>(instance_->motors.getValue(MOTOR_RIGHT_idx));
    instance_->control_input[1] = static_cast<float>(instance_->motors.getValue(MOTOR_LEFT_idx));

    instance_->measurement[0] = instance_->getSpeedFromEncoders();
    instance_->measurement[1] = instance_->getOmegaFromEncoders();
    instance_->measurement[2] = 0;
    instance_->measurement[3] = 0;
    instance_->measurement[4] = 0;
    //instance_->imu.gyrZ() * kDegToRad,
    //instance_->imu.accX() * 9.81f, 
    //instance_->imu.accY() * 9.81f};

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
        instance_->EKF.predict(instance_->control_input);
        instance_->EKF.update(instance_->control_input, instance_->measurement);
    }
}

bool ROBOT::updateDateTime(uint16_t year, uint8_t month, uint8_t day,
                           uint8_t hour, uint8_t minute, uint8_t second) {
    if (year < 2020 || month < 1 || month > 12 || day < 1 || day > 31 ||
        hour > 23 || minute > 59 || second > 59) {
        return false;
    }

    // Interpret the command values as local Brazilian time (UTC-3).
    setenv("TZ", ROBOT_TIMEZONE, 1);
    tzset();

    struct tm requested_time{};
    requested_time.tm_year = year - 1900;
    requested_time.tm_mon = month - 1;
    requested_time.tm_mday = day;
    requested_time.tm_hour = hour;
    requested_time.tm_min = minute;
    requested_time.tm_sec = second;
    requested_time.tm_isdst = -1;

    const time_t timestamp = mktime(&requested_time);
    if (timestamp == static_cast<time_t>(-1)) return false;

    // mktime normalizes invalid dates, such as 31 February. Compare the result
    // to reject those values instead of silently changing the requested date.
    struct tm verified_time{};
    localtime_r(&timestamp, &verified_time);
    if (verified_time.tm_year != year - 1900 ||
        verified_time.tm_mon != month - 1 ||
        verified_time.tm_mday != day ||
        verified_time.tm_hour != hour ||
        verified_time.tm_min != minute ||
        verified_time.tm_sec != second) {
        return false;
    }

    const struct timeval system_time{
        .tv_sec = timestamp,
        .tv_usec = 0,
    };

    if (settimeofday(&system_time, nullptr) != 0) return false;

    clock_synchronized = true;
    return true;
}

bool ROBOT::makeLogFilename(char* filename, size_t capacity) {
    if (!clock_synchronized || filename == nullptr || capacity == 0) return false;

    const time_t timestamp = time(nullptr);
    struct tm local_time{};
    if (localtime_r(&timestamp, &local_time) == nullptr) return false;

    int written = snprintf(
        filename, capacity, "log_%04d-%02d-%02d_%02d-%02d-%02d.blog",
        local_time.tm_year + 1900, local_time.tm_mon + 1, local_time.tm_mday,
        local_time.tm_hour, local_time.tm_min, local_time.tm_sec);

    if (written <= 0 || static_cast<size_t>(written) >= capacity) return false;
    if (!sd_card.file_exists(filename)) return true;

    // Do not overwrite a log when two new flushes happen in the same second.
    for (uint8_t suffix = 1; suffix < 100; ++suffix) {
        written = snprintf(
            filename, capacity,
            "log_%04d-%02d-%02d_%02d-%02d-%02d_%02u.blog",
            local_time.tm_year + 1900, local_time.tm_mon + 1,
            local_time.tm_mday, local_time.tm_hour, local_time.tm_min,
            local_time.tm_sec, suffix);

        if (written > 0 && static_cast<size_t>(written) < capacity &&
            !sd_card.file_exists(filename)) {
            return true;
        }
    }

    return false;
}

bool ROBOT::findLatestLogFile(char* filename, size_t capacity) {
    if (filename == nullptr || capacity == 0) return false;
    filename[0] = '\0';

    const uint16_t file_count = sd_card.get_file_count();
    for (uint16_t index = 0; index < file_count; ++index) {
        SDFileInfo info{};
        if (!sd_card.get_file_info(index, info)) continue;

        const size_t name_length = strlen(info.name);
        if (name_length < 10 || strncmp(info.name, "log_", 4) != 0 ||
            strcmp(info.name + name_length - 5, ".blog") != 0) {
            continue;
        }

        // ISO date/time in the filename makes lexical order chronological.
        if (filename[0] == '\0' || strcmp(info.name, filename) > 0) {
            const int written = snprintf(filename, capacity, "%s", info.name);
            if (written <= 0 || static_cast<size_t>(written) >= capacity) {
                filename[0] = '\0';
                return false;
            }
        }
    }

    return filename[0] != '\0';
}

bool ROBOT::flushLoggerToSD(bool append) {
    if (!sd_card.is_mounted()) return false;

    char filename[SDFileInfo::MAX_NAME_LENGTH];

    if (append) {
        if (last_log_file[0] != '\0' && sd_card.file_exists(last_log_file)) {
            snprintf(filename, sizeof(filename), "%s", last_log_file);
        } else if (!findLatestLogFile(filename, sizeof(filename))) {
            return false;
        }
    } else if (!makeLogFilename(filename, sizeof(filename))) {
        return false;
    }

    if (!sd_card.open_write_stream(filename, append)) return false;

    const bool stored = logger.flush_logs_to(
        [](const uint8_t* data, size_t length, void* context) -> bool {
            auto* card = static_cast<SDCard*>(context);
            return card->write_stream(data, length);
        },
        &sd_card);

    const bool closed = sd_card.close_stream();

    if (!append) {
        snprintf(last_log_file, sizeof(last_log_file), "%s", filename);
    } else if (last_log_file[0] == '\0') {
        snprintf(last_log_file, sizeof(last_log_file), "%s", filename);
    }

    return stored && closed;
}

void ROBOT::startWrappers() {
    // commands for testing and debugging
    shell.create_module("robot", "Module for robot control commands");
    shell.add(testPacket, "test_packet", "Send a long test packet to evaluate multi-packet handling", "robot");

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

    shell.add([](uint8_t led_idx, uint8_t pwm_value, uint32_t time) -> uint8_t {
        // set the PWM value for the motor with the given index
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

    // DEBUG commands schedule tests; no command blocks in a delay loop.
    shell.create_module("debug", "Safe non-blocking robot tests");

    shell.add([](uint32_t samples, uint32_t interval_ms) -> uint8_t {
        if (!instance_->startArraySensorTest(samples, interval_ms)) {
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

        if (instance_->array_sensor_test_remaining.load(
                std::memory_order_acquire) != 0) {
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
    }, "turnonstorage", "Expose the SD card through native USB MSC", "debug");

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
    }, "storage_status", "Show current SD card owner", "debug");

    // SD card, retained PSRAM logs and robot clock commands.
    shell.create_module("storage", "SD card and retained log management");

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
    }, "psram_usage", "Show retained Logger PSRAM usage", "storage");

    shell.add([](uint16_t year, uint8_t month, uint8_t day,
                 uint8_t hour, uint8_t minute, uint8_t second) -> uint8_t {
        if (!instance_->updateDateTime(
                year, month, day, hour, minute, second)) {
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
       "storage");

    shell.add([]() -> uint8_t {
        if (!instance_->flushLoggerToSD(false)) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "Failed to create a new SD log; synchronize date/time first");
            return RESULT_ERROR;
        }

        char response[SDFileInfo::MAX_NAME_LENGTH + 32];
        snprintf(response, sizeof(response), "PSRAM saved to %s",
                 instance_->last_log_file);
        ROBOT::logger.send_log_direct(logType::INFO, response);
        instance_->sendNextShellOutputDirect();
        return RESULT_OK;
    }, "flush_new", "Save retained PSRAM logs into a new dated file", "storage");

    shell.add([]() -> uint8_t {
        if (!instance_->flushLoggerToSD(true)) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "Failed to append PSRAM logs to the latest SD log");
            return RESULT_ERROR;
        }

        char response[SDFileInfo::MAX_NAME_LENGTH + 36];
        snprintf(response, sizeof(response), "PSRAM appended to %s",
                 instance_->last_log_file);
        ROBOT::logger.send_log_direct(logType::INFO, response);
        instance_->sendNextShellOutputDirect();
        return RESULT_OK;
    }, "flush_append", "Append retained PSRAM logs to the latest log file",
       "storage");

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
    }, "print_log", "Replay file by index: file_index,delay_msg_ms", "storage");
}

// ==============================================================================
// MAIN TASKS & INITIALIZATION
// ==============================================================================

void ROBOT::initInterruptions(void *param){
    (void)param; // Suppress unused parameter warning

    // set the interrupt type for the buttons and side sensors, 
    // and add the corresponding ISR handlers to set the flags when the interrupts are triggered
    gpio_set_intr_type((gpio_num_t)BTN0, GPIO_INTR_NEGEDGE); // FALLING
    gpio_isr_handler_add((gpio_num_t)BTN0, [](void* arg) IRAM_ATTR {
        instance_->buttons.setFlag(BIT_0);
    }, nullptr);

    gpio_set_intr_type((gpio_num_t)BTN1, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add((gpio_num_t)BTN1, [](void* arg) IRAM_ATTR {
        instance_->buttons.setFlag(BIT_1);
    }, nullptr);

    gpio_set_intr_type((gpio_num_t)BTN2, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add((gpio_num_t)BTN2, [](void* arg) IRAM_ATTR {
        instance_->buttons.setFlag(BIT_2);
    }, nullptr);

    gpio_set_intr_type((gpio_num_t)LEFT, GPIO_INTR_POSEDGE); // RISING
    gpio_isr_handler_add((gpio_num_t)LEFT, [](void* arg) IRAM_ATTR {
        instance_->sideSensors.setFlag(BIT_0);
    }, nullptr);

    gpio_set_intr_type((gpio_num_t)RIGHT, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)RIGHT, [](void* arg) IRAM_ATTR {
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
    esp_timer_start_periodic(timer, SAMPLE_MICROS); 

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
        ROBOT::logger.flush_logs();                 // send the logger messagens to output
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

    // configure the pins and i2c
    if (!configurePins())
        return false;

    logger.begin();
    shell.begin();

    // Initialize the card and give FAT ownership to the USB storage manager.
    // It starts mounted for the robot and only exposes it on a DEBUG command.
    if (!sd_card.begin()) {
        logger.insert_log(logType::ERRO, "Failed to initialize SD card");
    } else if (!usb_storage.begin(sd_card)) {
        logger.insert_log(logType::ERRO,
                          "Failed to mount SD card through USB storage manager");
    }

    receivedDataQueue = xQueueCreate(10, sizeof(message));
    if (receivedDataQueue == nullptr) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to create receive queue");
        return false;
    }

    if (!configureCommunication())
        return false;
        
    //motor_left.init();
    //motor_right.init();

    if (!encoder_left.init()) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to initialize left encoder");
        return false;
    }
    if (!encoder_right.init()) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to initialize right encoder");
        return false;
    }

    //if (!imu.begin()) {
    //    ROBOT::logger.insert_log(logType::ERRO, "Failed to initialize IMU");
    //    return false;
    //}

    setTimeLimit();
    startWrappers();
    initEKF();

    ROBOT::logger.insert_log(logType::INFO, "Welcome! the car is starting...");

    initialized = true;
    return true;
}
