#include <BallyRobot.h>
#include <Settings.h>

// ESP-IDF Includes
#include <cstring>
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
    // envia um texto grande para testar o envio de varios pacotes
    // o texto é uma citacao de "Dom Casmurro", de Machado de Assis
    const char* long_text =
        "Uma noite, ao chegar a casa,\n"
        "encontrei um bilhete de minha mãe, dizendo que ela e meu pai\n"
        "haviam saído para jantar, e que eu deveria me comportar.\n"
        "Fiquei sozinho em casa, e a solidão me envolveu como um manto.\n"
        "Sentei-me à janela, olhando para as estrelas,\n"
        "e pensei em tudo o que havia acontecido em minha vida até então.\n"
        "As lembranças de minha infância, de meus pais, de minha escola,\n"
        "de meus amigos, tudo isso passou diante de meus olhos como um filme.\n"
        "E então, percebi que a vida era como um rio, que corria sem parar,\n"
        "levando-nos para lugares desconhecidos, e que nós éramos como folhas,\n"
        "flutuando na correnteza, sem saber onde iríamos parar. Foi uma\n"
        "noite de reflexão profunda, e eu me senti mais maduro, mais consciente\n"
        "de mim mesmo e do mundo ao meu redor. E assim, adormeci, com a cabeça\n"
        "cheia de pensamentos e o coração cheio de emoções, sabendo que a vida\n"
        "continuaria a me surpreender, a me desafiar, e que eu teria que enfrentar\n"
        "tudo isso com coragem e determinação.";

    // When info logs are enabled, push to logger to validate its packetization path.
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log(logType::INFO, long_text);
        return RESULT_OK;
    #endif
}

// ==============================================================================
// HARDWARE CONFIGURATION
// ==============================================================================

void ROBOT::configure_interruptions(void *param){
    (void)param; // Suppress unused parameter warning

    // set the interrupt type for the buttons and side sensors, 
    // and add the corresponding ISR handlers to set the flags when the interrupts are triggered
    gpio_set_intr_type((gpio_num_t)BIT_0, GPIO_INTR_NEGEDGE); // FALLING
    gpio_isr_handler_add((gpio_num_t)BIT_0, [](void* arg) {
        instance_->buttons.setFlag(BIT_0);
    }, nullptr);

    gpio_set_intr_type((gpio_num_t)BIT_1, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add((gpio_num_t)BIT_1, [](void* arg) {
        instance_->buttons.setFlag(BIT_1);
    }, nullptr);

    gpio_set_intr_type((gpio_num_t)BIT_2, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add((gpio_num_t)BIT_2, [](void* arg) {
        instance_->buttons.setFlag(BIT_2);
    }, nullptr);

    gpio_set_intr_type((gpio_num_t)LEFT, GPIO_INTR_POSEDGE); // RISING
    gpio_isr_handler_add((gpio_num_t)LEFT, [](void* arg) {
        instance_->sideSensors.setFlag(BIT_0);
    }, nullptr);

    gpio_set_intr_type((gpio_num_t)RIGHT, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)RIGHT, [](void* arg) {
        instance_->sideSensors.setFlag(BIT_1);
    }, nullptr);

    vTaskDelete(NULL);
}

bool ROBOT::configurePins() {
    const gpio_num_t out_pins[] = {
        (gpio_num_t)LED_RGB_PIN, (gpio_num_t)AIN1, (gpio_num_t)AIN2, 
        (gpio_num_t)BIN1, (gpio_num_t)BIN2, (gpio_num_t)PWM_A, 
        (gpio_num_t)PWM_B//, (gpio_num_t)BZR
    };
    for(auto pin : out_pins) {
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    }

    const gpio_num_t in_pins[] = {
        (gpio_num_t)LEFT, (gpio_num_t)RIGHT, 
        (gpio_num_t)ENC_A0, (gpio_num_t)ENC_A1, 
        (gpio_num_t)ENC_B0, (gpio_num_t)ENC_B1
    };
    for(auto pin : in_pins) {
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_INPUT);
    }

    const gpio_num_t btn_pins[] = {
        (gpio_num_t)BIT_0, (gpio_num_t)BIT_1, (gpio_num_t)BIT_2
    };
    for(auto pin : btn_pins) {
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
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
    motor_left.applyPWM(motors.getValue(MOTOR_LEFT_idx));
    motor_right.applyPWM(motors.getValue(MOTOR_RIGHT_idx));
}

void ROBOT::executeCommandFromQueue() {
    if(uxQueueMessagesWaiting(instance_->receivedDataQueue) == 0) 
        return;
    
    message receivedMessage;
    if (xQueueReceive(receivedDataQueue, &receivedMessage, 0) == pdTRUE) {
        // Substituída a classe String do Arduino pela std::string do C++
        std::string command(receivedMessage.content.text);
        executeCommand(command.c_str());
    }
}

void ROBOT::executeCommand(const char* command) const {
    instance_->shell.run_command_line(command);
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
    float left_speed = encoder_left.getCountDiff() * ENCODER_PPR * WHEEL_RADIUS * 2.0f * PI * FREQ_EKF;
    float right_speed = encoder_right.getCountDiff() * ENCODER_PPR * WHEEL_RADIUS * 2.0f * PI * FREQ_EKF;
    return (left_speed + right_speed) / 2.0f;
}

float ROBOT::getOmegaFromEncoders() {
    float left_speed = encoder_left.getCountDiff() * ENCODER_PPR * WHEEL_RADIUS * 2.0f * PI * FREQ_EKF;
    float right_speed = encoder_right.getCountDiff() * ENCODER_PPR * WHEEL_RADIUS * 2.0f * PI * FREQ_EKF;
    return (right_speed - left_speed) / EKF_WHEEL_BASE;
}

void ROBOT::runEKF() {
    float u[2] = {static_cast<float>(instance_->motors.getValue(MOTOR_RIGHT_idx)),
                  static_cast<float>(instance_->motors.getValue(MOTOR_LEFT_idx))};
    float z[5] = {instance_->getSpeedFromEncoders(), 
                  instance_->getOmegaFromEncoders(), 
                  0,
                  0,
                  0};
                  //instance_->imu.gyrZ() * kDegToRad,
                  //instance_->imu.accX() * 9.81f, 
                  //instance_->imu.accY() * 9.81f};
    instance_->EKF.predict(u);
    instance_->EKF.update(z, u);
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
}

// ==============================================================================
// MAIN TASKS & INITIALIZATION
// ==============================================================================

void ROBOT::routine(void *param){
    (void)param; 

    while (!instance_->initialized)
        vTaskDelay(pdMS_TO_TICKS(100));

    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log(logType::INFO, "Parallel processing initialized");
    #endif

    while(true) {   
        ROBOT::logger.flush_logs();
        instance_->executeCommandFromQueue();
        instance_->resetFlags();
        instance_->setOutputs();
        instance_->checkStateMachine();
        vTaskDelay(pdMS_TO_TICKS(WDOG_TIMEOUT_MS));
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

    receivedDataQueue = xQueueCreate(10, sizeof(message));
    if (receivedDataQueue == nullptr) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to create receive queue");
        return false;
    }

    if (!configureCommunication())
        return false;
        
    motor_left.init();
    motor_right.init();

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