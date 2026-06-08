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

bool ROBOT::configurePins() {
    const gpio_num_t out_pins[] = {
        (gpio_num_t)LED_RGB_PIN, /*(gpio_num_t)AIN1, (gpio_num_t)AIN2,
        (gpio_num_t)BIN1, (gpio_num_t)BIN2, (gpio_num_t)PWM_A, 
        (gpio_num_t)PWM_B//, (gpio_num_t)BZR*/
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
    //motor_left.applyPWM(motors.getValue(MOTOR_LEFT_idx));
    //motor_right.applyPWM(motors.getValue(MOTOR_RIGHT_idx));
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

void ROBOT::initInterruptions(void *param){
    (void)param; // Suppress unused parameter warning

    // set the interrupt type for the buttons and side sensors, 
    // and add the corresponding ISR handlers to set the flags when the interrupts are triggered
    gpio_set_intr_type((gpio_num_t)BIT_0, GPIO_INTR_NEGEDGE); // FALLING
    gpio_isr_handler_add((gpio_num_t)BIT_0, [](void* arg) IRAM_ATTR {
        instance_->buttons.setFlag(BIT_0);
    }, nullptr);

    gpio_set_intr_type((gpio_num_t)BIT_1, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add((gpio_num_t)BIT_1, [](void* arg) IRAM_ATTR {
        instance_->buttons.setFlag(BIT_1);
    }, nullptr);

    gpio_set_intr_type((gpio_num_t)BIT_2, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add((gpio_num_t)BIT_2, [](void* arg) IRAM_ATTR {
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