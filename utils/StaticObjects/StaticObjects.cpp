#include <StaticObjects.h>
#include <Pinout.h>
#include <esp_log.h>
#include <esp_system.h>
#include <cstring>
#include <string>
#include <cstdio>

static const char* TAG = "ROBOT";

// active instance of the robot class
ROBOT* ROBOT::instance_ = nullptr;

uint8_t sensor_pins[LEN_SENSOR] = {D0, D1, D2, D3, D4, D5, D6, D7};

static bool gpio_isr_installed = false;

// Define the buttons and side sensors as Flags_in objects with their respective indices
static FlagsArg btnArgs[] = {
    {&ROBOT::buttons, 0},
    {&ROBOT::buttons, 1},
    {&ROBOT::buttons, 2}
};

static FlagsArg sideArgs[] = {
    {&ROBOT::sideSensors, 0},
    {&ROBOT::sideSensors, 1}
};

Flags_in ROBOT::buttons("Buttons");
Flags_in ROBOT::sideSensors("Side Sensors");
Flags_out ROBOT::leds("LEDs");
Flags_pwm ROBOT::motors("Motors");

Logger ROBOT::logger;
ArraySensor ROBOT::array_sensor(sensor_pins, LEN_SENSOR);
HBridge ROBOT::motor_left(AIN1, AIN2, CH0, PWM_A);
HBridge ROBOT::motor_right(BIN1, BIN2, CH1, PWM_B);
Control ROBOT::control;
TinyShell ROBOT::shell;
StateMachine ROBOT::machine(NONE, NULL, NULL);

void readMacAddress(){
    // logger the mac address
    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
            baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
        char mac_text[64];
        snprintf(mac_text, sizeof(mac_text), "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
            baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
        ROBOT::logger.insert_log(logType::INFO, mac_text);
    } else {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to read MAC address");
    }
}

// sample ISR (IRAM resident)
void IRAM_ATTR ROBOT::sampleISR(void* arg) {
    #if defined(LOG_ALL) || defined(LOG_TELEMETRY)
        // nothing 
    #endif
}

void ROBOT::configure_interruptions(void *param){
    // set the button interruptions
    if (!gpio_isr_installed) {
        esp_err_t isr_err = gpio_install_isr_service(0);
        if (isr_err == ESP_OK || isr_err == ESP_ERR_INVALID_STATE)
            gpio_isr_installed = true;
    }

    gpio_set_intr_type((gpio_num_t)BTN1, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type((gpio_num_t)BTN2, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type((gpio_num_t)BTN3, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type((gpio_num_t)LEFT, GPIO_INTR_POSEDGE);
    gpio_set_intr_type((gpio_num_t)RIGHT, GPIO_INTR_POSEDGE);

    gpio_isr_handler_add((gpio_num_t)BTN1, Flags_in::isr, &btnArgs[0]);
    gpio_isr_handler_add((gpio_num_t)BTN2, Flags_in::isr, &btnArgs[1]);
    gpio_isr_handler_add((gpio_num_t)BTN3, Flags_in::isr, &btnArgs[2]);
    gpio_isr_handler_add((gpio_num_t)LEFT, Flags_in::isr, &sideArgs[0]);
    gpio_isr_handler_add((gpio_num_t)RIGHT, Flags_in::isr, &sideArgs[1]);
    // set the timer interruptions
    #ifdef SAMPLING_ACTIVE
        esp_timer_create_args_t timer_args = {
          .callback = &ROBOT::sampleISR,
          .arg = NULL,
          .name = "timer_get_values"
        };

        // try init the timer interrupt
        bool ok = !(esp_timer_create(&timer_args, &ROBOT::timer_get_handle) != ESP_OK
                    || esp_timer_start_periodic(ROBOT::timer_get_handle, SAMPLE_MICROS) != ESP_OK);
    #endif
    // delete this task
    vTaskDelete(NULL);
}

bool ROBOT::configurePins() {
    // array of leds
    /*pinMode(YELLOW, OUTPUT);
    pinMode(RED, OUTPUT);
    pinMode(BLUE, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(UNK0, OUTPUT);
    pinMode(UNK1, OUTPUT);*/

    gpio_config_t io_conf = {};

    // Outputs
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = (1ULL << AIN1) |
                           (1ULL << AIN2) |
                           (1ULL << BIN1) |
                           (1ULL << BIN2) |
                           (1ULL << PWM_A) |
                           (1ULL << PWM_B);
    gpio_config(&io_conf);

    // Buttons (inputs with pull-up)
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pin_bit_mask = (1ULL << BTN1) | (1ULL << BTN2) | (1ULL << BTN3);
    gpio_config(&io_conf);

    // Side sensors (inputs without pull)
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = (1ULL << LEFT) | (1ULL << RIGHT);
    gpio_config(&io_conf);

    // Encoders
    /*pinMode(ENC_A0, INPUT);
    pinMode(ENC_A1, INPUT);
    pinMode(ENC_B0, INPUT);
    pinMode(ENC_B1, INPUT);

    // Buzzer
    pinMode(BZR, OUTPUT);

    // Multiplex
    pinMode(SIG, INPUT);
    pinMode(C0, OUTPUT);
    pinMode(C1, OUTPUT);
    pinMode(C2, OUTPUT);
    pinMode(C3, OUTPUT);

    // Bat
    pinMode(BAT, INPUT);

    // i2c communication
    bool i2c = Wire.begin(SDA, SCL);
    
    // all pins configured
    return i2c;*/

    return true;
}

bool ROBOT::configureCommunication() {
    // configure WiFi and ESP-NOW
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to init esp-netif");
        return false;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to create event loop");
        return false;
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to init WiFi");
        return false;
    }

    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to set WiFi storage");
        return false;
    }

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to set WiFi mode");
        return false;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to start WiFi");
        return false;
    }

    esp_wifi_disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);

    // initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to initialize ESP-NOW");
        return false;
    }

    // configure the ESP-NOW callbacks
    if (esp_now_register_recv_cb(handleReceiveStatic) != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to register receive callback");
        return false;
    }

    if (esp_now_register_send_cb(handleSendStatic) != ESP_OK) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to register send callback");
        return false;
    }

    // log the MAC address
    readMacAddress();

    // add peer if MAC_ADDR is defined
    #ifdef MAC_ADDR
        uint8_t peer_addr[6] = {MAC_ADDR};
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, peer_addr, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        peerInfo.ifidx = WIFI_IF_STA;
        esp_now_add_peer(&peerInfo);
    #endif

    return true;
}

void ROBOT::setTimeLimit() {
    // set the time limit for the flags, to reset them after a certain time
    buttons.setTimeLimit(DELAY_FLAGS);
    sideSensors.setTimeLimit(DELAY_FLAGS);
    leds.setTimeLimit(DELAY_FLAGS);
    motors.setTimeLimit(DELAY_FLAGS);
}

void ROBOT::resetFlags() {
    // check flags duration
    ROBOT::buttons.checkFlagsDuration();
    ROBOT::sideSensors.checkFlagsDuration();
    ROBOT::leds.checkFlagsDuration();
    ROBOT::motors.checkFlagsDuration();
}

void ROBOT::setOutputs() {
    // define the outputs value based on the flags
    ROBOT::motor_left.applyPWM(ROBOT::motors.getValue(0));
    ROBOT::motor_right.applyPWM(ROBOT::motors.getValue(1));

    // set the leds based on the flags
    // ainda nao existe leds
}

bool ROBOT::init() {
    // avoid initializing more than once
    if (initialized)    
        return true;

    // configure the pins (OUTPUT, INPUT, etc) and the i2c communication
    if (!configurePins())
        return false;

    // configure communication (wifi and esp-now)  
    if (!configureCommunication())
        return false;
        
    // configure motor (init the channel PWM and set the initial state)
    motor_left.init();
    motor_right.init();

    // set time for reset the flags signals
    setTimeLimit();

    // create the queue for the received data from the ESP-NOW
    receveivedDataQueue = xQueueCreate(10, sizeof(message));

    // log message 
	ROBOT::logger.insert_log(logType::INFO, "Welcome! the car is starting...");

    // return true if everything is ok
    initialized = true;
    return true;
}

void ROBOT::staticInsertLog(const char* message) {
    logger.insert_log(logType::ERRO, message);
}

void ROBOT::executeCommandFromQueue() {
    // check if there is a message in the queue, if not, return
    if(uxQueueMessagesWaiting(instance_->receveivedDataQueue) == 0) 
        return;
    message receivedMessage;
    // check if there is a message in the queue
    if (xQueueReceive(receveivedDataQueue, &receivedMessage, 0) == pdTRUE) {
        // convert the message to a string
        std::string command(receivedMessage.content.text);
        // execute the command and log the result
        executeCommand(command.c_str());
    }
}

void ROBOT::executeCommand(const char* command) const {
    // execute the command in the shell and get the result
    std::string result = shell.run_command_line(command);

    // log the command and the result
    logger.insert_log(logType::CMDO, result.c_str());
}

void ROBOT::checkStateMachine() {
    // check the state machine every DELAY_FLAGS milliseconds
    if (millis() - instance_->stateMachineTimer > DELAY_FLAGS) {
        // check the next state of the state machine
        ROBOT::machine.next(ROBOT::buttons.getFlags());
        // update the last state check time
        instance_->stateMachineTimer = millis();  
    }   
}

void ROBOT::routine(void *param){
    // if the robot is not initialized, we cannot run the routine
    while (!instance_->initialized)
        vTaskDelay(100/portTICK_PERIOD_MS);

    // log message  
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log(logType::INFO, "Parallel processing initialized");
    #endif

    // main loop of the parallel processing
    while(true) {	
        // logger print live
        #ifdef LOG_VERBOSE
            ROBOT::logger.flush_logs();
        #endif

        // execute the commands from the queue
        instance_->executeCommandFromQueue();

        // reset the flags if the time limit is reached
        instance_->resetFlags();

        // set the outputs signal based on the flags
        instance_->setOutputs();

        // check the state machine to change the state if the conditions are met
        instance_->checkStateMachine();
            
        // sample delay... (wait for the whatchdog to be ready)
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
}

// Adapter para callback estático de recebimento.
void ROBOT::handleReceiveStatic(const esp_now_recv_info_t* info, const uint8_t* incomingData, int len) {
    (void)info;
    (void)len;
    // verify if the queue is created
    if (instance_->receveivedDataQueue == nullptr) {
        ROBOT::logger.insert_log(logType::ERRO, "Receive callback called but queue is not initialized");
        return;
    }

    // add the buffer to the queue to be processed in the parallel processing
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(instance_->receveivedDataQueue, incomingData, &xHigherPriorityTaskWoken);
}

// Adapter para callback estático de envio.
void ROBOT::handleSendStatic(const uint8_t* mac, esp_now_send_status_t status) {
    // Currently, we do not have a send callback set up, but this is where you would handle it if needed.
}
