#include <StaticObjects.h>
#include <Arduino.h>
#include <Pinout.h>

// active instance of the robot class
ROBOT* ROBOT::instance_ = nullptr;

uint8_t sensor_pins[LEN_SENSOR] = {D0, D1, D2, D3, D4, D5, D6, D7};

Flags_in ROBOT::buttons("Buttons");
Flags_in ROBOT::sideSensors("Side Sensors");
Flags_out ROBOT::leds("LEDs");
Flags_pwm ROBOT::motors("Motors");

Logger ROBOT::logger;
RGBLed ROBOT::rgb_led;
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
    Serial.print("MAC Address: ");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    if (ret == ESP_OK) {
        ROBOT::logger.insert_log("This MAC Address: " +
                        String(baseMac[0], HEX) + ":" +
                        String(baseMac[1], HEX) + ":" +
                        String(baseMac[2], HEX) + ":" +
                        String(baseMac[3], HEX) + ":" +
                        String(baseMac[4], HEX) + ":" +
                        String(baseMac[5], HEX), logType::INFO);
    } else {
        ROBOT::logger.insert_log("Failed to read MAC address", logType::ERROR);
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
    attachInterruptArg(digitalPinToInterrupt(BTN1), Flags_in::isr, &btnArgs[0], FALLING);
    attachInterruptArg(digitalPinToInterrupt(BTN2), Flags_in::isr, &btnArgs[1], FALLING);
    attachInterruptArg(digitalPinToInterrupt(BTN3), Flags_in::isr, &btnArgs[2], FALLING);
    attachInterruptArg(digitalPinToInterrupt(LEFT), Flags_in::isr, &sideArgs[0], RISING);
    attachInterruptArg(digitalPinToInterrupt(RIGHT), Flags_in::isr, &sideArgs[1], RISING);
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

bool ROBOT::init() {
    // avoid initializing more than once
    if (initialized)    
        return true;

    // configure motor 
    motor_left.init();
    motor_right.init();

    // configure WiFi and ESP-NOW
    if (!WiFi.mode(WIFI_STA) || !WiFi.disconnect()) {
        ROBOT::logger.insert_log("Failed to configure WiFi", logType::ERROR);
        return false;
    }

    // initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        ROBOT::logger.insert_log("Failed to initialize ESP-NOW", logType::ERROR);
        return false;
    }

    // create the queue for the received data
    receveivedDataQueue = xQueueCreate(10, sizeof(message));

    // configure the ESP-NOW callbacks
    if (esp_now_register_recv_cb(handleReceiveStatic) != ESP_OK) {
        ROBOT::logger.insert_log("Failed to register receive callback", logType::ERROR);
        return false;
    }

    if (esp_now_register_send_cb(handleSendStatic) != ESP_OK) {
        ROBOT::logger.insert_log("Failed to register send callback", logType::ERROR);
        return false;
    }

    // add peer if MAC_ADDR is defined
    #ifdef MAC_ADDR
        uint8_t peer_addr[6] = {MAC_ADDR};
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, peer_addr, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    #endif

    // init log register
	ROBOT::logger.insert_log("Welcome! the car is starting...", logType::INFO);

    // log the MAC address
    readMacAddress();

    // set the time limit for the flags 
    ROBOT::buttons.setTimeLimit(DELAY_FLAGS);
    ROBOT::sideSensors.setTimeLimit(DELAY_FLAGS);
    ROBOT::leds.setTimeLimit(DELAY_FLAGS);
    ROBOT::motors.setTimeLimit(DELAY_FLAGS);

    // return true if everything is ok
    initialized = true;
    return true;
}

void ROBOT::staticInsertLog(const char* message) {
    logger.insert_log(message, logType::ERROR);
}

void ROBOT::executeCommandFromQueue() {
    // check if there is a message in the queue, if not, return
    if(uxQueueMessagesWaiting(instance_->receveivedDataQueue) == 0) 
        return;
    message receivedMessage;
    // check if there is a message in the queue
    if (xQueueReceive(receveivedDataQueue, &receivedMessage, 0) == pdTRUE) {
        // convert the message to a string
        String command(receivedMessage.msg);
        // execute the command and log the result
        executeCommand(command.c_str());
    }
}

void ROBOT::executeCommand(const char* command) const {
    // execute the command in the shell and get the result
    std::string result = shell.run_command_line(command);

    // log the command and the result
    logger.insert_log(result.c_str(), logType::CMD);
}

void ROBOT::resetFlags() {
    // check flags duration
    ROBOT::buttons.checkFlagsDuration();
    ROBOT::sideSensors.checkFlagsDuration();
    ROBOT::leds.checkFlagsDuration();
    ROBOT::motors.checkFlagsDuration();
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
        ROBOT::logger.insert_log("Parallel processing initialized", logType::INFO);
    #endif

    // main loop of the parallel processing
    while(true) {	
        // logger print live
        #ifdef LOG_VERBOSE
            ROBOT::logger.send_logger_live();
        #endif

        // execute the commands from the queue
        instance_->executeCommandFromQueue();

        // reset the flags if the time limit is reached
        instance_->resetFlags();

        // check the state machine to change the state if the conditions are met
        instance_->checkStateMachine();
            
        // sample delay... (wait for the whatchdog to be ready)
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
}

// Adapter para callback estático de recebimento.
void ROBOT::handleReceiveStatic(const uint8_t* mac, const uint8_t* incomingData, int len) {
    // verify if the queue is created
    if (instance_->receveivedDataQueue == nullptr) {
        ROBOT::logger.insert_log("Receive callback called but queue is not initialized", logType::ERROR);
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
