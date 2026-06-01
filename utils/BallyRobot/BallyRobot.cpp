#include <BallyRobot.h>
#include <Arduino.h>
#include <Pinout.h>

// active instance of the robot class
ROBOT* ROBOT::instance_ = nullptr;

const uint8_t sensor_pins[LEN_SENSOR] = {D0, D1, D2, D3, D4, D5, D6, D7};

Flags_in ROBOT::buttons("Buttons");
Flags_in ROBOT::sideSensors("Side Sensors");
Flags_out ROBOT::leds("LEDs");
Flags_pwm ROBOT::motors("Motors");

Logger ROBOT::logger;
RGBLed ROBOT::rgb_led;
ArraySensor<LEN_SENSOR> ROBOT::array_sensor(sensor_pins);
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
        ROBOT::logger.insert_log(logType::INFO, ("MAC Address: " + String(baseMac[0], HEX) + ":" + String(baseMac[1], HEX) + ":" + String(baseMac[2], HEX) + ":" + String(baseMac[3], HEX) + ":" + String(baseMac[4], HEX) + ":" + String(baseMac[5], HEX)).c_str());
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
    attachInterruptArg(digitalPinToInterrupt(BIT_0), Flags_in::isr, &btnArgs[0], FALLING);
    attachInterruptArg(digitalPinToInterrupt(BIT_1), Flags_in::isr, &btnArgs[1], FALLING);
    attachInterruptArg(digitalPinToInterrupt(BIT_2), Flags_in::isr, &btnArgs[2], FALLING);
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

bool ROBOT::configurePins() {
    // array of leds
    /*pinMode(YELLOW, OUTPUT);
    pinMode(RED, OUTPUT);
    pinMode(BLUE, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(UNK0, OUTPUT);
    pinMode(UNK1, OUTPUT);*/

    pinMode(LED_RGB_PIN, OUTPUT);

    // H bridge
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWM_A, OUTPUT);
    pinMode(PWM_B, OUTPUT);

    // Buttons
    pinMode(BIT_0, INPUT_PULLUP);
    pinMode(BIT_1, INPUT_PULLUP);
    pinMode(BIT_2, INPUT_PULLUP);

    // Side sensors
    pinMode(LEFT, INPUT);
    pinMode(RIGHT, INPUT);

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
    if (!WiFi.mode(WIFI_STA)) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to configure WiFi mode");
        return false;
    }

    // disconnect is best-effort; avoid failing if already disconnected
    WiFi.disconnect();
    delay(50);

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

void ROBOT::setTimeLimit() {
    // set the time limit for the flags, to reset them after a certain time
    buttons.setTimeLimit(DELAY_FLAGS);
    sideSensors.setTimeLimit(DELAY_FLAGS);
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
    ROBOT::motor_left.applyPWM(ROBOT::motors.getValue(MOTOR_LEFT_idx));
    ROBOT::motor_right.applyPWM(ROBOT::motors.getValue(MOTOR_RIGHT_idx));

    // set the leds based on the flags
    // ainda nao existe leds
}

void ROBOT::executeCommandFromQueue() {
    // check if there is a message in the queue, if not, return
    if(uxQueueMessagesWaiting(instance_->receveivedDataQueue) == 0) 
        return;
    message receivedMessage;
    // check if there is a message in the queue
    if (xQueueReceive(receveivedDataQueue, &receivedMessage, 0) == pdTRUE) {
        // convert the message to a string
        String command(receivedMessage.content.text);
        // execute the command and log the result
        executeCommand(command.c_str());
    }
}

void ROBOT::executeCommand(const char* command) const {
    // execute the command in the shell and get the result
    uint8_t result = shell.run_command_line(command);
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

// Adapter para callback estático de recebimento.
void ROBOT::handleReceiveStatic(const uint8_t* mac, const uint8_t* incomingData, int len) {
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
    // calculate the speed of the robot based on the encoders values
    float left_speed = encoder_left.getCountDiff() * ENCODER_PPR * WHEEL_RADIUS * 2.0f * PI * FREQ_EKF; // distance traveled by the left wheel in meters
    float right_speed = encoder_right.getCountDiff() * ENCODER_PPR * WHEEL_RADIUS * 2.0f * PI * FREQ_EKF; // distance traveled by the right wheel in meters
    float speed = (left_speed + right_speed) / 2.0f; // average speed of the two wheels
    return speed;
}

float ROBOT::getOmegaFromEncoders() {
    // calculate the angular velocity of the robot based on the encoders values
    float left_speed = encoder_left.getCountDiff() * ENCODER_PPR * WHEEL_RADIUS * 2.0f * PI * FREQ_EKF; // distance traveled by the left wheel in meters
    float right_speed = encoder_right.getCountDiff() * ENCODER_PPR * WHEEL_RADIUS * 2.0f * PI * FREQ_EKF; // distance traveled by the right wheel in meters
    float omega = (right_speed - left_speed) / EKF_WHEEL_BASE; // difference in speed divided by wheel base
    return omega;
}

void ROBOT::runEKF() {
    // get the input control signals (motor commands) and the measurements (encoders and IMU)
    float u[2] = {instance_->motors.getValue(MOTOR_RIGHT_idx), 
                  instance_->motors.getValue(MOTOR_LEFT_idx)};
    float z[5] = {instance_->getSpeedFromEncoders(), 
                  instance_->getOmegaFromEncoders(), 
                  instance_->imu.gyrZ() * (M_PI / 180.0f), 
                  instance_->imu.accX() * 9.81f, 
                  instance_->imu.accY() * 9.81f};
    // run EKF prediction and update
    instance_->EKF.predict(u);
    instance_->EKF.update(z, u);
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

bool ROBOT::init() {
    // avoid initializing more than once
    if (initialized)    
        return true;

    // start serial communication for debuggind when the espnow is not working
	// using USB CDC communication, the baud rate is not relevant
	Serial.begin(BAUDRATE); 

    // configure the pins (OUTPUT, INPUT, etc) and the i2c communication
    if (!configurePins())
        return false;

    // initialize the logger
	logger.begin();

    // create the queue for the received data from the ESP-NOW
    receveivedDataQueue = xQueueCreate(10, sizeof(message));
    if (receveivedDataQueue == nullptr) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to create receive queue");
        return false;
    }

    // configure WiFi and ESP-NOW before any send attempts
    if (!configureCommunication())
        return false;
        
    // configure motor (init the channel PWM and set the initial state)
    motor_left.init();
    motor_right.init();

    // configure the encoders
    if (!encoder_left.init()) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to initialize left encoder");
        return false;
    }
    if (!encoder_right.init()) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to initialize right encoder");
        return false;
    }

    // configure the imu
    if (!imu.begin()) {
        ROBOT::logger.insert_log(logType::ERRO, "Failed to initialize IMU");
        return false;
    }

    // set time for reset the flags signals
    setTimeLimit();

    // init the EKF
    initEKF();

    // log message 
	ROBOT::logger.insert_log(logType::INFO, "Welcome! the car is starting...");

    // return true if everything is ok
    initialized = true;
    return true;
}
