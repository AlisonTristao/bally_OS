#include <StaticObjects.h>
#include <Arduino.h>
#include <Pinout.h>

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

// timer handle (moved from ParallelProcessing.h)
esp_timer_handle_t ROBOT::timer_get_handle;

// sample ISR (IRAM resident)
void IRAM_ATTR ROBOT::sampleISR(void* arg) {
    #if defined(LOG_ALL) || defined(LOG_TELEMETRY)
        if(!(ROBOT::machine.current_state == RUN)) return;
        /*ROBOT::logger.insert_log(
                            String(ROBOT::encoder_left.getCount())    + ";" +
                            String(ROBOT::encoder_right.getCount()),
                            logType::TELEMETRY);*/
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

void ROBOT::init() {
    // configure motor 
    motor_left.init();
    motor_right.init();

    // configure WiFi and ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // log the MAC address
    readMacAddress();

    // initialize ESP-NOW
    esp_now_init();

    // configure the ESP-NOW callbacks
    esp_now_register_recv_cb(handleReceiveStatic);
    esp_now_register_send_cb(handleSendStatic);

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
}

void ROBOT::logStateMachineError(const char* message) {
    logger.insert_log(message, logType::ERROR);
}

void ROBOT::routine(void *param){
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

        // check flags duration
        ROBOT::buttons.checkFlagsDuration();
        ROBOT::sideSensors.checkFlagsDuration();
        ROBOT::leds.checkFlagsDuration();
        ROBOT::motors.checkFlagsDuration();

        // unlock the CPU (wait for the whatchdog to be ready) 
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

// Adapter para callback estático de recebimento.
void ROBOT::handleReceiveStatic(const uint8_t* mac, const uint8_t* incomingData, int len) {

    // Copy the incoming data into a message struct
    message incoming = {};
    memcpy(&incoming, incomingData, sizeof(message));
    incoming.msg[sizeof(incoming.msg) - 1] = '\0';

    // run the command in the shell and log it as CMD
    string result = ROBOT::shell.run_command_line(incoming.msg);

    // log the command result
    #if defined(LOG_ALL) || defined(LOG_CMD)
        ROBOT::logger.insert_log(String(result.c_str()), logType::CMD);
    #endif
}

// Adapter para callback estático de envio.
void ROBOT::handleSendStatic(const uint8_t* mac, esp_now_send_status_t status) {
    // Currently, we do not have a send callback set up, but this is where you would handle it if needed.
    (void)mac;
    (void)status;
}
