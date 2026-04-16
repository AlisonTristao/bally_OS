#include <StaticObjects.h>

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
uint32_t ROBOT::delay_flags = 100; // default delay for flags duration check

// timer handle (moved from ParallelProcessing.h)
esp_timer_handle_t ROBOT::timer_get_handle;

struct InterruptInitResult {
    volatile bool done;
    volatile bool success;
    InterruptInitResult(): done(false), success(false) {}
};

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
    InterruptInitResult* res = (InterruptInitResult*) param;

    // set the buttons interruptions
    /*attachInterrupt(digitalPinToInterrupt(BTN1), Signals_IN::isrBtn0, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN2), Signals_IN::isrBtn1, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN3), Signals_IN::isrBtn2, FALLING);

    // set the side sensors interruptions
    attachInterrupt(digitalPinToInterrupt(LEFT), Signals_IN::isrsideSensor0, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT), Signals_IN::isrsideSensor1, RISING);*/

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
        res->success = ok;
    #else
        res->success = true;
    #endif

    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("Interruptions configured", logType::INFO);
    #endif

    res->done = true;

    // delete this task
    vTaskDelete(NULL);
}

void ROBOT::init() {
    motor_left.init();
    motor_right.init();
}

void ROBOT::logStateMachineError(const char* message) {
    logger.insert_log(message, logType::ERROR);
}

void ROBOT::routine(void *param){
    // timer of state machine actualization
    uint32_t timer_state_machine = millis();

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
        taskYIELD();
    }
}