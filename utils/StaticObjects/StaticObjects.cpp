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

        // keep periodic debug prints without changing state here
        if(millis() - timer_state_machine > ROBOT::delay_flags)
            timer_state_machine = millis();

        // sample delay... (wait for the whatchdog to be ready) 
        delay(1);
    }
}