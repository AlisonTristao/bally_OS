#include <StaticObjects.h>

uint8_t sensor_pins[LEN_SENSOR] = {D0, D1, D2, D3, D4, D5, D6, D7};

Signals_IN ROBOT::flags;
Logger ROBOT::logger;
RGBLed ROBOT::rgb_led;
ArraySensor ROBOT::array_sensor(sensor_pins, LEN_SENSOR);
HBridge ROBOT::motor_left(AIN1, AIN2, CH0, PWM_A);
HBridge ROBOT::motor_right(BIN1, BIN2, CH1, PWM_B);
Control ROBOT::control;
TinyShell ROBOT::shell;
StateMachine ROBOT::machine;

void ROBOT::init() {
    motor_left.init();
    motor_right.init();
}

void ROBOT::applyStateOutputs(uint8_t state) {
    switch (state) {
        case RUN:
            break;
        case FINISH:
        case ERROR:
            brakeMotors();
            break;
        default:
            stopMotors();
            break;
    }
}

void ROBOT::stopMotors() {
    motor_left.applyPWM(0);
    motor_right.applyPWM(0);
}

void ROBOT::brakeMotors() {
    motor_left.brake();
    motor_right.brake();
}

void ROBOT::logStateMachineError(const char* message) {
    if (message != nullptr) {
        logger.insert_log(message, logType::ERROR);
    }
}
