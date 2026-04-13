#include <StaticObjects.h>

// initialize static objects
uint8_t sensor_pins[LEN_SENSOR] = {D0, D1, D2, D3, D4, D5, D6, D7};
ArraySensor ROBOT::array_sensor_(sensor_pins, LEN_SENSOR);
HBridge ROBOT::motor_left_(AIN1, AIN2, CH0, PWM_A);
HBridge ROBOT::motor_right_(BIN1, BIN2, CH1, PWM_B);
Control ROBOT::control_;
TinyShell ROBOT::shell_;

void ROBOT::init() {
	motor_left_.init();
	motor_right_.init();
}

void ROBOT::applyStateOutputs(uint8_t state) {
	switch (state) {
		case RUN:
			// RUN controls motors actively inside its own state function.
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
	motor_left_.applyPWM(0);
	motor_right_.applyPWM(0);
}

void ROBOT::brakeMotors() {
	motor_left_.brake();
	motor_right_.brake();
}

void ROBOT::applyMotorPWM(int32_t pwmLeft, int32_t pwmRight) {
	motor_left_.applyPWM(pwmLeft);
	motor_right_.applyPWM(pwmRight);
}

double ROBOT::readLine() {
	return array_sensor_.read_line();
}

bool ROBOT::calibrateSensors(uint8_t samples, uint8_t delayMs) {
	return array_sensor_.calibrate(samples, delayMs);
}

void ROBOT::saveCalibration() {
	array_sensor_.saveCalibration();
}

String ROBOT::calibrateStatus() {
	return array_sensor_.calibrate_status();
}

String ROBOT::debugSensors() {
	return array_sensor_.debug();
}

double ROBOT::controlSimplePD(double kp, double kd, double error, double sample) {
	return control_.simplePD(kp, kd, error, sample);
}

void ROBOT::log(const String& msg, logType type) {
	Logger::insert_log(msg, type);
}

void ROBOT::logStateMachineError(const char* message) {
	if (message == nullptr) {
		return;
	}

	Logger::insert_log(message, logType::ERROR);
}

void ROBOT::sendLogger(logType type) {
	Logger::send_logger(type);
}

void ROBOT::sendLoggerLive() {
	Logger::send_logger_live();
}

void ROBOT::setFlagFilterTime(uint32_t timeMs) {
	Signals_IN::setFilterTime(timeMs);
}

void ROBOT::checkFlagsDuration() {
	Signals_IN::checkFlagsDuration();
}

uint8_t ROBOT::getButtons() {
	return Signals_IN::getButtons();
}

uint8_t ROBOT::getSideSensors() {
	return Signals_IN::getSideSensors();
}

TinyShell& ROBOT::shell() {
	return shell_;
}

String ROBOT::runLineCommand(const String& cmd) {
	return shell_.run_line_command(cmd.c_str()).c_str();
}