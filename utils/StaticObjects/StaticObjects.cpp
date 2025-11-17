#include <StaticObjects.h>

// initialize static objects
uint8_t sensor_pins[LEN_SENSOR] = {D0, D1, D2, D3, D4, D5, D6, D7};
ArraySensor StaticObjects::array_sensor(sensor_pins, LEN_SENSOR);
//Encoder StaticObjects::encoder_left(ENC_A0, ENC_A1);
//Encoder StaticObjects::encoder_right(ENC_B0, ENC_B1);
HBridge StaticObjects::motor_left(AIN1, AIN2, CH0, PWM_A);
HBridge StaticObjects::motor_right(BIN1, BIN2, CH1, PWM_B);
Control StaticObjects::control;
TinyShell StaticObjects::shell;