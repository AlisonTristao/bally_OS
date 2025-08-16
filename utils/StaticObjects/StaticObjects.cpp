#include <StaticObjects.h>

// initialize static objects
ArraySensor StaticObjects::array_sensor(LEN_SENSOR, SIG, C0, C1, C2, C3);
Encoder StaticObjects::encoder_left(ENC_A0, ENC_A1);
Encoder StaticObjects::encoder_right(ENC_B0, ENC_B1);
HBridge StaticObjects::motor_left(AIN1, AIN2, CH0, PWM_A);
HBridge StaticObjects::motor_right(BIN1, BIN2, CH1, PWM_B);
Control StaticObjects::control;
TinyShell StaticObjects::shell;