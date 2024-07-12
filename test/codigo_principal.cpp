#include <Arduino.h>
#include <pins.h>
#include "stateflow.h"
#include "arraySensor.h"
#include "interrupt.h"

#include "HBridge.h"
#include <ESP32Encoder.h>

states state = POWER_ON;
arraySensor sensor(8, SIG, C0, C1, C2, C3, WHITE);

HBridge motorE(AIN1, AIN2, CH1, PWM_A);
HBridge motorD(BIN1, BIN2, CH2, PWM_B);

void setup()
{
	Serial.begin(921600);

	ESP32Encoder encoderD;
	ESP32Encoder encoderE;

	// init pins
	init_pins();
	motorD.init();
	motorE.init();

	encoderD.attachHalfQuad(ENC_A0, ENC_A1);
	encoderE.attachHalfQuad(ENC_B0, ENC_B1);

	encoderD.clearCount();
	encoderE.clearCount();

	// Set omterruptions
	set_all_interruptions();
}

void loop()
{
	// debug
	// Serial.print(sensor.debub());
	// Serial.print(" ");
	// Serial.println(sensor.read_line());

	switch (state)
	{

	case POWER_ON:
		state_1();
		break;

	case CALIBRATION:
		state_2();
		break;

	case RUNNING:
		state_3();
		break;

	default:
		break;
	}
}
