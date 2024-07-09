#include <Arduino.h>
#include <pins.h>
#include "stateflow.h"
#include "arraySensor.h"
#include "leds.h"
#include "interrupt.h"

#include "HBridge.h"
#include <ESP32Encoder.h>

// init satate machine
states state = POWER_ON;
bool calibrated = false; 		// flag to check if the sensor is calibrated

// sensor array
arraySensor sensor(8, SIG, C0, C1, C2, C3, WHITE);

// motors
HBridge motorE(AIN1, AIN2, CH1, PWM_A);
HBridge motorD(BIN1, BIN2, CH2, PWM_B);

// encoders
ESP32Encoder encoderD;
ESP32Encoder encoderE;

void setup(){
	Serial.begin(921600);

	// init pins
	init_pins();

	// init interruptions
	set_all_interruptions();	

	// init motors
	motorD.init();
	motorE.init();

	// init encoders
	encoderD.attachHalfQuad(A0, A1);
	encoderE.attachHalfQuad(B0, B1);

	// init sensor
	sensor.init();
	sensor.set_init_arr(4);				// mux in the middle 
}

void loop()
{
	switch (state)
	{
	case POWER_ON:
		/* musica e led pisca */
		power_func();
		break;
	
	case CALIBRATION:
		/* calibrate sensors */
		if (!calibrated) 	calibrated = calibrate(&sensor);
		if (calibrated) 	ledUp((uint8_t) map(sensor.read_line(), 1000, 8000, 0, 5));
		break;

	case RUNNING:
		running_func();		
		break;

	default:
		break;
	}
}