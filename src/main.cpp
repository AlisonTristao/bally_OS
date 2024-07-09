#include <Arduino.h>
#include <pins.h>
#include "stateflow.h"
#include "arraySensor.h"
#include "leds.h"
#include "interrupt.h"
#include "Control.h"

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

// control
Control controlW;
Control controlT;

// control gains
#define KpT		1
#define KiT		1
#define KdT		1
#define	KpW		1
#define KiW		1
#define KdW		1

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
	sensor.set_init_arr(4);	// mux in the middle 
}

uint32_t i = 0;
void loop()
{
	switch (state)
	{
	case POWER_ON:{
		/* musica e led pisca */
		power_func();
		break;
	}

	case CALIBRATION:{
		/* calibrate sensors */
		if (!calibrated) 	calibrated = calibrate(&sensor);
		else	 			ledUp((uint8_t) map(sensor.read_line(), 1000, 8000, 0, 5));
		break;
	}

	case RUNNING:{
		uint32_t timer = millis();

		/* get speed of motors */
		double speedE = encoderE.getSpeed();
		double speedD = encoderD.getSpeed();

		double erro = 500 - ((speedE+speedD)/2);

		// calcular as ações de controle
		double uT = controlT.simplePID(0.01, 0.01, 0.01, erro);
		//int32_t uW = controlW.simplePID(KpW, KiW, KdW, 10, 0);
		
		motorD.applyPWM((int32_t)uT);
		motorE.applyPWM((int32_t)uT);

		//Serial.print("loop");
		//Serial.println(i);
		Serial.print(">pwm:");
		Serial.print(uT);
		Serial.print(" >e:");
		Serial.print(erro);
		Serial.print(" >vm:");
		Serial.println((int)(speedE+speedD)/2);


		while (millis() - timer < SAMPLE_MS);
		break;
	}
	default:
		break;
	}
}