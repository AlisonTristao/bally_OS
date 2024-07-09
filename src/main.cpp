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
HBridge motorE(AIN1, AIN2, CH0, PWM_A);
HBridge motorD(BIN1, BIN2, CH1, PWM_B);

// encoders
ESP32Encoder encoderD;
ESP32Encoder encoderE;

// control
Control controlW; // VELOCIDADE ANGULAR
Control controlT; // VELOCIDADE LINEAR

Control controlE; // RODA ESQUERDA
Control controlD; // RODA DIREITA

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
		/* musica e led pisca s*/
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

		//get speed of motors 
		double speedE = encoderE.getSpeed();
		double speedD = encoderD.getSpeed();

		double erroE = 2000 - speedE;
		double erroD = 2000 - speedD;

		// calcular as ações de controle
		double uTE = controlE.simplePID(0.1, 1, 0.01, erroE, SAMPLE_MS/1000.0);
		double uTD = controlD.simplePID(0.1, 1, 0.01, erroD, SAMPLE_MS/1000.0);
		//int32_t uW = controlW.simplePID(KpW, KiW, KdW, 10, 0);
		
		motorE.applyPWM((int32_t)uTE);
		motorD.applyPWM((int32_t)uTD);

		Serial.print("loop");
		Serial.println(i);
		Serial.print(" >pwm_E:");
		Serial.print((int)uTE);
		Serial.print(" >pwm_D:");
		Serial.print((int)uTD);
		Serial.print(" >e_E:");
		Serial.print((int)erroE);
		Serial.print(" >e_D:");
		Serial.print((int)erroD);
		Serial.print(" >v_E:");
		Serial.print((int)speedE);
		Serial.print(" >v_D:");
		Serial.println((int)speedD);


		while (millis() - timer < SAMPLE_MS);
		break;
	}
	default:
		break;
	}
}