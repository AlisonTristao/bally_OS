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

ESP32Encoder encoderD;
ESP32Encoder encoderE;

void setup()
{
	Serial.begin(921600);

	encoderD.attachHalfQuad(A0, A1);
	encoderE.attachHalfQuad(B0, B1);

	// init pins
	init_pins();
	motorD.init();
	motorE.init();
	motorD.applyPWM(2000);
	motorE.applyPWM(-1000);

	encoderD.clearCount();
	encoderE.clearCount();
}

void loop()
{

	Serial.print(String((double)encoderD.getSpeed()));
	Serial.println(" " + String((double)encoderE.getSpeed()));
	delay(100);
}
