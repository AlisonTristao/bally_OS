#include <Arduino.h>
#include <pins.h>
#include <Encoder.h>

// obj encoder
Encoder encoderA(ENC_A0, ENC_A1);
Encoder encoderB(ENC_B0, ENC_B1);

void setup() {
	Serial.begin(921600);

	// init pins
	init_pins();

	Serial.println("limite de contadores: " + String(PCNT_UNIT_MAX));
	Serial.println("loop in core: " + String(xPortGetCoreID()));

	// init encoder
	encoderA.init(250, 0);
	encoderB.init(250, 1);

	delay(1000);
}

void loop() {
	Serial.print("A: ");
	//Serial.print(encoderA.getCount());
	Serial.print(" B: ");
	//Serial.println(encoderB.getCount());
	delay(100);
}