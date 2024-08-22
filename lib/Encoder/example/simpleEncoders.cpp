#include <Arduino.h>
#include <Config.h>
#include <Encoder.h>

// obj encoder
Encoder encoderA(ENC_A0, ENC_A1);
Encoder encoderB(ENC_B0, ENC_B1);

void setup() {
	Serial.begin(921600);

	// init pins
	init_structure();

	// max enconder in the esp32
	Serial.println("limite de contadores: " + String(PCNT_UNIT_MAX));

	// ISR to second core
	Encoder::core_to_run_ISR = 0;

	// init encoder
	encoderA.init();
	encoderB.init();
	delay(1000);

}

void loop() {
	Serial.print("A: ");
	Serial.print(encoderA.getCount());
	Serial.print(" B: ");
	Serial.println(encoderB.getCount());
	delay(100);
}