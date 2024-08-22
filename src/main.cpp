#include <Arduino.h>
#include <Config.h>
#include <Interrupt.h>

void setup() {
	Serial.begin(921600);

	// init pins direction, settings, i2c communication...
	init_structure();

	// init interruptions in parallel processing
	init_interruptions();

	

}

void loop() {	
	Flags::printFlags();
	routine();
} 