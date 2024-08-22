#include <Arduino.h>
#include <Config.h>
#include <Interrupt.h>

// tempo de filtro das interrupcoes
uint32_t Flags::filterTime = DLY_LONG;

void setup() {
	Serial.begin(921600);

	// init pins direction, settings, i2c communication...
	init_structure();

	// init interruptions in parallel processing
	init_interruptions();


}

void loop() {
	// printa o nucleo que esta rodando
	Flags::checkFlagsDuration();
	Flags::printFlags();
	delay(100);
}