#include <Arduino.h>
#include <pins.h>

void setup() {
	Serial.begin(921600);

	// init pins
	init_pins();

}

void loop() {
	digitalWrite(YELLOW, HIGH);
	delay(50);
	digitalWrite(YELLOW, LOW);
	digitalWrite(RED, HIGH);
	delay(50);
	digitalWrite(RED, LOW);
	digitalWrite(BLUE, HIGH);
	delay(50);
	digitalWrite(BLUE, LOW);
	digitalWrite(GREEN, HIGH);
	delay(50);
	digitalWrite(GREEN, LOW);
	delay(50);
	tone(BZR, TONE_HIGH);
	delay(50);
	noTone(BZR);

	// faz o inverso 
	digitalWrite(GREEN, HIGH);
	delay(50);
	digitalWrite(GREEN, LOW);
	digitalWrite(BLUE, HIGH);
	delay(50);
	digitalWrite(BLUE, LOW);
	digitalWrite(RED, HIGH);
	delay(50);
	digitalWrite(RED, LOW);
	digitalWrite(YELLOW, HIGH);
	delay(50);
	digitalWrite(YELLOW, LOW);
	delay(50);
	tone(BZR, TONE_HIGH);
	delay(50);
	noTone(BZR);
}