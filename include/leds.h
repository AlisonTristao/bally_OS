#ifndef LEDS_H
#define LEDS_H
#include <Arduino.h>
#include "pins.h"

// function to set the leds based on the value code
void setLeds(byte value, uint8_t len, uint8_t ledPins[]) {
    for (int i = 0; i < len; i++) {
        // define the state of the led based on the bit of the value code
        digitalWrite(ledPins[i], (value >> i) & 0x01); 
    }
}

// default function to set the leds
void setLeds(byte value){
    uint8_t arr[6] = {LED0, LED1, LED2, LED3, LED4, LED5};
    setLeds(value, 6, arr);
}

void ledUp(uint8_t value){
    digitalWrite(LED0, value == 0);
    digitalWrite(LED1, value == 1);
    digitalWrite(LED2, value == 2);
    digitalWrite(LED3, value == 3);
    digitalWrite(LED4, value == 4);
    digitalWrite(LED5, value == 5);
}

#endif