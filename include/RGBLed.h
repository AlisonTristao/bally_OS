#ifndef RGBLED_H
#define RGBLED_H

#include <Arduino.h>

class RGBLed {
public:
    void set(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
        uint8_t rr = ((uint16_t)r * brightness) / 255;
        uint8_t gg = ((uint16_t)g * brightness) / 255;
        uint8_t bb = ((uint16_t)b * brightness) / 255;

        // Em muitas placas ESP32-S3 o LED onboard usa ordem GRB
        neopixelWrite(LED_RGB_PIN, rr, gg, bb);
    }

    void off() {
        neopixelWrite(LED_RGB_PIN, 0, 0, 0);
    }
};

#endif