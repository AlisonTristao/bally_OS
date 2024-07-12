#ifndef PINS_H
#define PINS_H
#include <Arduino.h>

// filter
#define WC_RPM 50

// timers
#define SAMPLE_MS 2
#define DLY_LONG 500
#define DLY_SHORT 100

// bzer tone
#define TONE_HIGH 3000
#define TONE_LOW 500

// channels
#define CH0 0
#define CH1 1
#define CH2 2

// array of leds
#define LED0 38
#define LED1 37
#define LED2 36
#define LED3 35
#define LED4 45
#define LED5 46

// H bridge
// controla a direção do motor A, exemplo se AIN1 = HIGH e AIN2 = LOW o motor A gira no sentido horário.
#define AIN1 11
#define AIN2 10
// controla a direção do motor B, exemplo se AIN1 = HIGH e AIN2 = LOW o motor A gira no sentido horário.
#define BIN1 12
#define BIN2 13
// controla a velocidade do motor A
#define PWM_A 9
// controla a velocidade do motor B
#define PWM_B 14

// Encoders
#define A0 21
#define A1 47
#define B0 20
#define B1 19

// Buttons
#define BTN1 1
#define BTN2 2

// Buzzer
#define BZR 6

// Multiplex
#define SIG 18
#define C0 39
#define C1 40
#define C2 41
#define C3 42

// Side sensors
#define LEFT  8 
#define RIGHT 17

// i2c devices
#define SDA 4
#define SDL 5

// Tensao dividers
#define BAT 7

void init_pins()
{
    // array of leds
    pinMode(LED0, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);
    pinMode(LED4, OUTPUT);
    pinMode(LED5, OUTPUT);

    // H bridge
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWM_A, OUTPUT);
    pinMode(PWM_B, OUTPUT);

    // Encoders
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(B0, INPUT);
    pinMode(B1, INPUT);

    // Buttons
    pinMode(BTN1, INPUT_PULLUP);
    pinMode(BTN2, INPUT_PULLUP);

    // Buzzer
    pinMode(BZR, OUTPUT);
    ledcSetup(CH2, 2000, 8);
    ledcAttachPin(BZR, CH2);

    // Multiplex
    pinMode(SIG, INPUT);
    pinMode(C0, OUTPUT);
    pinMode(C1, OUTPUT);
    pinMode(C2, OUTPUT);
    pinMode(C3, OUTPUT);

    // Side sensors
    pinMode(LEFT, INPUT);
    pinMode(RIGHT, INPUT);

    // Bat
    pinMode(BAT, INPUT);
}

#endif