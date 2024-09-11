#ifndef SETTINGS_H
#define SETTINGS_H

// native libraries
#include <Arduino.h>
#include <Wire.h>

// static libraries
#include <Flags.h>
#include <Logger.h>
#include <StateMachine.h>

// sampling activation
// -> SAMPLING_ACTIVE
#define SAMPLING_ACTIVE

// logger configuration
// -> LOG_ALL             // register all logs
// -> LOG_INFO            // register configuration logs
// -> LOG_TELEMETRY       // register operational logs
// -> LOG_ERRO            // register error logs
// -> LOG_DEBUG           // register debug logs
// -> LOG_VERBOSE         // print log messages in real time
#define LOG_TELEMETRY

// esp32 core 
#define PRIMARY_CORE    1   // void loop
#define SECONDARY_CORE  0   // parallel processing 

// array sensor configuration
#define LEN_SENSOR      8
#define INIT_MUX        4   
#define SAMPLES         30      // calibration samples
#define DELAY_SAMPLE    100     // delay between samples

// timers
#define SAMPLE_MICROS   1000000
#define DELAY_FLAGS     250

// channels
#define CH0             0
#define CH1             1
#define CH2             2
#define CH3             3

// array of leds
#define YELLOW          38
#define RED             37
#define BLUE            36
#define GREEN           35
#define UNK0            45
#define UNK1            46

// H bridge
#define AIN1            11
#define AIN2            10
#define BIN1            12
#define BIN2            13
#define PWM_A           9
#define PWM_B           14

// Encoders 
#define ENC_A0          21
#define ENC_A1          47
#define ENC_B0          20
#define ENC_B1          19

// Buttons
#define BTN1            1
#define BTN2            2
#define BTN3            0

// Buzzer
#define BZR             6

// Multiplex
#define SIG             18
#define C0              39
#define C1              40
#define C2              41
#define C3              42

// Side sensors
#define LEFT            8 
#define RIGHT           17

// i2c devices
#define SDA             4
#define SCL             5

// Tensao dividers
#define BAT             7

void setLeds(uint8_t bit_arr){
    digitalWrite(YELLOW, bit_arr & (1 << BIT_3));
    digitalWrite(RED, bit_arr & (1 << BIT_2));
    digitalWrite(BLUE, bit_arr & (1 << BIT_1));
    digitalWrite(GREEN, bit_arr & (1 << BIT_0));
    //digitalWrite(UNK0, bit_arr & (1 << BIT_4));
    //digitalWrite(UNK1, bit_arr & (1 << BIT_5));
}

bool configure_pins(){
    // array of leds
    pinMode(YELLOW, OUTPUT);
    pinMode(RED, OUTPUT);
    pinMode(BLUE, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(UNK0, OUTPUT);
    pinMode(UNK1, OUTPUT);

    // H bridge
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWM_A, OUTPUT);
    pinMode(PWM_B, OUTPUT);

    // Encoders
    pinMode(ENC_A0, INPUT);
    pinMode(ENC_A1, INPUT);
    pinMode(ENC_B0, INPUT);
    pinMode(ENC_B1, INPUT);

    // Buttons
    pinMode(BTN1, INPUT_PULLUP);
    pinMode(BTN2, INPUT_PULLUP);

    // Buzzer
    pinMode(BZR, OUTPUT);

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

    // i2c communication
    bool i2c = Wire.begin(SDA, SCL);
    
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::IN_LOG("Pins configured", logType::INFO);
    #endif
    
    // all pins configured
    return i2c;
}

bool init_structure() {
    // init pins direction, settings, i2c communication...
    return configure_pins();                               
}

#endif