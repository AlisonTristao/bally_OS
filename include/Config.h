#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>
#include <Flags.h>
#include <Wire.h>

// timer for loop
uint64_t timerLoop = 0;

// esp32 core
#define PRIMARY_CORE    1   // void loop
#define SECONDARY_CORE  0   // parallel processing 

// logger configuration
#define LOG_ALL             // register all logs
#define LOG_CONF            // register configuration logs
#define LOG_OPER            // register operational logs
#define LOG_ERRO            // register error logs
#define LOG_LIVE            // print log messages in real time

// timers
#define SAMPLE_MICROS   500000
#define LOOP_MICROS     500000
#define DLY_LONG        500
#define DLY_SHORT       50

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

void configure_pins(){
    // array of leds
    pinMode(38, OUTPUT);
    pinMode(37, OUTPUT);
    pinMode(36, OUTPUT);
    pinMode(35, OUTPUT);
    pinMode(45, OUTPUT);
    pinMode(46, OUTPUT);

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
    Wire.begin(SDA, SCL);
}

bool init_structure() {
    try {
        // init pins direction, settings, i2c communication...
        configure_pins();                               

        // set the filter time for flags
	    Flags::setFilterTime(DLY_LONG); 

        return true;
    } catch(const std::exception& e) {
        // LOGGER ERROR
        return false;
    }
    return NULL;
}

bool routine(){
    try {
        // check the flags duration    
        Flags::checkFlagsDuration();  

        // make sure the time is equal
        while (micros() - timerLoop <= LOOP_MICROS);    
        timerLoop = micros();
        
        return true;
    } catch(const std::exception& e) {
        // LOGGER ERROR
        return false;
    }
    return NULL;
}

#endif