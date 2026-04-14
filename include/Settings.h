#ifndef SETTINGS_H
#define SETTINGS_H

// logger configuration
// -> LOG_ALL             // register all logs
// -> LOG_INFO            // register configuration logs
// -> LOG_TELEMETRY       // register operational logs
// -> LOG_ERROR           // register error logs
// -> LOG_DEBUG           // register debug logs
// -> LOG_VERBOSE         // print log messages in real time
#define LOG_ALL
#define LOG_VERBOSE

// native libraries
#include <Arduino.h>

// wrapper
#include <Wrapper.h>

// header
#include <Pinout.h>
#include <EspNow.h>

// static libraries
#include <Flags.h>
#include <Logger.h>
#include <StateMachine.h>

// sampling activation
// -> SAMPLING_ACTIVE
//#define SAMPLING_ACTIVE

// esp32 core 
#define PRIMARY_CORE    1   // void loop
#define SECONDARY_CORE  0   // parallel processing 

// timers
#define SAMPLE_MICROS   100000
#define DELAY_FLAGS     250
#define CONTROL_TIME_MS 1

/*void setLeds(uint8_t bit_arr){
    digitalWrite(YELLOW, bit_arr & (1 << BIT_3));
    digitalWrite(RED, bit_arr & (1 << BIT_2));
    digitalWrite(BLUE, bit_arr & (1 << BIT_1));
    digitalWrite(GREEN, bit_arr & (1 << BIT_0));
    //digitalWrite(UNK0, bit_arr & (1 << BIT_4));
    //digitalWrite(UNK1, bit_arr & (1 << BIT_5));
}*/

bool configure_pins(){
    // array of leds
    /*pinMode(YELLOW, OUTPUT);
    pinMode(RED, OUTPUT);
    pinMode(BLUE, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(UNK0, OUTPUT);
    pinMode(UNK1, OUTPUT);*/

    pinMode(LED_RGB_PIN, OUTPUT);

    // H bridge
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWM_A, OUTPUT);
    pinMode(PWM_B, OUTPUT);

    // Encoders
    /*pinMode(ENC_A0, INPUT);
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
        ROBOT::logger.insert_log("Pins configured", logType::INFO);
    #endif
    
    // all pins configured
    return i2c;*/
    return true;
}

bool init_structure() {
    // init pins direction, settings, i2c communication...
    if (!configure_pins())
        return false;

    // init wifi
    if (!configure_wifi())
        return false;

    // init shell
    if (!start_shell_wrappers())
        return false;

    // all ok
    return true;
}

#endif