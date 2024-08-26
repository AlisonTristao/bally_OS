#ifndef FLAGS_H
#define FLAGS_H

#include <Arduino.h>

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

/********************/
/*  Class Of Flags  */
/********************/  

template<int N>
struct FlagsByteTemplate {
    union {
        uint8_t allFlags = 0;   // one byte to store all flags
        struct { 
            bool flag0 : 1;               
            bool flag1 : 1;
            bool flag2 : 1;
            bool flag3 : 1;
            bool flag4 : 1;
            bool flag5 : 1;
            bool flag6 : 1;
            bool flag7 : 1;
        };
    };
    uint32_t time[N] = {};  // timer for each flag if necessary
};

// signal to in
using FlagsByte = FlagsByteTemplate<8>;

// signal to out
using LedsByte = FlagsByteTemplate<0>; 

class Signals_IN {
    public:
        // handle the interrupt service routines
        static void IRAM_ATTR isrBtn0();
        static void IRAM_ATTR isrBtn1();
        static void IRAM_ATTR isrBtn2();
        static void IRAM_ATTR isrBtn3();
        static void IRAM_ATTR isrBtn4();
        static void IRAM_ATTR isrBtn5();
        static void IRAM_ATTR isrBtn6();
        static void IRAM_ATTR isrBtn7();

        // handle the interrupt service routines
        static void IRAM_ATTR isrsideSensor0();
        static void IRAM_ATTR isrsideSensor1();
        static void IRAM_ATTR isrsideSensor2();
        static void IRAM_ATTR isrsideSensor3();
        static void IRAM_ATTR isrsideSensor4();
        static void IRAM_ATTR isrsideSensor5();
        static void IRAM_ATTR isrsideSensor6();
        static void IRAM_ATTR isrsideSensor7();

        // check if any flag has been up for longer than a specified duration
        static void checkFlagsDuration();

        // return number of flags of the buttons
        static uint8_t getButtons();

        // return number of flags of the side sensors
        static uint8_t getSideSensors();

        // set the filter time
        static void setFilterTime(uint32_t time);
    private:
        // buttons flags
        static FlagsByte buttons;

        // side sensors flags
        static FlagsByte sideSensors;

        // timer for filtering the interruptions
        static uint32_t filterTime;
};

class Signals_OUT {
    public:
        // get and set the leds
        static void setLeds(uint8_t byte);
        static LedsByte getLeds();

        // get and set the buzzer
        static void setBuzzer(uint16_t freq);
        static uint16_t getBuzzer();
    private:
        // leds
        static LedsByte leds;

        // buzzer
        static uint16_t buzzer;
};

#endif // FLAGS_H
