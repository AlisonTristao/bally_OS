#ifndef FLAGS_H
#define FLAGS_H

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

// native libraries
#include <Arduino.h>

#define BIT_0 0
#define BIT_1 1
#define BIT_2 2
#define BIT_3 3
#define BIT_4 4
#define BIT_5 5
#define BIT_6 6
#define BIT_7 7

/********************/
/*  Class Of Flags  */
/********************/  

struct FlagsByte {
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
    uint32_t time[8] = {};  // timer for each flag if necessary
};

class Signals_IN {
    public:
        /*
            @brief set the flag of the button to up
        */
        static void IRAM_ATTR   isrBtn0(), 
                                isrBtn1(), 
                                isrBtn2(),
                                isrBtn3(),
                                isrBtn4(),
                                isrBtn5(),
                                isrBtn6(),
                                isrBtn7();

        /*
            @brief set the flag of the side sensor to up
        */
        static void IRAM_ATTR   isrsideSensor0(),
                                isrsideSensor1(),
                                isrsideSensor2(),
                                isrsideSensor3(),
                                isrsideSensor4(),
                                isrsideSensor5(),
                                isrsideSensor6(),
                                isrsideSensor7();

        static void IRAM_ATTR   setLed0on(),
                                setLed1on(),
                                setLed2on(),
                                setLed3on(),
                                setLed4on(),
                                setLed5on(),
                                setLed6on(),
                                setLed7on();

        /*
            @brief check the flags duration
        
            if the flag is up for more than the filter time, the flag is set to down
        */
        static void checkFlagsDuration();

        /*
            @brief get all boolean flags using a byte

            @return the buttons flags
        */
        static uint8_t getButtons();

        /*
            @brief get all boolean flags using a byte

            @return the side sensors flags
        */
        static uint8_t getSideSensors();

        /*
            @brief get all boolean flags using a byte

            @return the leds flags
        */
        static uint8_t getLeds();

        /*
            @brief set the filter time for the flags to be down

            @param time: time to filter the interruptions
        */
        static void setFilterTime(uint32_t time);
    private:
        /*
            @brief struct to store the flags of the buttons and make boolean union with byte
        */
        static FlagsByte buttons;

        /*
            @brief struct to store the flags of the side sensors and make boolean union with byte
        */
        static FlagsByte sideSensors;

        /*
            @brief struct to store the flags of the leds and make boolean union with byte
        */
        static FlagsByte leds;

        /*
            @brief time to filter the interruptions
        */
        static uint32_t filterTime;
};

#endif // FLAGS_H
