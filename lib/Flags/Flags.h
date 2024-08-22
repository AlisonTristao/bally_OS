#ifndef FLAGS_H
#define FLAGS_H

#include <Arduino.h>

// struct to store 8 flags in a single byte
struct FlagsByte {
    union {
        // one byte to store all flags
        uint8_t allFlags = 0;   
        // individual flags 
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
    // time of each flag
    uint32_t time[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

class Flags {
public:
    // handle the interrupt service routines
    static void IRAM_ATTR isrFlag0();
    static void IRAM_ATTR isrFlag1();
    static void IRAM_ATTR isrFlag2();
    static void IRAM_ATTR isrFlag3();
    static void IRAM_ATTR isrFlag4();
    static void IRAM_ATTR isrFlag5();
    static void IRAM_ATTR isrFlag6();
    static void IRAM_ATTR isrFlag7();

    // check if any flag has been up for longer than a specified duration
    static void checkFlagsDuration();

    // print the flags status
    static void printFlags();

    // return number of flags
    static uint8_t getFlags();

    // set the filter time
    static void setFilterTime(uint32_t time);
private:
    // flags
    static FlagsByte flags;

    // timer for filtering the interruptions
    static uint32_t filterTime;
};

#endif // FLAGS_H
