#ifndef FLAGS_H
#define FLAGS_H

#include <Arduino.h>

class Flags {
public:
    // handle the interrupt service routines
    static void IRAM_ATTR isrBtn1();
    static void IRAM_ATTR isrBtn2();
    static void IRAM_ATTR isrBtn3();
    static void IRAM_ATTR isrLeft();
    static void IRAM_ATTR isrRight();

    // check if any flag has been up for longer than a specified duration
    static void checkFlagsDuration();

    // print the flags status
    static void printFlags();

    // timer for filtering the interruptions
    static uint32_t filterTime;
private:
    // flag variables and timing variables
    static bool btn1;
    static bool btn2;
    static bool btn3;
    static bool left;
    static bool right;

    static uint32_t btn1Time;
    static uint32_t btn2Time;
    static uint32_t btn3Time;
    static uint32_t leftTime;
    static uint32_t rightTime;
};

#endif // FLAGS_H
