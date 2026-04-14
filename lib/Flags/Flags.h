#ifndef FLAGS_H
#define FLAGS_H

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

#include <Arduino.h>

#define BIT_0 0
#define BIT_1 1
#define BIT_2 2
#define BIT_3 3
#define BIT_4 4
#define BIT_5 5
#define BIT_6 6
#define BIT_7 7

struct FlagsByte {
    union {
        uint8_t allFlags = 0;
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
    uint32_t time[8] = {};
};

class Signals_IN {
public:
    Signals_IN();

    static void IRAM_ATTR isrBtn0();
    static void IRAM_ATTR isrBtn1();
    static void IRAM_ATTR isrBtn2();
    static void IRAM_ATTR isrBtn3();
    static void IRAM_ATTR isrBtn4();
    static void IRAM_ATTR isrBtn5();
    static void IRAM_ATTR isrBtn6();
    static void IRAM_ATTR isrBtn7();

    static void IRAM_ATTR isrsideSensor0();
    static void IRAM_ATTR isrsideSensor1();
    static void IRAM_ATTR isrsideSensor2();
    static void IRAM_ATTR isrsideSensor3();
    static void IRAM_ATTR isrsideSensor4();
    static void IRAM_ATTR isrsideSensor5();
    static void IRAM_ATTR isrsideSensor6();
    static void IRAM_ATTR isrsideSensor7();

    static void IRAM_ATTR setLed0on();
    static void IRAM_ATTR setLed1on();
    static void IRAM_ATTR setLed2on();
    static void IRAM_ATTR setLed3on();
    static void IRAM_ATTR setLed4on();
    static void IRAM_ATTR setLed5on();
    static void IRAM_ATTR setLed6on();
    static void IRAM_ATTR setLed7on();

    void checkFlagsDuration();
    uint8_t getButtons() const;
    uint8_t getSideSensors() const;
    uint8_t getLeds() const;
    void setFilterTime(uint32_t time);

private:
    FlagsByte buttons{};
    FlagsByte sideSensors{};
    FlagsByte leds{};
    uint32_t filterTime = 100;

    static Signals_IN* instance;

    void handleInterrupt(FlagsByte& flags, uint8_t index);
    void refreshFlags(FlagsByte& flags, uint32_t currentTime);

    static void IRAM_ATTR handleButtonISR(uint8_t index);
    static void IRAM_ATTR handleSideSensorISR(uint8_t index);
    static void IRAM_ATTR handleLedISR(uint8_t index);
};

#endif // FLAGS_H
