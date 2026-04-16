#ifndef FLAGS_H
#define FLAGS_H

#include <Arduino.h>

#define BIT_0 0
#define BIT_1 1
#define BIT_2 2
#define BIT_3 3
#define BIT_4 4
#define BIT_5 5
#define BIT_6 6
#define BIT_7 7

// Stores 8 flags in a byte + timestamp for each flag
struct FlagsByte {
    union {
        uint8_t allFlags = 0; // full byte access

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

    uint32_t time[8] = {}; // activation time for each flag
};

// Abstract base class shared by all flag types
class FlagsBase {
public:
    // Constructor with optional debug name
    FlagsBase(const String& name = "", uint32_t timeLimit = 100)
        : name(name) {
            for (int i = 0; i < 8; i++) {
                this->timeLimit[i] = timeLimit;
            }
        }

    // Virtual destructor for safe inheritance
    virtual ~FlagsBase() {};

    // Returns all flags as a byte
    uint8_t getFlags() const;

    // Sets time limit in milliseconds
    void setTimeLimit(uint8_t index, uint32_t time);
    void setFilterTime(uint32_t time);

    // Checks and clears flags based on duration
    void checkFlagsDuration();

    // String representation (can be overridden)
    String toString() const {
        return name + " Flags: " + String(flags.allFlags, BIN);
    };
protected:
    FlagsByte flags;
    String name;
    uint32_t timeLimit[8];

    // Updates flags based on current time
    void refreshFlags(uint32_t currentTime);
};

// Input flags controlled by interrupts
class Flags_in : public FlagsBase {
public:
    // Constructor with optional debug name
    Flags_in(const String& name = "") : FlagsBase(name) {}        
    virtual ~Flags_in() {};
    // Static ISR required for attachInterruptArg
    static void IRAM_ATTR isr(void* arg);
    // Sets a specific flag from an ISR
    void setFlag(uint8_t index);
protected:
    // Handles flag update triggered by interrupt
    void handleUpdate(uint8_t index);
};

// Digital output flags controlled by software
class Flags_out : public FlagsBase {
public:
    Flags_out(const String& name = "") : FlagsBase(name) {}
    virtual ~Flags_out() {};
    // Sets a specific flag manually
    void setFlag(uint8_t index, uint32_t time);
};

// PWM output flags storing values from 0 to 100
class Flags_pwm : public FlagsBase {
public:
    // Constructor with optional debug name
    Flags_pwm(const String& name = "") : FlagsBase(name) {}
    virtual ~Flags_pwm() {};
    // Sets PWM value (0–100)
    void setValue(uint8_t index, int16_t value, uint32_t time);

    // Gets PWM value
    int16_t getValue(uint8_t index) const;
protected:
    int16_t pwmValues[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // stores PWM values per channel
};

struct FlagsIsrArg {
    Flags_in* obj;
    uint8_t index;
};

#endif // FLAGS_H