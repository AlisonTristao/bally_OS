#include "Flags.h"

// flags
FlagsByte Flags::flags;

// timer for filtering the interruptions (default)
uint32_t Flags::filterTime = 100;

void Flags::setFilterTime(uint32_t time) {
    filterTime = time;
    delay(time);
}

// ---- handle the interrupt service routines ----
void IRAM_ATTR Flags::isrFlag0() {
    if (millis() - flags.time[0] >= filterTime) {
        flags.time[0] = millis(); 
        flags.flag0 = true;
    }
}

void IRAM_ATTR Flags::isrFlag1() {
    if (millis() - flags.time[1] >= filterTime) {
        flags.time[1] = millis(); 
        flags.flag1 = true;
    }
}

void IRAM_ATTR Flags::isrFlag2() {
    if (millis() - flags.time[2] >= filterTime) {
        flags.time[2] = millis(); 
        flags.flag2 = true;
    }
}

void IRAM_ATTR Flags::isrFlag3() {
    if (millis() - flags.time[3] >= filterTime) {
        flags.time[3] = millis(); 
        flags.flag3 = true;
    }
}

void IRAM_ATTR Flags::isrFlag4() {
    if (millis() - flags.time[4] >= filterTime) {
        flags.time[4] = millis(); 
        flags.flag4 = true;
    }
}

void IRAM_ATTR Flags::isrFlag5() {
    if (millis() - flags.time[5] >= filterTime) {
        flags.time[5] = millis(); 
        flags.flag5 = true;
    }
}

void IRAM_ATTR Flags::isrFlag6() {
    if (millis() - flags.time[6] >= filterTime) {
        flags.time[6] = millis(); 
        flags.flag6 = true;
    }
}

void IRAM_ATTR Flags::isrFlag7() {
    if (millis() - flags.time[7] >= filterTime) {
        flags.time[7] = millis(); 
        flags.flag7 = true;
    }
}

// check if any flag has been up for longer than a specified filterTime
void Flags::checkFlagsDuration() {
    uint32_t currentTime = millis();
    flags.flag0 = currentTime - flags.time[0] <= filterTime;
    flags.flag1 = currentTime - flags.time[1] <= filterTime;
    flags.flag2 = currentTime - flags.time[2] <= filterTime;
    flags.flag3 = currentTime - flags.time[3] <= filterTime;
    flags.flag4 = currentTime - flags.time[4] <= filterTime;
    flags.flag5 = currentTime - flags.time[5] <= filterTime;
    flags.flag6 = currentTime - flags.time[6] <= filterTime;
    flags.flag7 = currentTime - flags.time[7] <= filterTime;
}

void Flags::printFlags() {
    Serial.printf("%d%d%d%d%d%d%d%d\t%d\n",     flags.flag0,
                                                flags.flag1,
                                                flags.flag2,
                                                flags.flag3,
                                                flags.flag4,
                                                flags.flag5,
                                                flags.flag6,
                                                flags.flag7,
                                                flags.allFlags);
}

uint8_t Flags::getFlags() {
    return flags.allFlags;
}
