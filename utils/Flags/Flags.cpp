#include "Flags.h"

// timer for filtering the interruptions (default)
uint32_t Signals_IN::filterTime = 100;

// flags for the buttons
FlagsByte Signals_IN::buttons;

// flags for the side sensors
FlagsByte Signals_IN::sideSensors ;

void Signals_IN::setFilterTime(uint32_t time) {
    filterTime = time;
    delay(time);
}

// ---- handle the interrupt service routines ----
void IRAM_ATTR Signals_IN::isrBtn0() {
    if (millis() - buttons.time[0] >= filterTime) {
        buttons.time[0] = millis(); 
        buttons.flag0 = true;
    }
}

void IRAM_ATTR Signals_IN::isrBtn1() {
    if (millis() - buttons.time[1] >= filterTime) {
        buttons.time[1] = millis(); 
        buttons.flag1 = true;
    }
}

void IRAM_ATTR Signals_IN::isrBtn2() {
    if (millis() - buttons.time[2] >= filterTime) {
        buttons.time[2] = millis(); 
        buttons.flag2 = true;
    }
}

void IRAM_ATTR Signals_IN::isrBtn3() {
    if (millis() - buttons.time[3] >= filterTime) {
        buttons.time[3] = millis(); 
        buttons.flag3 = true;
    }
}

void IRAM_ATTR Signals_IN::isrBtn4() {
    if (millis() - buttons.time[4] >= filterTime) {
        buttons.time[4] = millis(); 
        buttons.flag4 = true;
    }
}

void IRAM_ATTR Signals_IN::isrBtn5() {
    if (millis() - buttons.time[5] >= filterTime) {
        buttons.time[5] = millis(); 
        buttons.flag5 = true;
    }
}

void IRAM_ATTR Signals_IN::isrBtn6() {
    if (millis() - buttons.time[6] >= filterTime) {
        buttons.time[6] = millis(); 
        buttons.flag6 = true;
    }
}

void IRAM_ATTR Signals_IN::isrBtn7() {
    if (millis() - buttons.time[7] >= filterTime) {
        buttons.time[7] = millis(); 
        buttons.flag7 = true;
    }
}

void IRAM_ATTR Signals_IN::isrsideSensor0() {
    if (millis() - sideSensors.time[0] >= filterTime) {
        sideSensors.time[0] = millis(); 
        sideSensors.flag0 = true;
    }
}

void IRAM_ATTR Signals_IN::isrsideSensor1() {
    if (millis() - sideSensors.time[1] >= filterTime) {
        sideSensors.time[1] = millis(); 
        sideSensors.flag1 = true;
    }
}

void IRAM_ATTR Signals_IN::isrsideSensor2() {
    if (millis() - sideSensors.time[2] >= filterTime) {
        sideSensors.time[2] = millis(); 
        sideSensors.flag2 = true;
    }
}

void IRAM_ATTR Signals_IN::isrsideSensor3() {
    if (millis() - sideSensors.time[3] >= filterTime) {
        sideSensors.time[3] = millis(); 
        sideSensors.flag3 = true;
    }
}

void IRAM_ATTR Signals_IN::isrsideSensor4() {
    if (millis() - sideSensors.time[4] >= filterTime) {
        sideSensors.time[4] = millis(); 
        sideSensors.flag4 = true;
    }
}

void IRAM_ATTR Signals_IN::isrsideSensor5() {
    if (millis() - sideSensors.time[5] >= filterTime) {
        sideSensors.time[5] = millis(); 
        sideSensors.flag5 = true;
    }
}

void IRAM_ATTR Signals_IN::isrsideSensor6() {
    if (millis() - sideSensors.time[6] >= filterTime) {
        sideSensors.time[6] = millis(); 
        sideSensors.flag6 = true;
    }
}

void IRAM_ATTR Signals_IN::isrsideSensor7() {
    if (millis() - sideSensors.time[7] >= filterTime) {
        sideSensors.time[7] = millis(); 
        sideSensors.flag7 = true;
    }
}

// ---- check if any flag has been up for longer than a specified duration ----
void Signals_IN::checkFlagsDuration() {
    uint32_t currentTime = millis();

    // check if any flag has been up for longer than a specified filterTime
    buttons.flag0 = currentTime - buttons.time[0] <= filterTime;
    buttons.flag1 = currentTime - buttons.time[1] <= filterTime;
    buttons.flag2 = currentTime - buttons.time[2] <= filterTime;
    buttons.flag3 = currentTime - buttons.time[3] <= filterTime;
    buttons.flag4 = currentTime - buttons.time[4] <= filterTime;
    buttons.flag5 = currentTime - buttons.time[5] <= filterTime;
    buttons.flag6 = currentTime - buttons.time[6] <= filterTime;
    buttons.flag7 = currentTime - buttons.time[7] <= filterTime;

    // check if any flag has been up for longer than a specified filterTime
    sideSensors.flag0 = currentTime - sideSensors.time[0] <= filterTime;
    sideSensors.flag1 = currentTime - sideSensors.time[1] <= filterTime;
    sideSensors.flag2 = currentTime - sideSensors.time[2] <= filterTime;
    sideSensors.flag3 = currentTime - sideSensors.time[3] <= filterTime;
    sideSensors.flag4 = currentTime - sideSensors.time[4] <= filterTime;
    sideSensors.flag5 = currentTime - sideSensors.time[5] <= filterTime;
    sideSensors.flag6 = currentTime - sideSensors.time[6] <= filterTime;
    sideSensors.flag7 = currentTime - sideSensors.time[7] <= filterTime;
}

uint8_t Signals_IN::getButtons() {
    return buttons.allFlags;
}

uint8_t Signals_IN::getSideSensors() {
    return sideSensors.allFlags;
}
