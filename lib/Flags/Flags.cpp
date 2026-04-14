#include "Flags.h"

Signals_IN* Signals_IN::instance = nullptr;

Signals_IN::Signals_IN() {
    instance = this;
}

void Signals_IN::setFilterTime(uint32_t time) {
    filterTime = time;
    delay(time);
}

void IRAM_ATTR Signals_IN::handleInterrupt(FlagsByte& flags, uint8_t index) {
    const uint32_t now = millis();
    if (now - flags.time[index] >= filterTime) {
        flags.time[index] = now;
        bitSet(flags.allFlags, index);
    }
}

void Signals_IN::refreshFlags(FlagsByte& flags, uint32_t currentTime) {
    flags.flag0 = currentTime - flags.time[0] <= filterTime;
    flags.flag1 = currentTime - flags.time[1] <= filterTime;
    flags.flag2 = currentTime - flags.time[2] <= filterTime;
    flags.flag3 = currentTime - flags.time[3] <= filterTime;
    flags.flag4 = currentTime - flags.time[4] <= filterTime;
    flags.flag5 = currentTime - flags.time[5] <= filterTime;
    flags.flag6 = currentTime - flags.time[6] <= filterTime;
    flags.flag7 = currentTime - flags.time[7] <= filterTime;
}

void Signals_IN::checkFlagsDuration() {
    const uint32_t currentTime = millis();
    refreshFlags(buttons, currentTime);
    refreshFlags(sideSensors, currentTime);
    refreshFlags(leds, currentTime);
}

uint8_t Signals_IN::getButtons() const {
    return buttons.allFlags;
}

uint8_t Signals_IN::getSideSensors() const {
    return sideSensors.allFlags;
}

uint8_t Signals_IN::getLeds() const {
    return leds.allFlags;
}

void IRAM_ATTR Signals_IN::handleButtonISR(uint8_t index) {
    if (instance != nullptr) {
        instance->handleInterrupt(instance->buttons, index);
    }
}

void IRAM_ATTR Signals_IN::handleSideSensorISR(uint8_t index) {
    if (instance != nullptr) {
        instance->handleInterrupt(instance->sideSensors, index);
    }
}

void IRAM_ATTR Signals_IN::handleLedISR(uint8_t index) {
    if (instance != nullptr) {
        instance->handleInterrupt(instance->leds, index);
    }
}

void IRAM_ATTR Signals_IN::isrBtn0() { handleButtonISR(0); }
void IRAM_ATTR Signals_IN::isrBtn1() { handleButtonISR(1); }
void IRAM_ATTR Signals_IN::isrBtn2() { handleButtonISR(2); }
void IRAM_ATTR Signals_IN::isrBtn3() { handleButtonISR(3); }
void IRAM_ATTR Signals_IN::isrBtn4() { handleButtonISR(4); }
void IRAM_ATTR Signals_IN::isrBtn5() { handleButtonISR(5); }
void IRAM_ATTR Signals_IN::isrBtn6() { handleButtonISR(6); }
void IRAM_ATTR Signals_IN::isrBtn7() { handleButtonISR(7); }

void IRAM_ATTR Signals_IN::isrsideSensor0() { handleSideSensorISR(0); }
void IRAM_ATTR Signals_IN::isrsideSensor1() { handleSideSensorISR(1); }
void IRAM_ATTR Signals_IN::isrsideSensor2() { handleSideSensorISR(2); }
void IRAM_ATTR Signals_IN::isrsideSensor3() { handleSideSensorISR(3); }
void IRAM_ATTR Signals_IN::isrsideSensor4() { handleSideSensorISR(4); }
void IRAM_ATTR Signals_IN::isrsideSensor5() { handleSideSensorISR(5); }
void IRAM_ATTR Signals_IN::isrsideSensor6() { handleSideSensorISR(6); }
void IRAM_ATTR Signals_IN::isrsideSensor7() { handleSideSensorISR(7); }

void IRAM_ATTR Signals_IN::setLed0on() { handleLedISR(0); }
void IRAM_ATTR Signals_IN::setLed1on() { handleLedISR(1); }
void IRAM_ATTR Signals_IN::setLed2on() { handleLedISR(2); }
void IRAM_ATTR Signals_IN::setLed3on() { handleLedISR(3); }
void IRAM_ATTR Signals_IN::setLed4on() { handleLedISR(4); }
void IRAM_ATTR Signals_IN::setLed5on() { handleLedISR(5); }
void IRAM_ATTR Signals_IN::setLed6on() { handleLedISR(6); }
void IRAM_ATTR Signals_IN::setLed7on() { handleLedISR(7); }
