#include "Flags.h"

// ===== FlagsBase =====

uint8_t FlagsBase::getFlags() const {
    return flags.allFlags;
}

void FlagsBase::setFilterTime(uint32_t time) {
    for (int i = 0; i < 8; i++)
        timeLimit[i] = time;
}

void FlagsBase::checkFlagsDuration() {
    refreshFlags(millis());
}

void FlagsBase::refreshFlags(uint32_t currentTime) {
    for (uint8_t i = 0; i < 8; i++)
        if (flags.allFlags & (1 << i))
            if (currentTime - flags.time[i] > timeLimit[i])
                flags.allFlags &= ~(1 << i);
}

// ===== Flags_in =====

void IRAM_ATTR Flags_in::isr(void* arg) {
    // Cast back to object
    uint8_t index = (uintptr_t) arg;
    handleUpdate(index);
}

void Flags_in::handleUpdate(uint8_t index) {
    uint32_t now = millis();

    // Check if enough time has passed since last activation
    if (!(now - flags.time[index] >= timeLimit[index]))
        return;

    flags.time[index] = now;
    flags.allFlags |= (1 << index);
}

// ===== Flags_out =====

void Flags_out::setFlag(uint8_t index, uint32_t time) {
    flags.time[index] = millis();
    flags.allFlags |= (1 << index);
    timeLimit[index] = time;
}

// ===== Flags_pwm ======

void Flags_pwm::setValue(uint8_t index, int16_t value, uint32_t time) {
    pwmValues[index] = value;
    flags.time[index] = millis();
    timeLimit[index] = time;
    flags.allFlags |= (1 << index);
}

int16_t Flags_pwm::getValue(uint8_t index) const {
    return pwmValues[index];
}