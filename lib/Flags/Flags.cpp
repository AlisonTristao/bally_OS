#include "Flags.h"

// ===== FlagsBase =====

uint8_t FlagsBase::getFlags() const {
    return flags.allFlags;
}

bool FlagsBase::isValidIndex(uint8_t index) const {
    return index < MAX_FLAGS;
}

void FlagsBase::setTimeLimit(uint8_t index, uint32_t time) {
    if (!isValidIndex(index))
        return;

    timeLimit[index] = time;
}

void FlagsBase::setFilterTime(uint32_t time) {
    for (int i = 0; i < MAX_FLAGS; i++)
        timeLimit[i] = time;
}

void FlagsBase::checkFlagsDuration() {
    refreshFlags(millis());
}

void FlagsBase::refreshFlags(uint32_t currentTime) {
    for (uint8_t i = 0; i < MAX_FLAGS; i++)
        if (flags.allFlags & (1 << i))
            if (currentTime - flags.time[i] > timeLimit[i])
                flags.allFlags &= ~(1 << i);
}

// ===== Flags_in =====

void IRAM_ATTR Flags_in::isr(void* arg) {
    FlagsIsrArg* data = static_cast<FlagsIsrArg*>(arg);

    if (data == nullptr || data->obj == nullptr)
        return;

    data->obj->handleUpdate(data->index);
}

void Flags_in::setFlag(uint8_t index) {
    if (!isValidIndex(index))
        return;

    flags.time[index] = millis();
    flags.allFlags |= (1 << index);
}

void IRAM_ATTR Flags_in::handleUpdate(uint8_t index) {
    if (!isValidIndex(index))
        return;

    uint32_t now = millis();

    // Check if enough time has passed since last activation
    if (!(now - flags.time[index] >= timeLimit[index]))
        return;

    flags.time[index] = now;
    flags.allFlags |= (1 << index);
}

// ===== Flags_out =====

void Flags_out::setFlag(uint8_t index, uint32_t time) {
    if (!isValidIndex(index))
        return;

    flags.time[index] = millis();
    flags.allFlags |= (1 << index);
    timeLimit[index] = time;
}

// ===== Flags_pwm ======

void Flags_pwm::setValue(uint8_t index, int16_t value, uint32_t time) {
    if (!isValidIndex(index))
        return;

    pwmValues[index] = value;
    flags.time[index] = millis();
    timeLimit[index] = time;
    flags.allFlags |= (1 << index);
}

int16_t Flags_pwm::getValue(uint8_t index) const {
    if (!isValidIndex(index))
        return 0;

    return pwmValues[index];
}