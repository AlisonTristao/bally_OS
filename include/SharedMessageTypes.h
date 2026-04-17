#pragma once

#include <stdint.h>

// Keep this file synchronized with t_dongle_develop/include/SharedMessageTypes.h.
enum class logType : uint8_t {
    NONE = 0,
    INFO,
    CMD,
    TELEMETRY,
    ERROR,
    DEBUG
};
