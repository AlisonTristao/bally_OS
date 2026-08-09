#ifndef LOG_TYPES_H
#define LOG_TYPES_H

#include <cstdint>

// LOG object_id values used by the firmware. The payload is the exact UTF-8
// text, without an implicit terminator.
enum class logType : std::uint8_t {
    NONE = 0,
    INFO = 1,
    WARN = 2,
    ERRO = 3,
    DEBG = 4,
    CMDO = 5,
};

inline constexpr const char* logTypeToString(logType type) {
    switch (type) {
        case logType::INFO: return "INFO";
        case logType::WARN: return "WARN";
        case logType::ERRO: return "ERRO";
        case logType::DEBG: return "DEBG";
        case logType::CMDO: return "CMDO";
        case logType::NONE: return "NONE";
    }
    return "UNKN";
}

#endif  // LOG_TYPES_H
