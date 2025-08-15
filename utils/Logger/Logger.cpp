#include <Logger.h>

// messages vector
std::vector<message> Logger::messages;

// index of the last message printed
uint32_t Logger::last_index = 0;

// log message
void Logger::IN_LOG(String msg, logType type) {
    // create message
    message m;
    m.timer = millis();
    m.msg = msg;
    m.type = type;

    // add message to vector
    messages.push_back(m);
}

// print message
void Logger::OUT_LOGGER(logType type) {
    // print all messages
    for (auto m : messages) {
        if (m.type == type)
            Serial.printf("[%d] %s\n", m.timer, m.msg.c_str());
        else if (type == logType::NONE)
            Serial.printf("[%7d] %s\n", m.timer, m.msg.c_str());
    }
}

// clear log
void Logger::CLEAR_LOG() {
    // clear all messages
    messages.clear();
}

String Logger::logTypeToString(logType type) {
    switch (type) {
        case logType::NONE:         return "NONE";
        case logType::INFO:         return "INFO";
        case logType::TELEMETRY:    return "TELEMETRY";
        case logType::ERROR:        return "ERROR";
        case logType::DEBUG:        return "DEBUG";
        default:                    return "UNKNOWN";
    }
}

// print all messages
void Logger::OUT_LOGGER_LIVE() {
    // print all messages
    for (uint32_t i = last_index; i < messages.size(); i++) {
        // espnow send
        String message_str = "[" + String(messages[i].timer) + "] " + logTypeToString(messages[i].type) + ": " + messages[i].msg;
        send_data((const uint8_t *)message_str.c_str(), message_str.length());
    }

    // update last index
    last_index = messages.size();
}