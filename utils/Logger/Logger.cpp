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

// print all messages
void Logger::OUT_LOGGER_LIVE() {
    // print all messages
    for (uint32_t i = last_index; i < messages.size(); i++) {
        Serial.printf("[%7d] %s\n", messages[i].timer, messages[i].msg.c_str());
    }

    // update last index
    last_index = messages.size();
}