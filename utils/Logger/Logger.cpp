#include <Logger.h>

// messages vector
std::vector<message> Logger::messages;

// index of the last message printed
uint32_t Logger::last_index = 0;

void Logger::IN_LOG_impl(const String& msg, logType type, unsigned long ts) {
    if (msg.length() == 0) return;

    // get the len of the message
    const size_t n = (msg.length() > MAX_MESSAGE_SIZE + 1) ? MAX_MESSAGE_SIZE + 1 : msg.length();

    // create message struct
    message m;
    m.timer = ts;
    m.type  = type;

    // add data to message
    memset(m.msg, 0, sizeof(m.msg));
    strncpy(m.msg, msg.c_str(), n);
    m.msg[n] = '\0';

    // save the message in the vector
    messages.push_back(m);

    // call recursively for the rest (if any)
    if (msg.length() > n) IN_LOG_impl(msg.substring(n), type, ts);
}

// print message
void Logger::OUT_LOGGER(logType type) {
    // print all messages
    for (auto m : messages) {
        if (m.type == type)
            Serial.printf("[%d] %s\n", m.timer, m.msg);
        else if (type == logType::NONE)
            Serial.printf("[%7d] %s\n", m.timer, m.msg);
    }
}

// clear log
void Logger::CLEAR_LOG() {
    // clear all messages
    messages.clear();
}

// print all messages
void Logger::OUT_LOGGER_LIVE() {
    // send all new messages to espnow
    for (uint32_t i = last_index; i < messages.size(); i++)
        send_data((const uint8_t *) &messages[i], sizeof(messages[i]));

    // update last index
    last_index = messages.size();
}