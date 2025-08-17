#include <Logger.h>

// messages vector
std::vector<message> Logger::messages;

// index of the last message printed
uint32_t Logger::last_index = 0;

void Logger::insert_log_impl(const String& msg, logType type, uint32_t ts) {
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
    if (msg.length() > n) insert_log_impl(msg.substring(n), type, ts);
}

void Logger::insert_cmd_impl(const String& cmd) {
    run_command(cmd);
}

// print messages
void Logger::send_logger(logType type) {
    for (size_t i = 0; i < messages.size(); ++i)
        if (messages[i].type == type || type == logType::NONE)
            send_data(reinterpret_cast<const uint8_t*>(&messages[i]), sizeof(messages[i]));
}

// clear logger
void Logger::clear_logger() {
    // clear all messages
    messages.clear();
}

// print all messages
void Logger::send_logger_live() {
    // send all messages after last_index
    for (; last_index < messages.size(); ++last_index)
        send_data(reinterpret_cast<const uint8_t*>(&messages[last_index]), sizeof(messages[last_index]));
}
