#include <Logger.h>

// messages array
message Logger::messages[MAX_MESSAGES];
uint32_t Logger::message_count = 0;

Logger::SendCallback Logger::send_callback_ = Logger::defaultSendCallback;
Logger::CommandCallback Logger::command_callback_ = Logger::defaultCommandCallback;

// mutex for thread safety
bool Logger::mutex = true;

// index of the last message printed
uint32_t Logger::last_index = 0;

void Logger::setSendCallback(SendCallback callback) {
    send_callback_ = (callback != nullptr) ? callback : defaultSendCallback;
}

void Logger::setCommandCallback(CommandCallback callback) {
    command_callback_ = (callback != nullptr) ? callback : defaultCommandCallback;
}

bool Logger::defaultSendCallback(const uint8_t* data, size_t len) {
    (void)data;
    (void)len;
    return false;
}

void Logger::defaultCommandCallback(const String& cmd) {
    (void)cmd;
}

void Logger::insert_log_impl(const String& msg, logType type, uint32_t ts) {
    if (msg.length() == 0) return;

    // get the len of the message
    const size_t n = (msg.length() > MAX_MESSAGE_SIZE) ? MAX_MESSAGE_SIZE : msg.length();

    // create message struct
    message m;
    m.timer = ts;
    m.type  = type;

    // add data to message
    memset(m.msg, 0, sizeof(m.msg));
    strncpy(m.msg, msg.c_str(), n);
    m.msg[n] = '\0';

    // clear the log of messages
    clear_logger();

    // save the message in the array
    messages[message_count++] = m;

    // call recursively for the rest (if any)
    if (msg.length() > n) insert_log_impl(msg.substring(n), type, ts);
}

void Logger::insert_cmd_impl(const String& cmd) {
    command_callback_(cmd);
}

// print messages
void Logger::send_logger(logType type) {
    for (size_t i = 0; i < message_count; ++i)
        if (messages[i].type == type || type == logType::NONE)
            send_callback_(reinterpret_cast<const uint8_t*>(&messages[i]), sizeof(messages[i]));
}

// clear logger
void Logger::clear_logger() {
    // lock
    mutex = false;

    // clear all messages
    if (message_count == MAX_MESSAGES) {
        //memset(messages, 0, sizeof(messages));
        message_count = 0;
        last_index = 0;
    }

    // unlock
    mutex = true;
}

// print all messages
void Logger::send_logger_live() {
    // send all messages after last_index
    for (; last_index < message_count; ++last_index) {
        if (!mutex) return;
        send_callback_(reinterpret_cast<const uint8_t*>(&messages[last_index]), sizeof(messages[last_index]));
    }
}
