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
    String result = StaticObjects::shell.run_line_command(string(cmd.c_str())).c_str();
    Logger::insert_log(result, logType::CMD);
}

// print messages
void Logger::send_logger_register(logType type) {
    // send all messages to espnow
    for (auto m : messages) {
        if (m.type == type)
            send_data((const uint8_t *) &m, sizeof(m));
    }
}

// clear log
void Logger::clear_logger() {
    // clear all messages
    messages.clear();
}

// print all messages
void Logger::send_logger_register_live() {
    // send all new messages to espnow
    for (uint32_t i = last_index; i < messages.size(); i++)
        send_data((const uint8_t *) &messages[i], sizeof(messages[i]));

    // update last index
    last_index = messages.size();
}