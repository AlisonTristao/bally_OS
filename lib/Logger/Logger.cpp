#include <Logger.h>

Logger::Logger()
    : send_callback_(defaultSendCallback),
      command_callback_(defaultCommandCallback) {}

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

void Logger::insert_log(const String& msg, logType type) {
    insert_log_impl(msg, type, millis());
}

void Logger::insert_log_impl(const String& msg, logType type, uint32_t ts) {
    if (msg.length() == 0) return;

    const size_t n = (msg.length() > MAX_MESSAGE_SIZE) ? MAX_MESSAGE_SIZE : msg.length();

    message m = {};
    m.timer = ts;
    m.type = type;

    memset(m.msg, 0, sizeof(m.msg));
    strncpy(m.msg, msg.c_str(), n);
    m.msg[n] = '\0';

    clear_logger();
    messages[message_count++] = m;

    if (msg.length() > n) {
        insert_log_impl(msg.substring(n), type, ts);
    }
}

void Logger::insert_cmd(const String& cmd) {
    command_callback_(cmd);
}

void Logger::send_logger(logType type) {
    for (size_t i = 0; i < message_count; ++i) {
        if (messages[i].type == type || type == logType::NONE) {
            send_callback_(reinterpret_cast<const uint8_t*>(&messages[i]), sizeof(messages[i]));
        }
    }
}

void Logger::clear_logger() {
    mutex = false;

    if (message_count == MAX_MESSAGES) {
        message_count = 0;
        last_index = 0;
    }

    mutex = true;
}

void Logger::send_logger_live() {
    for (; last_index < message_count; ++last_index) {
        if (!mutex) return;
        send_callback_(reinterpret_cast<const uint8_t*>(&messages[last_index]), sizeof(messages[last_index]));
    }
}

String Logger::logTypeToString(logType type) const {
    switch (type) {
        case logType::INFO: return "INFO";
        case logType::CMD: return "CMD";
        case logType::TELEMETRY: return "TELEMETRY";
        case logType::ERROR: return "ERROR";
        case logType::DEBUG: return "DEBUG";
        case logType::NONE:
        default:
            return "NONE";
    }
}
