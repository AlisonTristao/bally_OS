#include <Logger.h>

Logger::Logger()
    : send_callback_(defaultSendCallback),
      command_callback_(defaultCommandCallback) {
    mutex_ = xSemaphoreCreateMutex();
}

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

    const size_t msgLen = msg.length();
    size_t packetCount = (msgLen + MAX_MESSAGE_SIZE - 1) / MAX_MESSAGE_SIZE;
    if (packetCount == 0) {
        packetCount = 1;
    }

    // packet index is 7-bit (0..127)
    if (packetCount > (static_cast<size_t>(MESSAGE_PACKET_MAX_INDEX) + 1U)) {
        packetCount = static_cast<size_t>(MESSAGE_PACKET_MAX_INDEX) + 1U;
    }

    for (size_t i = 0; i < packetCount; ++i) {
        const size_t start = i * MAX_MESSAGE_SIZE;
        const size_t remaining = (start < msgLen) ? (msgLen - start) : 0;
        const size_t n = (remaining > MAX_MESSAGE_SIZE) ? MAX_MESSAGE_SIZE : remaining;
        const bool lastPacket = (i + 1U) == packetCount;

        message m = {};
        m.timer = ts;
        m.type = (i == 0U) ? type : logType::PAKG;
        m.packetInfo = makePacketInfo(static_cast<uint8_t>(i), lastPacket);

        memset(m.msg, 0, sizeof(m.msg));
        if (n > 0) {
            strncpy(m.msg, msg.c_str() + start, n);
        }
        m.msg[n] = '\0';

        clear_logger();
        messages[message_count++] = m;
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
    if (mutex_ == NULL) return;
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(LOGGER_MUTEX_TIMEOUT_MS))) {
        if (message_count == MAX_MESSAGES) {
            message_count = 0;
        }
        xSemaphoreGive(mutex_);
    }
}

void Logger::send_logger_live() {
    if (mutex_ == NULL) return;
    // copy range under lock
    if (!xSemaphoreTake(mutex_, pdMS_TO_TICKS(LOGGER_MUTEX_TIMEOUT_MS))) return;
    uint32_t start = last_index;
    uint32_t end = message_count;
    xSemaphoreGive(mutex_);

    if (start >= MAX_MESSAGES) start = 0;
    if (end > MAX_MESSAGES) end = MAX_MESSAGES;

    if (start == end) return;

    auto send_entry = [this](uint32_t idx) {
        send_callback_(reinterpret_cast<const uint8_t*>(&messages[idx]), sizeof(messages[idx]));
        // Optionally print if DEBUG is enabled
        #if defined(LOG_ALL) || defined(LOG_DEBUG)
            Serial.print("[");
            Serial.print(logTypeToString(messages[idx].type));
            Serial.print("] ");
            Serial.println(messages[idx].msg);
        #endif
    };

    if (start < end) {
        for (uint32_t i = start; i < end; ++i)
            send_entry(i);
    } else {
        // Buffer wrapped: send from last_index to end of array and then from 0 to message_count.
        for (uint32_t i = start; i < MAX_MESSAGES; ++i)
            send_entry(i);

        for (uint32_t i = 0; i < end; ++i)
            send_entry(i);
    }

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(LOGGER_MUTEX_TIMEOUT_MS))) {
        last_index = (end == MAX_MESSAGES) ? 0 : end;
        xSemaphoreGive(mutex_);
    }
}

String Logger::logTypeToString(logType type) const {
    switch (type) {
        case logType::INFO: return "INFO";
        case logType::CMD: return "CMD";
        case logType::TELEMETRY: return "TELEMETRY";
        case logType::ERROR: return "ERROR";
        case logType::DEBUG: return "DEBUG";
        case logType::PAKG: return "PAKG";
        case logType::NONE:
        default:
            return "NONE";
    }
}
