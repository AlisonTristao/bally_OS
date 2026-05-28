#include <Logger.h>

Logger::Logger()
    : send_callback_(defaultSendCallback) {
    mutex_ = nullptr;
}

Logger::~Logger() {
    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

void Logger::begin() {
    // verifiy if the logger is already initialized to avoid inserting duplicate logs and creating multiple mutexes
    if (initialized_) return;
    initialized_ = true;

    // create the mutex for the logger to ensure thread safety when multiple tasks are inserting logs concurrently
    if (mutex_ == nullptr)
        mutex_ = xSemaphoreCreateMutex();

    // add 4 logs
    // the max packet size, the max message size, the protocol overhead size and the max messages
    insert_log(logType::INFO, "Logger initialized.");
    insert_log(logType::INFO, "Max packet size: "           + String(MAX_PACKET_SIZE) + " bytes.");
    insert_log(logType::INFO, "Max message size: "          + String(MAX_CONTENT_SIZE) + " bytes.");
    insert_log(logType::INFO, "Overhead size: "             + String(PROTOCOL_OVERHEAD_SIZE) + " bytes.");
    insert_log(logType::INFO, "Buffer size: "               + String(MAX_PACKETS_IN_RAM) + ".");
}

bool Logger::wait_for_mutex() {
    if (mutex_ == nullptr) return false;
    // insert a debug to indicate 
    //insert_log(logType::DEBG, "Waiting for mutex...");
    return xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE;
}

void Logger::free_mutex() {
    if (mutex_ != nullptr)
        xSemaphoreGive(mutex_);
}

void Logger::set_send_callback(SendCallback callback) {
    // wait the mutex
    if (!wait_for_mutex()) return;

    // set the callback, if the callback is null, use the default callback that does nothing
    send_callback_ = (callback != nullptr) ? callback : defaultSendCallback;
    free_mutex();
}

bool Logger::defaultSendCallback(const uint8_t *data, size_t len) {
    (void)data;
    (void)len;
    return false;
}

void Logger::insert_log_int16_t(const int16_t *sound_data, size_t len) {
    // verify the mutex is created and try to take it before inserting the log,
    if (sound_data == nullptr || len == 0) return;

    // convert the sound data to send as bytes, calculating the byte length based on the number of int16_t elements and their size
    const size_t byte_len = len * sizeof(int16_t);
    const uint8_t* data = reinterpret_cast<const uint8_t*>(sound_data);

    // insert the log message into the buffer, using the current timestamp in milliseconds for the log entry
    if (!wait_for_mutex()) return;
    insert_log_impl(data, byte_len, logType::I2SD, millis());
    free_mutex();
}

void Logger::insert_logf(logType type, const char* format, ...) {
    // temporary buffer to hold the formatted string
    char buffer[256]; 

    // initialize the variable argument list
    va_list args;
    va_start(args, format);

    // format the string into the buffer and get the actual length
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // verify the mutex is created and try to take it before inserting the log
    if (!wait_for_mutex()) return;

    // check if formatting was successful and within bounds
    if (len > 0) {
        size_t final_len = (len < (int)sizeof(buffer)) ? (size_t)len : (sizeof(buffer) - 1);
        
        // convert the formatted buffer to bytes and call the implementation
        const uint8_t* data = reinterpret_cast<const uint8_t*>(buffer);
        insert_log_impl(data, final_len, type, millis());
    }

    // always release the mutex after the operation
    free_mutex();
}

void Logger::insert_log(logType type, const String& msg) {
    // verify the mutex is created and try to take it before inserting the log, 
    // to ensure thread safety when multiple tasks are inserting logs concurrently
    if (!wait_for_mutex()) return;

    // convert the string to send as bytes
    const uint8_t* data = reinterpret_cast<const uint8_t*>(const_cast<char*>(msg.c_str()));

    // insert the log message into the buffer, using the current timestamp in milliseconds for the log entry
    insert_log_impl(data, msg.length(), type, millis());
    free_mutex();
}

uint8_t Logger::calculate_checksum(const message& msg) {
    // simple checksum calculation by summing all the bytes of the message content, type, packet number, and total packets, modulo 256.
    uint32_t sum = 0;

    // get the length of the content to calculate the checksum, if the content is null, the length is 0
    const size_t len = (msg.content.byte == nullptr) ? 0 : msg.content.size;

    // sum all the bytes of the message content
    for (size_t i = 0; i < len; ++i) sum += static_cast<uint8_t>(msg.content.byte[i]);

    // sum the type, packet number, and total packets to the checksum calculation
    sum += static_cast<uint8_t>(msg.type);
    sum += static_cast<uint16_t>(msg.packet_number);
    sum += static_cast<uint16_t>(msg.total_packets);
    return static_cast<uint8_t>(sum % 256);
}

void Logger::insert_log_impl(const uint8_t* data, size_t len, logType type, uint32_t ts) {
    if (data == nullptr || len == 0) return;

    // calculates the number of packets needed to send the message, considering the maximum message size and protocol overhead
    uint16_t total_packets_to_send = (len + MAX_CONTENT_SIZE - 1) / MAX_CONTENT_SIZE;
    if (total_packets_to_send == 0) // should not happen since we check for empty message
        total_packets_to_send = 1;

    // loop to create and store each packet in the messages array, handling fragmentation if necessary
    for (uint16_t i = 0; i < total_packets_to_send; ++i) {
        // variables to get the caracteres for the current packet, 
        // calculating the start index and the number of characters to copy for this packet
        const uint16_t start      = i * MAX_CONTENT_SIZE;
        const uint16_t remaining  = (start < len) ? (len - start) : 0; // remaining characters to send starting from the current packet's start index
        const size_t length       = (remaining > MAX_CONTENT_SIZE) ? MAX_CONTENT_SIZE : remaining;    // length of the substring for the current packet

        if (message_count >= MAX_PACKETS_IN_RAM) {
            message_count = 0;
            last_index = 0; // drop old data to avoid re-sending stale messages
        }

        message& m          = messages[message_count++];
        m.timer             = ts;
        m.type              = type;
        m.packet_number     = i + 1;
        m.total_packets     = total_packets_to_send;
        m.content.size      = length;

        // copy the payload for the current packet into the message struct
        if (m.content.byte) {
            memset(m.content.byte, 0, MAX_CONTENT_SIZE + 1);
            if (length > 0)
                memcpy(m.content.byte, data + start, length);
            m.content.text[length] = '\0';
        }

        // calculate checksum with the sum of the bytes of the message content, type, packet number, and total packets, modulo 256. 
        // This is a simple checksum to verify the integrity of the message on the receiving end.
        m.checksum = calculate_checksum(m);
    }
}

void Logger::reset_loop_counter() {
    if (wait_for_mutex()) {
        if (message_count == MAX_PACKETS_IN_RAM)
            message_count = 0;
        free_mutex();
    }
}

void Logger::send_live_logger() {
    // verify the mutex is created and try to take it before sending the logs,
    if (!wait_for_mutex()) return;

    // get the init and the end index for the messages to send, 
    // this metod send the messagens using a circular buffer.
    uint32_t start = last_index;
    uint32_t end = message_count;

    // just verify the indexes to be in the correct range
    if (start >= MAX_PACKETS_IN_RAM)   start = 0;
    if (end > MAX_PACKETS_IN_RAM)      end = MAX_PACKETS_IN_RAM;
    if (start == end) {
        free_mutex();
        return; // nothing to send
    }     

    // send the messages from the buffer using the configured send callback, 
    // starting from the last index and going up to the current message count.
    auto send_entry = [this](uint32_t idx) {
        send_callback_(reinterpret_cast<const uint8_t*>(&messages[idx]), sizeof(messages[idx]));
    };

    // logic to send the messages in the correct order, using a circular buffer
    if (start < end) {
        for (uint32_t i = start; i < end; ++i)
            send_entry(i);
    } else {
        // Buffer wrapped: send from last_index to end of array and then from 0 to message_count.
        for (uint32_t i = start; i < MAX_PACKETS_IN_RAM; ++i)
            send_entry(i);

        for (uint32_t i = 0; i < end; ++i)
            send_entry(i);
    }
    
    // to change the last index to the current message count, we need to take the mutex
    last_index = (end == MAX_PACKETS_IN_RAM) ? 0 : end;
    free_mutex();
}

