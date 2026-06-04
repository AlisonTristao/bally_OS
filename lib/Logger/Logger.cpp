#include <Logger.h>

#include "esp_timer.h"

#include <cstring>
#include <cstdio>

Logger::Logger()
    : send_callback_(defaultSendCallback) {
    mutex_ = nullptr;
    // Alocação com segurança
    size_t total_size = sizeof(message) * MAX_PACKETS_IN_PSRAM;
    messages = (message*) heap_caps_malloc(total_size, MALLOC_CAP_SPIRAM);

    // Checagem vital!
    if (messages == nullptr) {
        // Se falhou, logue um erro crítico antes que o sistema trave
        printf("CRITICAL: Failed to allocate %d bytes in PSRAM!\n", total_size);
    }
}

void Logger::begin() {
    // verifiy if the logger is already initialized to avoid inserting duplicate logs and creating multiple mutexes
    if (initialized_) return;

    initialized_ = true;

    // create the mutex for the logger to ensure thread safety when multiple tasks are inserting logs concurrently
    if (mutex_ == nullptr)
        mutex_ = xSemaphoreCreateMutex();
}

bool Logger::wait_for_mutex() {
    if (mutex_ == nullptr) return false;

    return xSemaphoreTake(mutex_, pdMS_TO_TICKS(2)) == pdTRUE;
}

bool Logger::check_mutex() {
    if (mutex_ == nullptr) return false;

    return xSemaphoreTake(mutex_, 0) == pdTRUE;
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

        insert_log_impl(data, final_len, type, (uint32_t)(esp_timer_get_time() / 1000ULL));
    }

    // always release the mutex after the operation
    free_mutex();
}

void Logger::insert_log(logType type, const char* msg) {
    if (msg == nullptr) return;

    // verify the mutex is created and try to take it before inserting the log, 
    // to ensure thread safety when multiple tasks are inserting logs concurrently
    if (!wait_for_mutex()) return;

    // convert the string to send as bytes
    const uint8_t* data = reinterpret_cast<const uint8_t*>(msg);

    // insert the log message into the buffer, using the current timestamp in milliseconds for the log entry
    insert_log_impl(data, strlen(msg), type, (uint32_t)(esp_timer_get_time() / 1000ULL));

    free_mutex();
}

uint8_t Logger::calculate_checksum(const message& msg) {
    // simple checksum calculation by summing all the bytes of the message content, type, packet number, and total packets, modulo 256.
    uint32_t sum = 0;

    // get the length of the content to calculate the checksum, if the content is null, the length is 0
    const size_t len = msg.content.size;

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

        // warning - if the buffer is full and the logger is not sending the messages fast enough, 
        // we will start overwriting the old messages in the buffer,
        message& m = messages[write_index];

        m.timer             = ts;
        m.type              = type;
        m.packet_number     = i + 1;
        m.total_packets     = total_packets_to_send;
        m.content.size      = length;

        // copy the payload for the current packet into the message struct
        memset(m.content.byte, 0, MAX_CONTENT_SIZE + 1);
        memcpy(m.content.byte, data + start, length);

        // calculate checksum with the sum of the bytes of the message content, type, packet number, and total packets, modulo 256. 
        // This is a simple checksum to verify the integrity of the message on the receiving end.
        m.checksum = calculate_checksum(m);

        write_index = (write_index + 1) % MAX_PACKETS_IN_PSRAM;

        if (pending_count < MAX_PACKETS_IN_PSRAM) {
            pending_count++;
        } else {
            read_index = (read_index + 1) % MAX_PACKETS_IN_PSRAM;
        }
    }
}

void Logger::reset_loop_counter() {
    if (wait_for_mutex()) {
        write_index = 0;
        read_index = 0;
        pending_count = 0;

        free_mutex();
    }
}

void Logger::flush_logs() {
    // define chunk size to balance mutex locking and stack usage
    // this is used to free the mutex to other tasks while empty the array
    const uint32_t chunk_size = BLOCK_SIZE;  // size multiple os 2 for better performance
    message temp_buffer[chunk_size]; // local buffer on stack to copy messages safely 

    // loop until all messages are sent
    for (uint32_t chunk = 0; chunk < MAX_CHUNKS_PER_FLUSH; ++chunk) {
        uint32_t to_copy = 0;

        // lock mutex only to copy the block of messages
        if (!wait_for_mutex()) return;

        // check if there are any messages to process
        if (pending_count == 0) {
            free_mutex();
            break; // exit loop if buffer is empty
        }

        // determine how many messages to copy in this iteration
        to_copy = (pending_count < chunk_size) ? pending_count : chunk_size;

        // copy block from psram to local stack buffer
        for (uint32_t i = 0; i < to_copy; ++i) {
            temp_buffer[i] = messages[read_index];
            read_index = (read_index + 1) % MAX_PACKETS_IN_PSRAM;
        }
        pending_count -= to_copy;

        // release mutex immediately after copy to allow other tasks to insert logs
        free_mutex();

        // send the block of messages
        for (uint32_t i = 0; i < to_copy; ++i)
            send_callback_(reinterpret_cast<const uint8_t*>(&temp_buffer[i]), sizeof(message));
    }
}