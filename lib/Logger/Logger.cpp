#include <Logger.h>

#include "esp_heap_caps.h"
#include "esp_timer.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

Logger::Logger()
    : send_callback_(defaultSendCallback) {
    storage_ = static_cast<uint8_t*>(heap_caps_malloc(
        storage_capacity_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

    if (storage_ == nullptr) {
        printf("CRITICAL: Failed to allocate %u bytes in PSRAM!\n",
               static_cast<unsigned>(storage_capacity_));
    }
}

Logger::~Logger() {
    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }

    if (storage_ != nullptr) {
        heap_caps_free(storage_);
        storage_ = nullptr;
    }
}

void Logger::begin() {
    // Verify if the logger is already initialized to avoid creating multiple mutexes.
    if (initialized_) return;

    initialized_ = true;

    if (mutex_ == nullptr) mutex_ = xSemaphoreCreateMutex();
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
    if (mutex_ != nullptr) xSemaphoreGive(mutex_);
}

void Logger::set_send_callback(SendCallback callback) {
    if (!wait_for_mutex()) return;

    send_callback_ = (callback != nullptr) ? callback : defaultSendCallback;

    free_mutex();
}

bool Logger::defaultSendCallback(const uint8_t *data, size_t len) {
    (void)data;
    (void)len;

    return false;
}

void Logger::insert_logf(logType type, const char* format, ...) {
    char buffer[256];

    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (!wait_for_mutex()) return;

    if (len > 0) {
        const size_t final_len = (len < static_cast<int>(sizeof(buffer)))
                                     ? static_cast<size_t>(len)
                                     : sizeof(buffer) - 1;

        insert_log_impl(reinterpret_cast<const uint8_t*>(buffer), final_len, type,
                        static_cast<uint32_t>(esp_timer_get_time() / 1000ULL));
    }

    free_mutex();
}

void Logger::insert_log(logType type, const char* msg) {
    if (msg == nullptr) return;

    if (!wait_for_mutex()) return;

    insert_log_impl(reinterpret_cast<const uint8_t*>(msg), strlen(msg), type,
                    static_cast<uint32_t>(esp_timer_get_time() / 1000ULL));

    free_mutex();
}

uint8_t Logger::calculate_checksum(const message& msg) {
    uint32_t sum = 0;
    const size_t len = msg.content.size;

    for (size_t i = 0; i < len; ++i) {
        sum += static_cast<uint8_t>(msg.content.byte[i]);
    }

    sum += static_cast<uint8_t>(msg.type);
    sum += static_cast<uint16_t>(msg.packet_number);
    sum += static_cast<uint16_t>(msg.total_packets);

    return static_cast<uint8_t>(sum % 256);
}

void Logger::insert_log_impl(const uint8_t* data, size_t len, logType type, uint32_t ts) {
    if (data == nullptr || len == 0 || storage_ == nullptr) return;

    // The compact record uses 16 bits for its payload length. This is much larger
    // than insert_logf's formatted buffer and avoids one accidental huge string
    // monopolizing the complete log buffer.
    const uint16_t stored_length = static_cast<uint16_t>(
        len > 0xFFFFU ? 0xFFFFU : len);
    const uint32_t required = sizeof(StoredLogHeader) + stored_length;

    if (required > storage_capacity_) return;

    // Drop complete old records until the new one fits. A record being flushed
    // stays protected because its payload is read fragment by fragment.
    while ((storage_capacity_ - used_bytes_) < required) {
        if (flush_record_active_ || !discard_oldest_record()) return;
    }

    const StoredLogHeader header{
        .timer = ts,
        .length = stored_length,
        .type = type,
        .reserved = 0,
    };

    write_to_ring(&header, sizeof(header));
    write_to_ring(data, stored_length);
    used_bytes_ += required;
}

void Logger::write_to_ring(const void* source, size_t len) {
    if (len == 0) return;

    const auto* bytes = static_cast<const uint8_t*>(source);
    const size_t contiguous = storage_capacity_ - write_offset_;
    const size_t first = (len < contiguous) ? len : contiguous;

    memcpy(storage_ + write_offset_, bytes, first);
    if (len > first) memcpy(storage_, bytes + first, len - first);

    write_offset_ = (write_offset_ + len) % storage_capacity_;
}

void Logger::read_from_ring(uint32_t offset, void* destination, size_t len) const {
    if (len == 0) return;

    auto* bytes = static_cast<uint8_t*>(destination);
    const size_t contiguous = storage_capacity_ - offset;
    const size_t first = (len < contiguous) ? len : contiguous;

    memcpy(bytes, storage_ + offset, first);
    if (len > first) memcpy(bytes + first, storage_, len - first);
}

bool Logger::peek_oldest_header(StoredLogHeader& header) const {
    const uint32_t used = used_bytes_;
    if (used < sizeof(StoredLogHeader)) return false;

    read_from_ring(read_offset_, &header, sizeof(header));
    const uint32_t record_size = sizeof(StoredLogHeader) + header.length;

    return header.length > 0 && record_size <= used && record_size <= storage_capacity_;
}

bool Logger::discard_oldest_record() {
    StoredLogHeader header{};
    if (!peek_oldest_header(header)) {
        clear_ring();
        return false;
    }

    const uint32_t record_size = sizeof(StoredLogHeader) + header.length;
    read_offset_ = (read_offset_ + record_size) % storage_capacity_;
    used_bytes_ -= record_size;
    return true;
}

void Logger::clear_ring() {
    write_offset_ = 0;
    read_offset_ = 0;
    used_bytes_ = 0;
    flush_record_active_ = false;
    flush_header_ = {};
    flush_packet_index_ = 0;
}

void Logger::reset_loop_counter() {
    if (!wait_for_mutex()) return;

    clear_ring();
    free_mutex();
}

void Logger::flush_logs() {
    const uint32_t max_packets = MAX_CHUNKS_PER_FLUSH * BLOCK_SIZE;

    for (uint32_t sent_packets = 0; sent_packets < max_packets; ++sent_packets) {
        message tx{};
        SendCallback callback = defaultSendCallback;
        uint16_t total_packets = 0;

        // Protect the record and copy just the next fixed-size transport frame.
        if (!wait_for_mutex()) return;

        if (storage_ == nullptr || used_bytes_ == 0) {
            free_mutex();
            return;
        }

        if (!flush_record_active_) {
            if (!peek_oldest_header(flush_header_)) {
                clear_ring();
                free_mutex();
                return;
            }

            flush_record_active_ = true;
            flush_packet_index_ = 0;
        }

        total_packets = static_cast<uint16_t>(
            (flush_header_.length + MAX_CONTENT_SIZE - 1) / MAX_CONTENT_SIZE);
        const uint32_t payload_offset = flush_packet_index_ * MAX_CONTENT_SIZE;
        const uint32_t remaining = flush_header_.length - payload_offset;
        const uint16_t fragment_length = static_cast<uint16_t>(
            remaining < MAX_CONTENT_SIZE ? remaining : MAX_CONTENT_SIZE);
        const uint32_t ring_payload_offset =
            (read_offset_ + sizeof(StoredLogHeader) + payload_offset) % storage_capacity_;

        tx.timer = flush_header_.timer;
        tx.type = flush_header_.type;
        tx.packet_number = flush_packet_index_ + 1;
        tx.total_packets = total_packets;
        tx.content.size = fragment_length;
        memset(tx.content.byte, 0, sizeof(tx.content.byte));
        read_from_ring(ring_payload_offset, tx.content.byte, fragment_length);
        tx.checksum = calculate_checksum(tx);
        callback = send_callback_;

        free_mutex();

        // A rejected transport frame remains at the head for the next retry.
        if (!callback(reinterpret_cast<const uint8_t*>(&tx), sizeof(tx))) return;

        // Commit the accepted fragment. Waiting here avoids resending a fragment
        // merely because a producer held the mutex for a couple of milliseconds.
        if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) return;

        // A future public reset could clear the protected record while the send
        // callback is running.
        if (!flush_record_active_) {
            free_mutex();
            return;
        }

        ++flush_packet_index_;
        if (flush_packet_index_ >= total_packets) {
            const uint32_t record_size = sizeof(StoredLogHeader) + flush_header_.length;
            read_offset_ = (read_offset_ + record_size) % storage_capacity_;
            used_bytes_ -= record_size;
            flush_record_active_ = false;
            flush_header_ = {};
            flush_packet_index_ = 0;
        }

        free_mutex();
    }
}
