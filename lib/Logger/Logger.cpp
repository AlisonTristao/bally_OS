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

bool Logger::send_log_direct(logType type, const char* msg) {
    if (msg == nullptr || msg[0] == '\0') return false;

    const size_t length = strlen(msg);
    const uint16_t total_packets = static_cast<uint16_t>(
        (length + MAX_CONTENT_SIZE - 1) / MAX_CONTENT_SIZE);
    const uint32_t timestamp = static_cast<uint32_t>(
        esp_timer_get_time() / 1000ULL);

    for (uint16_t packet = 0; packet < total_packets; ++packet) {
        message direct_message{};
        const size_t offset = packet * MAX_CONTENT_SIZE;
        const size_t remaining = length - offset;
        const size_t fragment_length =
            remaining < MAX_CONTENT_SIZE ? remaining : MAX_CONTENT_SIZE;

        direct_message.timer = timestamp;
        direct_message.type = type;
        direct_message.packet_number = packet + 1;
        direct_message.total_packets = total_packets;
        direct_message.content.size = fragment_length;
        memcpy(direct_message.content.byte, msg + offset, fragment_length);
        direct_message.checksum = calculate_checksum(direct_message);

        if (!send_message(direct_message)) return false;
    }

    return true;
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

    // Only an SD flush is allowed to release retained PSRAM. When the ring is
    // full, reject the new log instead of deleting an older unsaved record.
    if ((storage_capacity_ - used_bytes_) < required) return;

    const StoredLogHeader header{
        .timer = ts,
        .length = stored_length,
        .type = type,
        .reserved = 0,
    };

    write_to_ring(&header, sizeof(header));
    write_to_ring(data, stored_length);
    used_bytes_ += required;
    pending_send_bytes_ += required;
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

bool Logger::read_header_at(uint32_t offset, uint32_t available,
                            StoredLogHeader& header) const {
    if (available < sizeof(StoredLogHeader)) return false;

    read_from_ring(offset, &header, sizeof(header));
    const uint32_t record_size = sizeof(StoredLogHeader) + header.length;

    return header.length > 0 && record_size <= available &&
           record_size <= storage_capacity_;
}

void Logger::clear_ring() {
    write_offset_ = 0;
    read_offset_ = 0;
    used_bytes_ = 0;
    send_offset_ = 0;
    pending_send_bytes_ = 0;
    send_record_active_ = false;
    send_header_ = {};
    send_packet_index_ = 0;
    consumer_busy_ = false;
}

bool Logger::begin_consumer() {
    if (!wait_for_mutex()) return false;

    if (consumer_busy_) {
        free_mutex();
        return false;
    }

    consumer_busy_ = true;
    free_mutex();
    return true;
}

void Logger::end_consumer() {
    if (mutex_ == nullptr) return;

    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
        consumer_busy_ = false;
        free_mutex();
    }
}

void Logger::reset_loop_counter() {
    if (!wait_for_mutex()) return;

    clear_ring();
    free_mutex();
}

void Logger::flush_logs() {
    if (!begin_consumer()) return;

    const uint32_t max_packets = MAX_CHUNKS_PER_FLUSH * BLOCK_SIZE;

    for (uint32_t sent_packets = 0; sent_packets < max_packets; ++sent_packets) {
        message tx{};
        SendCallback callback = defaultSendCallback;
        uint16_t total_packets = 0;

        // Copy just the next fixed-size transport frame.
        if (!wait_for_mutex()) break;

        if (storage_ == nullptr || pending_send_bytes_ == 0) {
            free_mutex();
            break;
        }

        if (!send_record_active_) {
            if (!read_header_at(send_offset_, pending_send_bytes_, send_header_)) {
                free_mutex();
                break;
            }

            send_record_active_ = true;
            send_packet_index_ = 0;
        }

        total_packets = static_cast<uint16_t>(
            (send_header_.length + MAX_CONTENT_SIZE - 1) / MAX_CONTENT_SIZE);
        const uint32_t payload_offset = send_packet_index_ * MAX_CONTENT_SIZE;
        const uint32_t remaining = send_header_.length - payload_offset;
        const uint16_t fragment_length = static_cast<uint16_t>(
            remaining < MAX_CONTENT_SIZE ? remaining : MAX_CONTENT_SIZE);
        const uint32_t ring_payload_offset =
            (send_offset_ + sizeof(StoredLogHeader) + payload_offset) % storage_capacity_;

        tx.timer = send_header_.timer;
        tx.type = send_header_.type;
        tx.packet_number = send_packet_index_ + 1;
        tx.total_packets = total_packets;
        tx.content.size = fragment_length;
        memset(tx.content.byte, 0, sizeof(tx.content.byte));
        read_from_ring(ring_payload_offset, tx.content.byte, fragment_length);
        tx.checksum = calculate_checksum(tx);
        callback = send_callback_;

        free_mutex();

        // A rejected frame keeps the packet cursor unchanged for the next call.
        if (!callback(reinterpret_cast<const uint8_t*>(&tx), sizeof(tx))) break;

        if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) break;

        if (!send_record_active_) {
            free_mutex();
            break;
        }

        ++send_packet_index_;
        if (send_packet_index_ >= total_packets) {
            const uint32_t record_size = sizeof(StoredLogHeader) + send_header_.length;
            send_offset_ = (send_offset_ + record_size) % storage_capacity_;
            pending_send_bytes_ -= record_size;
            send_record_active_ = false;
            send_header_ = {};
            send_packet_index_ = 0;
        }

        free_mutex();
    }

    end_consumer();
}

bool Logger::flush_logs_to(StorageCallback callback, void* context) {
    if (callback == nullptr || !begin_consumer()) return false;

    bool success = true;
    uint32_t bytes_remaining = 0;

    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
        end_consumer();
        return false;
    }

    bytes_remaining = used_bytes_;
    free_mutex();

    // Save only the snapshot that existed when the flush started. Logs inserted
    // during this operation remain retained for the next SD flush.
    while (bytes_remaining > 0) {
        StoredLogHeader header{};

        if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
            success = false;
            break;
        }

        if (!read_header_at(read_offset_, bytes_remaining, header)) {
            free_mutex();
            success = false;
            break;
        }

        const uint32_t record_offset = read_offset_;
        free_mutex();

        const uint16_t total_packets = static_cast<uint16_t>(
            (header.length + MAX_CONTENT_SIZE - 1) / MAX_CONTENT_SIZE);

        for (uint16_t packet = 0; packet < total_packets; ++packet) {
            message stored_message{};
            const uint32_t payload_offset = packet * MAX_CONTENT_SIZE;
            const uint32_t remaining = header.length - payload_offset;
            const uint16_t fragment_length = static_cast<uint16_t>(
                remaining < MAX_CONTENT_SIZE ? remaining : MAX_CONTENT_SIZE);
            const uint32_t ring_payload_offset =
                (record_offset + sizeof(StoredLogHeader) + payload_offset) %
                storage_capacity_;

            stored_message.timer = header.timer;
            stored_message.type = header.type;
            stored_message.packet_number = packet + 1;
            stored_message.total_packets = total_packets;
            stored_message.content.size = fragment_length;
            memset(stored_message.content.byte, 0,
                   sizeof(stored_message.content.byte));

            if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
                success = false;
                break;
            }
            read_from_ring(ring_payload_offset, stored_message.content.byte,
                           fragment_length);
            free_mutex();

            stored_message.checksum = calculate_checksum(stored_message);
            if (!callback(reinterpret_cast<const uint8_t*>(&stored_message),
                          sizeof(stored_message), context)) {
                success = false;
                break;
            }
        }

        if (!success) break;

        if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
            success = false;
            break;
        }

        const uint32_t record_size = sizeof(StoredLogHeader) + header.length;

        // If the oldest retained record has not gone through ESP-NOW yet, the SD
        // flush removes it from both cursor ranges.
        if (pending_send_bytes_ == used_bytes_) {
            send_offset_ = (send_offset_ + record_size) % storage_capacity_;
            pending_send_bytes_ -= record_size;
        }

        read_offset_ = (read_offset_ + record_size) % storage_capacity_;
        used_bytes_ -= record_size;
        bytes_remaining -= record_size;
        free_mutex();
    }

    end_consumer();
    return success;
}

bool Logger::send_message(const message& msg) {
    if (msg.content.size > MAX_CONTENT_SIZE || !wait_for_mutex()) return false;

    const SendCallback callback = send_callback_;
    free_mutex();

    return callback(reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}
