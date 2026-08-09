#include <Logger.h>
#include <SDCard.h>
#include <TinyShell.h>
#include <RobotSettings.h>
#include <Format.h>
#include <BtpTransport.h>

#include <btp/codec.hpp>

#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/task.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <sys/time.h>

Logger::Logger() {
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

void Logger::configure_btp(BtpEndpoint& endpoint) {
    if (!wait_for_mutex()) return;
    endpoint_ = &endpoint;
    free_mutex();
}

void Logger::set_flush_limits(uint32_t max_chunks_per_flush, uint32_t block_size) {
    if (!wait_for_mutex()) return;

    max_chunks_per_flush_ = max_chunks_per_flush;
    block_size_ = block_size;

    free_mutex();
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
                        static_cast<uint64_t>(esp_timer_get_time()));
    }

    free_mutex();
}

void Logger::insert_log(logType type, const char* msg) {
    if (msg == nullptr) return;

    if (!wait_for_mutex()) return;

    insert_log_impl(reinterpret_cast<const uint8_t*>(msg), strlen(msg), type,
                    static_cast<uint64_t>(esp_timer_get_time()));

    free_mutex();
}

bool Logger::send_log_direct(logType type, const char* msg) {
    if (msg == nullptr || msg[0] == '\0') return false;
    BtpEndpoint* endpoint = nullptr;
    if (!wait_for_mutex()) return false;
    endpoint = endpoint_;
    free_mutex();
    if (endpoint == nullptr) return false;

    return endpoint->send_logical(
        btp::MessageType::Log, static_cast<uint16_t>(type),
        reinterpret_cast<const uint8_t*>(msg), strlen(msg),
        static_cast<uint64_t>(esp_timer_get_time()));
}

void Logger::insert_log_impl(const uint8_t* data, size_t len, logType type,
                             uint64_t timestamp_us) {
    if (data == nullptr || len == 0 || storage_ == nullptr ||
        endpoint_ == nullptr) return;

    // The compact record uses 16 bits for its payload length. This is much larger
    // than insert_logf's formatted buffer and avoids one accidental huge string
    // monopolizing the complete log buffer.
    constexpr size_t kMaxLogicalLogSize =
        btp::kEspNowMaxPayloadSize * 255U;
    const uint16_t stored_length = static_cast<uint16_t>(
        len > kMaxLogicalLogSize ? kMaxLogicalLogSize : len);
    const uint32_t required = sizeof(StoredLogHeader) + stored_length;

    if (required > storage_capacity_) return;

    // Only an SD flush is allowed to release retained PSRAM. When the ring is
    // full, reject the new log instead of deleting an older unsaved record.
    if ((storage_capacity_ - used_bytes_) < required) return;

    uint32_t sequence = 0U;
    if (!endpoint_->reserve_sequence(&sequence)) return;

    const StoredLogHeader header{
        .timestamp_us = timestamp_us,
        .sequence = sequence,
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

    const uint32_t max_packets = max_chunks_per_flush_ * block_size_;

    for (uint32_t sent_packets = 0; sent_packets < max_packets; ++sent_packets) {
        uint8_t payload[btp::kEspNowMaxPayloadSize];
        uint8_t fragment_index = 0U;
        uint8_t fragment_count = 0U;
        uint16_t fragment_length = 0U;
        StoredLogHeader header{};
        BtpEndpoint* endpoint = nullptr;

        // Copy only the next fragment from PSRAM. The frame itself is encoded
        // into a 250-byte stack buffer by BtpEndpoint and sent at its real size.
        if (!wait_for_mutex()) break;

        if (storage_ == nullptr || pending_send_bytes_ == 0 || endpoint_ == nullptr) {
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

        fragment_count = static_cast<uint8_t>(
            (send_header_.length + btp::kEspNowMaxPayloadSize - 1U) /
            btp::kEspNowMaxPayloadSize);
        const uint32_t payload_offset =
            send_packet_index_ * btp::kEspNowMaxPayloadSize;
        const uint32_t remaining = send_header_.length - payload_offset;
        fragment_length = static_cast<uint16_t>(
            remaining < btp::kEspNowMaxPayloadSize
                ? remaining
                : btp::kEspNowMaxPayloadSize);
        const uint32_t ring_payload_offset =
            (send_offset_ + sizeof(StoredLogHeader) + payload_offset) % storage_capacity_;

        header = send_header_;
        fragment_index = send_packet_index_;
        endpoint = endpoint_;
        read_from_ring(ring_payload_offset, payload, fragment_length);

        free_mutex();

        // A rejected frame keeps the packet cursor unchanged for the next call.
        if (!endpoint->send_fragment(
                btp::MessageType::Log, static_cast<uint16_t>(header.type),
                header.sequence, header.timestamp_us, payload, fragment_length,
                fragment_index, fragment_count)) {
            break;
        }

        if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) break;

        if (!send_record_active_) {
            free_mutex();
            break;
        }

        ++send_packet_index_;
        if (send_packet_index_ >= fragment_count) {
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

        const uint8_t fragment_count = static_cast<uint8_t>(
            (header.length + btp::kEspNowMaxPayloadSize - 1U) /
            btp::kEspNowMaxPayloadSize);

        for (uint8_t packet = 0; packet < fragment_count; ++packet) {
            uint8_t payload[btp::kEspNowMaxPayloadSize];
            uint8_t frame[btp::kEspNowMaxFrameSize];
            size_t frame_size = 0U;
            const uint32_t payload_offset =
                packet * btp::kEspNowMaxPayloadSize;
            const uint32_t remaining = header.length - payload_offset;
            const uint16_t fragment_length = static_cast<uint16_t>(
                remaining < btp::kEspNowMaxPayloadSize
                    ? remaining
                    : btp::kEspNowMaxPayloadSize);
            const uint32_t ring_payload_offset =
                (record_offset + sizeof(StoredLogHeader) + payload_offset) %
                storage_capacity_;

            if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
                success = false;
                break;
            }
            read_from_ring(ring_payload_offset, payload, fragment_length);
            BtpEndpoint* endpoint = endpoint_;
            free_mutex();

            if (endpoint == nullptr ||
                !endpoint->encode_fragment(
                    btp::MessageType::Log, static_cast<uint16_t>(header.type),
                    header.sequence, header.timestamp_us, payload,
                    fragment_length, packet, fragment_count, frame,
                    sizeof(frame), &frame_size) ||
                !callback(frame, frame_size, context)) {
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

bool Logger::set_datetime(uint16_t year, uint8_t month, uint8_t day,
                          uint8_t hour, uint8_t minute, uint8_t second,
                          const char* posix_tz) {
    if (year < 2020 || month < 1 || month > 12 || day < 1 || day > 31 ||
        hour > 23 || minute > 59 || second > 59) {
        return false;
    }

    setenv("TZ", posix_tz, 1);
    tzset();

    struct tm requested_time{};
    requested_time.tm_year = year - 1900;
    requested_time.tm_mon = month - 1;
    requested_time.tm_mday = day;
    requested_time.tm_hour = hour;
    requested_time.tm_min = minute;
    requested_time.tm_sec = second;
    requested_time.tm_isdst = -1;

    const time_t timestamp = mktime(&requested_time);
    if (timestamp == static_cast<time_t>(-1)) return false;

    // mktime normalizes invalid dates, such as 31 February. Compare the result
    // to reject those values instead of silently changing the requested date.
    struct tm verified_time{};
    localtime_r(&timestamp, &verified_time);
    if (verified_time.tm_year != year - 1900 ||
        verified_time.tm_mon != month - 1 ||
        verified_time.tm_mday != day ||
        verified_time.tm_hour != hour ||
        verified_time.tm_min != minute ||
        verified_time.tm_sec != second) {
        return false;
    }

    const struct timeval system_time{
        .tv_sec = timestamp,
        .tv_usec = 0,
    };

    if (settimeofday(&system_time, nullptr) != 0) return false;

    clock_synchronized_ = true;
    return true;
}

bool Logger::make_log_filename(SDCard& card, char* filename, size_t capacity) const {
    if (!clock_synchronized_ || filename == nullptr || capacity == 0) return false;

    const time_t timestamp = time(nullptr);
    struct tm local_time{};
    if (localtime_r(&timestamp, &local_time) == nullptr) return false;

    int written = snprintf(
        filename, capacity, "log_%04d-%02d-%02d_%02d-%02d-%02d.blog",
        local_time.tm_year + 1900, local_time.tm_mon + 1, local_time.tm_mday,
        local_time.tm_hour, local_time.tm_min, local_time.tm_sec);

    if (written <= 0 || static_cast<size_t>(written) >= capacity) return false;
    if (!card.file_exists(filename)) return true;

    // Do not overwrite a log when two new flushes happen in the same second.
    for (uint8_t suffix = 1; suffix < 100; ++suffix) {
        written = snprintf(
            filename, capacity,
            "log_%04d-%02d-%02d_%02d-%02d-%02d_%02u.blog",
            local_time.tm_year + 1900, local_time.tm_mon + 1,
            local_time.tm_mday, local_time.tm_hour, local_time.tm_min,
            local_time.tm_sec, suffix);

        if (written > 0 && static_cast<size_t>(written) < capacity &&
            !card.file_exists(filename)) {
            return true;
        }
    }

    return false;
}

bool Logger::find_latest_log_file(SDCard& card, char* filename, size_t capacity) const {
    if (filename == nullptr || capacity == 0) return false;
    filename[0] = '\0';

    const uint16_t file_count = card.get_file_count();
    for (uint16_t index = 0; index < file_count; ++index) {
        SDFileInfo info{};
        if (!card.get_file_info(index, info)) continue;

        const size_t name_length = strlen(info.name);
        if (name_length < 10 || strncmp(info.name, "log_", 4) != 0 ||
            strcmp(info.name + name_length - 5, ".blog") != 0) {
            continue;
        }

        // ISO date/time in the filename makes lexical order chronological.
        if (filename[0] == '\0' || strcmp(info.name, filename) > 0) {
            const int written = snprintf(filename, capacity, "%s", info.name);
            if (written <= 0 || static_cast<size_t>(written) >= capacity) {
                filename[0] = '\0';
                return false;
            }
        }
    }

    return filename[0] != '\0';
}

bool Logger::flush_to_sd(SDCard& card, bool append,
                         char* out_filename, size_t out_capacity) {
    if (!card.is_mounted()) return false;

    char filename[kLogFilenameCapacity];

    if (append) {
        if (last_log_file_[0] != '\0' && card.file_exists(last_log_file_)) {
            snprintf(filename, sizeof(filename), "%s", last_log_file_);
        } else if (!find_latest_log_file(card, filename, sizeof(filename))) {
            return false;
        }
    } else if (!make_log_filename(card, filename, sizeof(filename))) {
        return false;
    }

    if (!card.open_write_stream(filename, append)) return false;

    const bool stored = flush_logs_to(
        [](const uint8_t* data, size_t length, void* context) -> bool {
            auto* out_card = static_cast<SDCard*>(context);
            return out_card->write_stream(data, length);
        },
        &card);

    const bool closed = card.close_stream();

    if (!append || last_log_file_[0] == '\0') {
        snprintf(last_log_file_, sizeof(last_log_file_), "%s", filename);
    }

    if (out_filename != nullptr && out_capacity > 0) {
        snprintf(out_filename, out_capacity, "%s", filename);
    }

    return stored && closed;
}

// ============================================================================
// Shell commands
// ============================================================================

void Logger::register_shell_commands(TinyShell& shell, SDCard& sd_card, RobotSettings& settings,
                                     std::function<void()> mark_direct_output) {
    shell.create_module("logger", "Retained PSRAM log buffer and SD log files");

    shell.add([this]() -> uint8_t {
        // Envia um texto longo numerado para testar fragmentacao e perda de pacotes
        // Referencia: Terminal log da GLaDOS (Portal) - "Still Alive"
        const char* long_text =
            "[01/14] Forms FORM-29827281-12-2: Notice of Dismissal\n"
            "[02/14] Aperture Science computer-aided enrichment center\n"
            "[03/14] This was a triumph.\n"
            "[04/14] I'm making a note here: HUGE SUCCESS.\n"
            "[05/14] It's hard to overstate my satisfaction.\n"
            "[06/14] Aperture Science\n"
            "[07/14] We do what we must because we can.\n"
            "[08/14] For the good of all of us.\n"
            "[09/14] Except the ones who are dead.\n"
            "[10/14] But there's no sense crying over every mistake.\n"
            "[11/14] You just keep on trying till you run out of cake.\n"
            "[12/14] And the Science gets done.\n"
            "[13/14] And you make a neat gun.\n"
            "[14/14] For the people who are still alive.\n";

        // When info logs are enabled, push to logger to validate its packetization path.
        #if defined(LOG_ALL) || defined(LOG_INFO)
            insert_log(logType::INFO, long_text);
            return RESULT_OK;
        #endif
    }, "test_packet", "Send a long test packet to evaluate multi-packet handling", "logger");

    shell.add([this]() -> uint8_t {
        char used_text[24];
        char capacity_text[24];
        formatBytes(get_used_bytes(), used_text, sizeof(used_text));
        formatBytes(get_capacity_bytes(), capacity_text, sizeof(capacity_text));

        insert_logf(
            logType::INFO,
            "PSRAM logger used=%s capacity=%s (%.2f%%)",
            used_text,
            capacity_text,
            get_write_pct());
        return RESULT_OK;
    }, "psram_usage", "Show retained Logger PSRAM usage", "logger");

    shell.add([this, &settings](uint16_t year, uint8_t month, uint8_t day,
                 uint8_t hour, uint8_t minute, uint8_t second) -> uint8_t {
        if (!set_datetime(year, month, day, hour, minute, second,
                          settings.data().timezone)) {
            insert_log(logType::ERRO,
                       "Invalid date/time or clock update failed");
            return RESULT_ERROR;
        }

        insert_logf(
            logType::INFO,
            "Robot time updated: %04u-%02u-%02u %02u:%02u:%02u",
            year, month, day, hour, minute, second);
        return RESULT_OK;
    }, "set_datetime", "Set local time: year,month,day,hour,minute,second",
       "logger");

    shell.add([this, &sd_card, mark_direct_output]() -> uint8_t {
        char filename[SDFileInfo::MAX_NAME_LENGTH] = {};
        if (!flush_to_sd(sd_card, false, filename, sizeof(filename))) {
            insert_log(
                logType::ERRO,
                "Failed to create a new SD log; synchronize date/time first");
            return RESULT_ERROR;
        }

        char response[sizeof(filename) + 32];
        snprintf(response, sizeof(response), "PSRAM saved to %s", filename);
        send_log_direct(logType::INFO, response);
        mark_direct_output();
        return RESULT_OK;
    }, "flush_new", "Save retained PSRAM logs into a new dated file", "logger");

    shell.add([this, &sd_card, mark_direct_output]() -> uint8_t {
        char filename[SDFileInfo::MAX_NAME_LENGTH] = {};
        if (!flush_to_sd(sd_card, true, filename, sizeof(filename))) {
            insert_log(
                logType::ERRO,
                "Failed to append PSRAM logs to the latest SD log");
            return RESULT_ERROR;
        }

        char response[sizeof(filename) + 36];
        snprintf(response, sizeof(response), "PSRAM appended to %s", filename);
        send_log_direct(logType::INFO, response);
        mark_direct_output();
        return RESULT_OK;
    }, "flush_append", "Append retained PSRAM logs to the latest log file",
       "logger");

    shell.add([this, &sd_card](uint16_t file_index, uint32_t delay_msg_ms) -> uint8_t {
        SDFileInfo info{};
        if (!sd_card.get_file_info(file_index, info) ||
            !sd_card.open_read_stream(info.name)) {
            insert_log(logType::ERRO,
                       "Invalid log index or file format");
            return RESULT_ERROR;
        }

        bool success = true;
        uint32_t sent_messages = 0;

        while (true) {
            uint8_t frame[btp::kEspNowMaxFrameSize];
            const size_t header_read =
                sd_card.read_stream(frame, btp::kV1HeaderSize);

            if (header_read == 0) break;
            if (header_read != btp::kV1HeaderSize) {
                success = false;
                break;
            }

            const size_t payload_size =
                static_cast<size_t>(frame[10]) |
                (static_cast<size_t>(frame[11]) << 8U);
            const size_t tail_size = payload_size + btp::kV1CrcSize;
            const size_t frame_size = btp::kV1HeaderSize + tail_size;
            if (frame_size > sizeof(frame) ||
                sd_card.read_stream(frame + btp::kV1HeaderSize, tail_size) !=
                    tail_size) {
                success = false;
                break;
            }

            btp::DecodedFrame decoded{};
            if (btp::decode(frame, frame_size,
                            btp::TransportProfile::EspNow,
                            &decoded) != btp::Error::Ok ||
                decoded.header.type != btp::MessageType::Log) {
                success = false;
                break;
            }

            BtpEndpoint* endpoint = nullptr;
            if (wait_for_mutex()) {
                endpoint = endpoint_;
                free_mutex();
            }
            if (endpoint == nullptr || !endpoint->send_encoded(frame, frame_size)) {
                success = false;
                break;
            }

            ++sent_messages;
            if (delay_msg_ms > 0) {
                TickType_t delay_ticks = pdMS_TO_TICKS(delay_msg_ms);
                if (delay_ticks == 0) delay_ticks = 1;
                vTaskDelay(delay_ticks);
            }
        }

        if (sd_card.stream_has_error()) success = false;
        if (!sd_card.close_stream()) success = false;

        insert_logf(
            success ? logType::INFO : logType::ERRO,
            "Log playback %s: %u messages from %s",
            success ? "finished" : "failed", sent_messages, info.name);

        return success ? RESULT_OK : RESULT_ERROR;
    }, "print_log", "Replay file by index: file_index,delay_msg_ms", "logger");
}
