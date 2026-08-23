#include <CommandProcessor.h>

#include <cstring>

void CommandProcessor::configure(BtpEndpoint& endpoint, BtpSealFn seal,
                                 void* seal_context) noexcept {
    endpoint_ = &endpoint;
    seal_ = seal;
    seal_context_ = seal_context;
}

bool CommandProcessor::same_key(const RequestKey& left,
                                const RequestKey& right) noexcept {
    return left.source_id == right.source_id && left.boot_id == right.boot_id &&
           left.sequence == right.sequence;
}

void CommandProcessor::write_u16(std::uint8_t* output,
                                 std::uint16_t value) noexcept {
    output[0] = static_cast<std::uint8_t>(value);
    output[1] = static_cast<std::uint8_t>(value >> 8U);
}

void CommandProcessor::write_u32(std::uint8_t* output,
                                 std::uint32_t value) noexcept {
    output[0] = static_cast<std::uint8_t>(value);
    output[1] = static_cast<std::uint8_t>(value >> 8U);
    output[2] = static_cast<std::uint8_t>(value >> 16U);
    output[3] = static_cast<std::uint8_t>(value >> 24U);
}

std::uint16_t CommandProcessor::read_u16(const std::uint8_t* input) noexcept {
    return static_cast<std::uint16_t>(input[0]) |
           static_cast<std::uint16_t>(
               static_cast<std::uint16_t>(input[1]) << 8U);
}

const char* CommandProcessor::parse_message(
    btp_command::ParseError error) noexcept {
    switch (error) {
        case btp_command::ParseError::WrongTarget: return "stale command target";
        case btp_command::ParseError::UnsupportedAction: return "unsupported action";
        case btp_command::ParseError::InvalidAction: return "invalid action";
        case btp_command::ParseError::ParametersTooLarge: return "parameters too large";
        case btp_command::ParseError::InvalidShellText: return "invalid shell command";
        default: return "malformed command request";
    }
}

CommandProcessor::Status CommandProcessor::parse_status(
    btp_command::ParseError error) noexcept {
    return error == btp_command::ParseError::UnsupportedAction
               ? Status::Unsupported
               : Status::Rejected;
}

CommandProcessor::ErrorCode CommandProcessor::parse_error_code(
    btp_command::ParseError error) noexcept {
    switch (error) {
        case btp_command::ParseError::WrongTarget:
            return ErrorCode::StaleTargetBoot;
        case btp_command::ParseError::UnsupportedAction:
            return ErrorCode::UnsupportedVersion;
        case btp_command::ParseError::InvalidAction:
        case btp_command::ParseError::ParametersTooLarge:
        case btp_command::ParseError::InvalidShellText:
        case btp_command::ParseError::OutputTooSmall:
            return ErrorCode::InvalidArgument;
        default:
            return ErrorCode::MalformedPayload;
    }
}

CommandProcessor::ResultView CommandProcessor::view(
    const CacheEntry& entry) const noexcept {
    return {
        entry.result_sequence,
        entry.result_timestamp_us,
        entry.result,
        entry.result_size,
    };
}

bool CommandProcessor::encode_result(
    const RequestKey& key, std::uint16_t action_id,
    std::uint16_t action_version, Status status, ErrorCode error,
    const char* message, std::uint8_t* output, std::size_t capacity,
    std::uint16_t* size_out) noexcept {
    if (output == nullptr || size_out == nullptr || message == nullptr) {
        return false;
    }
    const std::size_t message_size = std::strlen(message);
    // Fixed fields (20), utf8_u16 prefix (2), bytes_u32 prefix (4).
    const std::size_t total = 26U + message_size;
    if (message_size > 512U || total > capacity) return false;

    write_u32(output, key.source_id);
    write_u32(output + 4U, key.boot_id);
    write_u32(output + 8U, key.sequence);
    write_u16(output + 12U, action_id);
    write_u16(output + 14U, action_version);
    output[16U] = static_cast<std::uint8_t>(status);
    output[17U] = 0U;
    write_u16(output + 18U, static_cast<std::uint16_t>(error));
    write_u16(output + 20U, static_cast<std::uint16_t>(message_size));
    std::memcpy(output + 22U, message, message_size);
    write_u32(output + 22U + message_size, 0U);
    *size_out = static_cast<std::uint16_t>(total);
    return true;
}

bool CommandProcessor::finish(CacheEntry& entry, Status status,
                              ErrorCode error, const char* message,
                              std::uint64_t now_us,
                              ResultView* result_out) noexcept {
    if (endpoint_ == nullptr || result_out == nullptr || entry.complete) {
        return false;
    }
    std::uint32_t sequence = 0U;
    if (!endpoint_->reserve_sequence(&sequence) ||
        !encode_result(entry.key, entry.action_id, entry.action_version,
                       status, error, message, entry.result,
                       sizeof(entry.result), &entry.result_size)) {
        dropped_.fetch_add(1U, std::memory_order_relaxed);
        return false;
    }
    entry.result_sequence = sequence;
    entry.result_timestamp_us = now_us;
    entry.complete = true;
    *result_out = view(entry);
    return true;
}

bool CommandProcessor::make_transient(
    const RequestKey& key, std::uint16_t action_id,
    std::uint16_t action_version, Status status, ErrorCode error,
    const char* message, std::uint64_t now_us,
    ResultView* result_out) noexcept {
    if (endpoint_ == nullptr || result_out == nullptr ||
        !endpoint_->reserve_sequence(&transient_sequence_) ||
        !encode_result(key, action_id, action_version, status, error, message,
                       transient_result_, sizeof(transient_result_),
                       &transient_size_)) {
        dropped_.fetch_add(1U, std::memory_order_relaxed);
        return false;
    }
    transient_timestamp_us_ = now_us;
    *result_out = {transient_sequence_, transient_timestamp_us_,
                   transient_result_, transient_size_};
    return true;
}

CommandProcessor::Intake CommandProcessor::intake(
    const btp::Header& header, btp::ByteView payload,
    std::uint64_t now_us) noexcept {
    Intake intake{};
    // RX runs on the Wi-Fi task: never wait behind the shell task. A retry is
    // safe and will hit the same dedup key after the short critical section.
    if (cache_lock_.test_and_set(std::memory_order_acquire)) {
        note_drop();
        return intake;
    }
    struct Unlock {
        std::atomic_flag& lock;
        ~Unlock() { lock.clear(std::memory_order_release); }
    } unlock{cache_lock_};

    if (endpoint_ == nullptr || header.type != btp::MessageType::Command ||
        header.object_id != btp_command::kCommandRequestObjectId ||
        header.source_id == 0U || header.boot_id == 0U ||
        header.sequence == 0U || payload.data == nullptr ||
        payload.size > btp_command::kMaxLogicalRequestSize) {
        note_drop();
        return intake;
    }

    const RequestKey key{header.source_id, header.boot_id, header.sequence};
    std::uint16_t action_id = 0U;
    std::uint16_t action_version = 0U;
    if (payload.size >= 12U) {
        action_id = read_u16(payload.data + 8U);
        action_version = read_u16(payload.data + 10U);
    }

    CacheEntry* free_entry = nullptr;
    for (CacheEntry& entry : cache_) {
        if (!entry.used) {
            if (free_entry == nullptr) free_entry = &entry;
            continue;
        }
        if (!same_key(entry.key, key)) continue;

        duplicates_.fetch_add(1U, std::memory_order_relaxed);
        const bool identical = entry.request_size == payload.size &&
            std::memcmp(entry.request, payload.data, payload.size) == 0;
        if (!identical) {
            rejected_.fetch_add(1U, std::memory_order_relaxed);
            if (make_transient(key, action_id, action_version,
                               Status::Rejected, ErrorCode::RequestConflict,
                               "request identity conflict", now_us,
                               &intake.result)) {
                intake.kind = IntakeKind::ResultReady;
            }
            return intake;
        }
        if (!entry.complete) {
            intake.kind = IntakeKind::DuplicateInProgress;
            return intake;
        }
        replayed_.fetch_add(1U, std::memory_order_relaxed);
        intake.kind = IntakeKind::ResultReady;
        intake.result = view(entry);
        return intake;
    }

    if (free_entry == nullptr) {
        rejected_.fetch_add(1U, std::memory_order_relaxed);
        if (make_transient(key, action_id, action_version, Status::Busy,
                           ErrorCode::CapacityExhausted,
                           "command cache exhausted", now_us,
                           &intake.result)) {
            intake.kind = IntakeKind::ResultReady;
        }
        return intake;
    }

    free_entry->used = true;
    free_entry->complete = false;
    free_entry->key = key;
    free_entry->action_id = action_id;
    free_entry->action_version = action_version;
    free_entry->request_size = static_cast<std::uint16_t>(payload.size);
    std::memcpy(free_entry->request, payload.data, payload.size);
    const std::uint8_t slot = static_cast<std::uint8_t>(free_entry - cache_);

    btp_command::RequestView request{};
    btp_command::ParseError parse_error = btp_command::parse_request(
        header, payload, endpoint_->source_id(), endpoint_->boot_id(),
        &request);
    if (parse_error == btp_command::ParseError::Ok) {
        parse_error = btp_command::copy_shell_command(
            request, intake.work.command, sizeof(intake.work.command));
    }
    if (parse_error != btp_command::ParseError::Ok) {
        rejected_.fetch_add(1U, std::memory_order_relaxed);
        if (finish(*free_entry, parse_status(parse_error),
                   parse_error_code(parse_error), parse_message(parse_error),
                   now_us, &intake.result)) {
            intake.kind = IntakeKind::ResultReady;
        }
        return intake;
    }

    accepted_.fetch_add(1U, std::memory_order_relaxed);
    intake.work.cache_slot = slot;
    intake.kind = IntakeKind::Ready;
    return intake;
}

bool CommandProcessor::complete(std::uint8_t cache_slot,
                                std::uint8_t shell_status,
                                std::uint64_t now_us,
                                ResultView* result_out) noexcept {
    while (cache_lock_.test_and_set(std::memory_order_acquire)) {}
    struct Unlock {
        std::atomic_flag& lock;
        ~Unlock() { lock.clear(std::memory_order_release); }
    } unlock{cache_lock_};
    if (cache_slot >= kCacheCapacity || !cache_[cache_slot].used) return false;
    const bool success = shell_status == 0U;
    if (!finish(cache_[cache_slot],
                success ? Status::Success : Status::Failed,
                success ? ErrorCode::None : ErrorCode::InternalError,
                success ? "" : "shell command failed", now_us, result_out)) {
        return false;
    }
    executed_.fetch_add(1U, std::memory_order_relaxed);
    return true;
}

bool CommandProcessor::reject_busy(std::uint8_t cache_slot,
                                   std::uint64_t now_us,
                                   ResultView* result_out) noexcept {
    while (cache_lock_.test_and_set(std::memory_order_acquire)) {}
    struct Unlock {
        std::atomic_flag& lock;
        ~Unlock() { lock.clear(std::memory_order_release); }
    } unlock{cache_lock_};
    if (cache_slot >= kCacheCapacity || !cache_[cache_slot].used) return false;
    rejected_.fetch_add(1U, std::memory_order_relaxed);
    return finish(cache_[cache_slot], Status::Busy,
                  ErrorCode::CapacityExhausted,
                  "command execution queue full", now_us, result_out);
}

bool CommandProcessor::send_result(const ResultView& result) noexcept {
    if (endpoint_ == nullptr || result.sequence == 0U ||
        result.payload == nullptr || result.payload_size == 0U ||
        result.payload_size > btp::kEspNowMaxPayloadSize) {
        note_drop();
        return false;
    }
    const bool sent = endpoint_->send_fragment(
        btp::MessageType::Command, kCommandResultObjectId, result.sequence,
        result.timestamp_us, result.payload, result.payload_size, 0U, 1U,
        seal_, seal_context_);
    if (!sent) note_drop();
    return sent;
}

void CommandProcessor::note_unauthorized() noexcept {
    unauthorized_.fetch_add(1U, std::memory_order_relaxed);
}

void CommandProcessor::note_drop() noexcept {
    dropped_.fetch_add(1U, std::memory_order_relaxed);
}

CommandProcessor::Stats CommandProcessor::stats() const noexcept {
    return {
        accepted_.load(std::memory_order_relaxed),
        executed_.load(std::memory_order_relaxed),
        duplicates_.load(std::memory_order_relaxed),
        replayed_.load(std::memory_order_relaxed),
        rejected_.load(std::memory_order_relaxed),
        dropped_.load(std::memory_order_relaxed),
        unauthorized_.load(std::memory_order_relaxed),
    };
}
