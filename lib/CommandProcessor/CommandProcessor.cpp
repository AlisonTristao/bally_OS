#include <CommandProcessor.h>

#include <btp/messages.hpp>

#include <cstring>

namespace {

struct Unlock {
    std::atomic_flag& lock;
    ~Unlock() { lock.clear(std::memory_order_release); }
};

}  // namespace

CommandProcessor::CommandProcessor() noexcept : dedup_(bind_dedup()) {}

btp::DedupCache CommandProcessor::bind_dedup() noexcept {
    for (std::size_t i = 0U; i < kCacheCapacity; ++i) {
        dedup_storage_[i].data = dedup_bytes_[i];
        dedup_storage_[i].capacity = kSlotStorageBytes;
    }
    return btp::DedupCache(dedup_slots_, dedup_storage_, kCacheCapacity,
                           dedup_requesters_, kRequesterCapacity);
}

void CommandProcessor::configure(BtpEndpoint& endpoint, BtpSealFn seal_link,
                                 void* seal_link_context,
                                 BtpSealFn seal_endpoint,
                                 void* seal_endpoint_context) noexcept {
    endpoint_ = &endpoint;
    seal_link_ = seal_link;
    seal_link_context_ = seal_link_context;
    seal_endpoint_ = seal_endpoint;
    seal_endpoint_context_ = seal_endpoint_context;
}

std::uint16_t CommandProcessor::read_u16(const std::uint8_t* input) noexcept {
    return static_cast<std::uint16_t>(input[0]) |
           static_cast<std::uint16_t>(
               static_cast<std::uint16_t>(input[1]) << 8U);
}

std::uint32_t CommandProcessor::read_u32(const std::uint8_t* input) noexcept {
    return static_cast<std::uint32_t>(input[0]) |
           (static_cast<std::uint32_t>(input[1]) << 8U) |
           (static_cast<std::uint32_t>(input[2]) << 16U) |
           (static_cast<std::uint32_t>(input[3]) << 24U);
}

std::uint64_t CommandProcessor::read_u64(const std::uint8_t* input) noexcept {
    std::uint64_t value = 0U;
    for (std::size_t i = 0U; i < 8U; ++i) {
        value |= static_cast<std::uint64_t>(input[i]) << (i * 8U);
    }
    return value;
}

void CommandProcessor::write_u32(std::uint8_t* output,
                                 std::uint32_t value) noexcept {
    output[0] = static_cast<std::uint8_t>(value);
    output[1] = static_cast<std::uint8_t>(value >> 8U);
    output[2] = static_cast<std::uint8_t>(value >> 16U);
    output[3] = static_cast<std::uint8_t>(value >> 24U);
}

void CommandProcessor::write_u64(std::uint8_t* output,
                                 std::uint64_t value) noexcept {
    for (std::size_t i = 0U; i < 8U; ++i) {
        output[i] = static_cast<std::uint8_t>(value >> (i * 8U));
    }
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

bool CommandProcessor::encode_result(
    const btp::DedupKey& key, std::uint16_t action_id,
    std::uint16_t action_version, Status status, ErrorCode error,
    const char* message, std::uint8_t* output, std::size_t capacity,
    std::uint16_t* size_out) noexcept {
    if (output == nullptr || size_out == nullptr || message == nullptr) {
        return false;
    }

    // COMMAND_RESULT layout (commands.md section 2.4) is btp::encode_command_result.
    // A shell result never carries a bytes_u32 result body -- only the message.
    btp::CommandResult out{};
    out.request = {key.source_id, key.boot_id, key.sequence};
    out.action_id = action_id;
    out.action_version = action_version;
    out.status = static_cast<std::uint8_t>(status);
    out.error_code = static_cast<std::uint16_t>(error);
    out.message = {reinterpret_cast<const std::uint8_t*>(message),
                   std::strlen(message)};

    std::size_t written = 0U;
    if (btp::encode_command_result(out, output, capacity, &written) !=
        btp::MessageError::Ok) {
        return false;
    }
    *size_out = static_cast<std::uint16_t>(written);
    return true;
}

bool CommandProcessor::finish(std::size_t slot, Status status, ErrorCode error,
                              const char* message, std::uint64_t now_us,
                              std::uint8_t* view_buf,
                              ResultView* result_out) noexcept {
    if (endpoint_ == nullptr || result_out == nullptr ||
        slot >= kCacheCapacity) {
        return false;
    }
    const SlotMeta& meta = slot_meta_[slot];

    std::uint8_t packed[kReplayPrefix + kMaxResultPayloadSize];
    std::uint32_t sequence = 0U;
    std::uint16_t payload_size = 0U;
    if (!endpoint_->reserve_sequence(&sequence) ||
        !encode_result(meta.key, meta.action_id, meta.action_version, status,
                       error, message, packed + kReplayPrefix,
                       kMaxResultPayloadSize, &payload_size)) {
        dropped_.fetch_add(1U, std::memory_order_relaxed);
        return false;
    }

    // Prefix the stored copy with the frame fields a retransmission replays
    // verbatim and the payload does not carry.
    write_u32(packed, sequence);
    write_u64(packed + 4U, now_us);
    if (dedup_.record_result(slot, packed, kReplayPrefix + payload_size) !=
        btp::MessageError::Ok) {
        dropped_.fetch_add(1U, std::memory_order_relaxed);
        return false;
    }

    std::memcpy(view_buf, packed + kReplayPrefix, payload_size);
    *result_out = {sequence, now_us, view_buf, payload_size, meta.channel};
    return true;
}

bool CommandProcessor::make_transient(
    const btp::DedupKey& key, std::uint16_t action_id,
    std::uint16_t action_version, Status status, ErrorCode error,
    const char* message, std::uint64_t now_us, bally::Channel channel,
    ResultView* result_out) noexcept {
    if (endpoint_ == nullptr || result_out == nullptr) {
        return false;
    }
    std::uint32_t sequence = 0U;
    std::uint16_t size = 0U;
    if (!endpoint_->reserve_sequence(&sequence) ||
        !encode_result(key, action_id, action_version, status, error, message,
                       rx_view_, sizeof(rx_view_), &size)) {
        dropped_.fetch_add(1U, std::memory_order_relaxed);
        return false;
    }
    *result_out = {sequence, now_us, rx_view_, size, channel};
    return true;
}

CommandProcessor::ResultView CommandProcessor::replay_view(
    btp::ByteView stored, bally::Channel channel) noexcept {
    ResultView view{};
    if (stored.data == nullptr || stored.size < kReplayPrefix) {
        return view;
    }
    std::size_t payload_size = stored.size - kReplayPrefix;
    if (payload_size > sizeof(rx_view_)) {
        payload_size = sizeof(rx_view_);
    }
    std::memcpy(rx_view_, stored.data + kReplayPrefix, payload_size);
    view.sequence = read_u32(stored.data);
    view.timestamp_us = read_u64(stored.data + 4U);
    view.payload = rx_view_;
    view.payload_size = payload_size;
    view.channel = channel;
    return view;
}

CommandProcessor::Intake CommandProcessor::intake(
    const btp::Header& header, btp::ByteView payload, std::uint64_t now_us,
    bally::Channel channel) noexcept {
    Intake intake{};
    // RX runs on the Wi-Fi task: never wait behind the shell task. A retry is
    // safe and will hit the same dedup key after the short critical section.
    if (cache_lock_.test_and_set(std::memory_order_acquire)) {
        note_drop();
        return intake;
    }
    Unlock unlock{cache_lock_};

    if (endpoint_ == nullptr || header.type != btp::MessageType::Command ||
        header.object_id != btp_command::kCommandRequestObjectId ||
        header.source_id == 0U || header.boot_id == 0U ||
        header.sequence == 0U || payload.data == nullptr ||
        payload.size > btp_command::kMaxLogicalRequestSize) {
        note_drop();
        return intake;
    }

    const btp::DedupKey key{header.source_id, header.boot_id, header.sequence};
    std::uint16_t action_id = 0U;
    std::uint16_t action_version = 0U;
    if (payload.size >= 12U) {
        action_id = read_u16(payload.data + 8U);
        action_version = read_u16(payload.data + 10U);
    }

    std::size_t slot = 0U;
    btp::ByteView stored{};
    const btp::DedupVerdict verdict = dedup_.classify(
        key, payload.data, payload.size, &slot, &stored);

    switch (verdict) {
        case btp::DedupVerdict::DuplicateComplete:
            duplicates_.fetch_add(1U, std::memory_order_relaxed);
            replayed_.fetch_add(1U, std::memory_order_relaxed);
            intake.kind = IntakeKind::ResultReady;
            intake.result = replay_view(stored, channel);
            return intake;

        case btp::DedupVerdict::DuplicateInFlight:
            duplicates_.fetch_add(1U, std::memory_order_relaxed);
            intake.kind = IntakeKind::DuplicateInProgress;
            return intake;

        case btp::DedupVerdict::Conflict:
            duplicates_.fetch_add(1U, std::memory_order_relaxed);
            rejected_.fetch_add(1U, std::memory_order_relaxed);
            if (make_transient(key, action_id, action_version, Status::Rejected,
                               ErrorCode::RequestConflict,
                               "request identity conflict", now_us, channel,
                               &intake.result)) {
                intake.kind = IntakeKind::ResultReady;
            }
            return intake;

        case btp::DedupVerdict::Evicted:
        case btp::DedupVerdict::CapacityExhausted:
            rejected_.fetch_add(1U, std::memory_order_relaxed);
            if (make_transient(key, action_id, action_version, Status::Busy,
                               ErrorCode::CapacityExhausted,
                               "command cache exhausted", now_us, channel,
                               &intake.result)) {
                intake.kind = IntakeKind::ResultReady;
            }
            return intake;

        case btp::DedupVerdict::InvalidArgument:
            note_drop();
            return intake;

        case btp::DedupVerdict::Fresh:
            break;
    }

    // Fresh: the slot is reserved. Record what complete()/reject_busy() will
    // need, then parse -- a parse failure finishes the slot right here.
    slot_meta_[slot].key = key;
    slot_meta_[slot].action_id = action_id;
    slot_meta_[slot].action_version = action_version;
    slot_meta_[slot].channel = channel;

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
        if (finish(slot, parse_status(parse_error),
                   parse_error_code(parse_error), parse_message(parse_error),
                   now_us, rx_view_, &intake.result)) {
            intake.kind = IntakeKind::ResultReady;
        }
        return intake;
    }

    accepted_.fetch_add(1U, std::memory_order_relaxed);
    intake.work.cache_slot = static_cast<std::uint8_t>(slot);
    intake.kind = IntakeKind::Ready;
    return intake;
}

bool CommandProcessor::complete(std::uint8_t cache_slot,
                                std::uint8_t shell_status,
                                std::uint64_t now_us,
                                ResultView* result_out) noexcept {
    while (cache_lock_.test_and_set(std::memory_order_acquire)) {}
    Unlock unlock{cache_lock_};
    if (cache_slot >= kCacheCapacity) return false;
    const bool success = shell_status == 0U;
    if (!finish(cache_slot, success ? Status::Success : Status::Failed,
                success ? ErrorCode::None : ErrorCode::InternalError,
                success ? "" : "shell command failed", now_us, shell_view_,
                result_out)) {
        return false;
    }
    executed_.fetch_add(1U, std::memory_order_relaxed);
    return true;
}

bool CommandProcessor::reject_busy(std::uint8_t cache_slot,
                                   std::uint64_t now_us,
                                   ResultView* result_out) noexcept {
    while (cache_lock_.test_and_set(std::memory_order_acquire)) {}
    Unlock unlock{cache_lock_};
    if (cache_slot >= kCacheCapacity) return false;
    rejected_.fetch_add(1U, std::memory_order_relaxed);
    return finish(cache_slot, Status::Busy, ErrorCode::CapacityExhausted,
                  "command execution queue full", now_us, rx_view_, result_out);
}

bool CommandProcessor::send_result(const ResultView& result) noexcept {
    if (endpoint_ == nullptr || result.sequence == 0U ||
        result.payload == nullptr || result.payload_size == 0U ||
        result.payload_size > btp::kEspNowMaxPayloadSize) {
        note_drop();
        return false;
    }

    // channel_of_peer(Vantage::Robot, ...) never returns A_Console (see
    // bally_channels.h) -- this firmware is never the console's own peer --
    // so B_Endpoint is the only branch besides the C_Link default.
    const BtpSealFn seal = result.channel == bally::Channel::B_Endpoint
                                ? seal_endpoint_
                                : seal_link_;
    void* const seal_context = result.channel == bally::Channel::B_Endpoint
                                    ? seal_endpoint_context_
                                    : seal_link_context_;

    // Both configured nullptr means "no encryption at all" (see configure's
    // comment) and every reply goes out unsealed, matching pre-channel-B
    // behavior for the native tests. But once ANY channel has a key
    // configured, a channel whose own key is still missing must never fall
    // back to cleartext or to the other channel's key -- that key is the one
    // its requester actually holds.
    if (seal == nullptr &&
        (seal_link_ != nullptr || seal_endpoint_ != nullptr)) {
        note_drop();
        return false;
    }

    const bool sent = endpoint_->send_fragment(
        btp::MessageType::Command, kCommandResultObjectId, result.sequence,
        result.timestamp_us, result.payload, result.payload_size, 0U, 1U,
        seal, seal_context);
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
