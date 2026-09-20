#include <BtpTransport.h>

#include <btp/messages.hpp>

#include <cstring>

bool BtpEndpoint::default_send(void*, const std::uint8_t*, std::size_t) noexcept {
    return false;
}

bool BtpEndpoint::legacy_send(void* context, const std::uint8_t* data,
                              std::size_t size) noexcept {
    if (context == nullptr) return false;
    auto* endpoint = static_cast<BtpEndpoint*>(context);
    const SendCallback callback =
        endpoint->legacy_callback_.load(std::memory_order_acquire);
    return callback != nullptr && callback(data, size);
}

void BtpEndpoint::set_send_callback(SendCallback callback) noexcept {
    legacy_callback_.store(callback, std::memory_order_release);
    set_send_callback(callback != nullptr ? &legacy_send : &default_send,
                      callback != nullptr ? this : nullptr);
}

void BtpEndpoint::set_send_callback(ContextSendCallback callback,
                                    void* context) noexcept {
    send_context_.store(context, std::memory_order_release);
    send_callback_.store(callback != nullptr ? callback : &default_send,
                         std::memory_order_release);
}

namespace {

btp::LogicalMessage make_logical(btp::MessageType type, std::uint16_t object_id,
                                 const std::uint8_t* payload,
                                 std::size_t payload_size,
                                 std::uint64_t timestamp_us) noexcept {
    btp::LogicalMessage message;
    message.type = type;
    message.object_id = object_id;
    message.timestamp_us = timestamp_us;
    message.payload = {payload, payload_size};
    return message;
}

}  // namespace

bool BtpEndpoint::send_logical(btp::MessageType type, std::uint16_t object_id,
                               const std::uint8_t* payload,
                               std::size_t payload_size,
                               std::uint64_t timestamp_us, SealFn seal,
                               void* seal_context) noexcept {
    const btp::LogicalMessage message =
        make_logical(type, object_id, payload, payload_size, timestamp_us);
    const ContextSendCallback send =
        send_callback_.load(std::memory_order_acquire);
    void* const send_context = send_context_.load(std::memory_order_acquire);

    if (seal == nullptr) {
        return active_->send_logical(message, transport_,
                                     send, send_context, nullptr, 0U);
    }
    std::uint8_t scratch[kMaxLogicalPayloadSize + kAeadTagSize];
    return active_->send_logical(message, transport_,
                                 send, send_context, scratch,
                                 sizeof(scratch), seal, seal_context);
}

bool BtpEndpoint::send_logical_reserved(btp::MessageType type,
                                       std::uint16_t object_id,
                                       std::uint32_t sequence,
                                       const std::uint8_t* payload,
                                       std::size_t payload_size,
                                       std::uint64_t timestamp_us, SealFn seal,
                                       void* seal_context) const noexcept {
    const btp::LogicalMessage message =
        make_logical(type, object_id, payload, payload_size, timestamp_us);
    const ContextSendCallback send =
        send_callback_.load(std::memory_order_acquire);
    void* const send_context = send_context_.load(std::memory_order_acquire);

    if (seal == nullptr) {
        return active_->send_logical_reserved(
            sequence, message, transport_, send, send_context,
            nullptr, 0U);
    }
    std::uint8_t scratch[kMaxLogicalPayloadSize + kAeadTagSize];
    return active_->send_logical_reserved(
        sequence, message, transport_, send, send_context,
        scratch, sizeof(scratch), seal, seal_context);
}

bool BtpEndpoint::encode_fragment(btp::MessageType type, std::uint16_t object_id,
                                  std::uint32_t sequence,
                                  std::uint64_t timestamp_us,
                                  const std::uint8_t* payload,
                                  std::size_t payload_size,
                                  std::uint8_t fragment_index,
                                  std::uint8_t fragment_count,
                                  std::uint8_t* output,
                                  std::size_t output_capacity,
                                  std::size_t* bytes_written, SealFn seal,
                                  void* seal_context) const noexcept {
    return active_->encode_fragment(
        make_logical(type, object_id, payload, payload_size, timestamp_us),
        transport_, sequence, fragment_index, fragment_count,
        output, output_capacity, bytes_written, seal, seal_context);
}

bool BtpEndpoint::send_fragment(btp::MessageType type, std::uint16_t object_id,
                                std::uint32_t sequence,
                                std::uint64_t timestamp_us,
                                const std::uint8_t* payload,
                                std::size_t payload_size,
                                std::uint8_t fragment_index,
                                std::uint8_t fragment_count, SealFn seal,
                                void* seal_context) const noexcept {
    return active_->send_fragment(
        make_logical(type, object_id, payload, payload_size, timestamp_us),
        transport_, sequence, fragment_index, fragment_count,
        send_callback_.load(std::memory_order_acquire),
        send_context_.load(std::memory_order_acquire), seal, seal_context);
}

bool BtpEndpoint::send_encoded(const std::uint8_t* frame,
                               std::size_t frame_size) const noexcept {
    return active_->send_encoded(
        frame, frame_size, transport_,
        send_callback_.load(std::memory_order_acquire),
        send_context_.load(std::memory_order_acquire));
}

// ---------------------------------------------------------------------------
// TcpBusyResponder
// ---------------------------------------------------------------------------

TcpBusyResponder::TcpBusyResponder() noexcept
    : decoder_(cobs_buffer_, sizeof(cobs_buffer_), decoded_buffer_,
              sizeof(decoded_buffer_)) {}

void TcpBusyResponder::reset() noexcept {
    decoder_.reset();
}

bool TcpBusyResponder::feed(const std::uint8_t* data, std::size_t size,
                            const btp::Hello& local, std::uint32_t source_id,
                            std::uint32_t boot_id, std::uint64_t now_ms,
                            const std::uint8_t** frame_out,
                            std::size_t* frame_size) noexcept {
    if (data == nullptr || frame_out == nullptr || frame_size == nullptr) {
        return false;
    }

    for (std::size_t i = 0U; i < size; ++i) {
        btp::DecodedFrame decoded{};
        const btp::SerialDecodeResult result = decoder_.push(data[i], &decoded);
        if (result.event != btp::SerialDecodeEvent::Frame) {
            // None, CobsError, FrameError, Overflow: nothing conclusive from
            // this byte alone -- keep feeding (SerialDecoder resynchronizes
            // on the next delimiter by itself).
            continue;
        }

        // A throwaway responder session: it exists only to reuse
        // btp::Session's own HELLO negotiation (version selection, limits
        // minimum) so the version this rejection reports is the exact one
        // the real session would have picked -- see this class's own
        // comment. hello_deadline_ms=0 disables ITS deadline (irrelevant,
        // on_frame() is called exactly once, immediately after arm()).
        btp::Session probe(local, /*hello_deadline_ms=*/0U);
        probe.arm(now_ms);
        std::uint8_t success_payload[btp::kSessionMaxReplySize];
        const btp::SessionOutcome outcome =
            probe.on_frame(decoded, now_ms, success_payload,
                          sizeof(success_payload));
        if (outcome.event != btp::SessionEvent::HelloAccepted) {
            // Not a HELLO this responder would have accepted anyway (wrong
            // object_id/type, malformed, no common version, or the probe
            // itself timed out/was ignored) -- nothing to reject as BUSY;
            // BTP/docs/session-and-terminal.md section 2.4 only calls for a
            // BUSY reply on a HELLO that would otherwise have succeeded.
            continue;
        }

        // LIBRARY GAP (found while implementing this, not fixed here -- BTP
        // is read-only for this task): btp::encode_hello_result() itself
        // hard-rejects any status other than Success/Unsupported
        // (BTP/src/messages.cpp, encode_hello_result(): "in.status !=
        // Success && in.status != Unsupported -> InvalidValue"), even though
        // session-and-terminal.md section 2.4 explicitly calls for reusing
        // this same message with status=BUSY. There is no public API to
        // produce that wire payload, so the SUCCESS payload
        // probe.on_frame() already built above (a real, library-validated
        // HELLO_RESULT encoding) is patched in place instead: same wire
        // layout (BTP/src/messages.cpp's encode_hello_result(), verified
        // against this exact byte sequence while debugging this responder),
        // only status/error_code/the five negotiated-limit fields change.
        // request/selected_version/peer_uuid/config_revision are left
        // exactly as negotiated, untouched by the patch below.
        static_assert(btp::kSessionMaxReplySize == 52U,
                     "HelloResult wire layout offsets below assume this size");
        constexpr std::size_t kStatusOffset = 12U;           // u8
        constexpr std::size_t kErrorCodeOffset = 14U;        // u16 LE
        // The five negotiated-limit fields (max_logical_payload u32,
        // max_inflight_reassemblies u16, max_subscriptions u16,
        // max_dedup_entries u32, session_timeout_ms u32) are contiguous,
        // offsets 16..31, and ALL become zero (section 2.4: "all negotiated
        // limits are zero") -- zeroed as one 16-byte range rather than field
        // by field. peer_uuid (32..47) and config_revision (48..51) are left
        // exactly as negotiated.
        constexpr std::size_t kLimitsOffset = 16U;
        constexpr std::size_t kLimitsSize = 16U;
        if (outcome.reply_size != btp::kSessionMaxReplySize) {
            // A sizing assumption broke (a future library change resized
            // HELLO_RESULT) -- refuse to guess at a possibly-wrong layout
            // rather than corrupt a reply.
            continue;
        }

        std::uint8_t busy_payload[btp::kSessionMaxReplySize];
        std::memcpy(busy_payload, success_payload, sizeof(busy_payload));
        busy_payload[kStatusOffset] = static_cast<std::uint8_t>(btp::ResultStatus::Busy);
        const std::uint16_t error_code =
            static_cast<std::uint16_t>(btp::ResultError::CapacityExhausted);
        busy_payload[kErrorCodeOffset] = static_cast<std::uint8_t>(error_code);
        busy_payload[kErrorCodeOffset + 1U] = static_cast<std::uint8_t>(error_code >> 8U);
        std::memset(busy_payload + kLimitsOffset, 0, kLimitsSize);
        const std::size_t payload_written = sizeof(busy_payload);

        // Frame it exactly like every other HELLO_RESULT this library sends
        // today: cleartext (flags=0), Control/kHelloResult. Cleartext here
        // matches btp::Node's own current behavior for HELLO_RESULT (see
        // BtpTransport.h's BtpSealFn comment and
        // TAREFAS_TCP_BLE_ANDROID.txt's T21/T22 notes) -- a known gap
        // against fragmentation-and-transports.md section 9.5's TCP-mandatory-
        // encryption rule that this responder does not attempt to fix on its
        // own (that is a btp::Node/btp::Session library gap, not something a
        // one-off rejection frame can close by itself).
        btp::Header header{};
        header.type = btp::MessageType::Control;
        header.flags = 0U;
        header.source_id = source_id;
        header.boot_id = boot_id;
        // Arbitrary and unobserved by anything else: this frame is a one-shot
        // rejection with no session and no continuation, so it needs no real
        // place in this identity's normal outgoing sequence.
        header.sequence = 1U;
        header.timestamp_us = now_ms * 1000ULL;
        header.object_id = btp::object_id::kHelloResult;
        header.fragment_index = 0U;
        header.fragment_count = 1U;

        const btp::Frame frame{header, {busy_payload, payload_written}};
        std::uint8_t encoded[btp::kSessionMaxReplySize + 40U];
        std::size_t encoded_written = 0U;
        if (btp::encode(frame, btp::kTcpTransport, encoded, sizeof(encoded),
                        &encoded_written) != btp::Error::Ok) {
            continue;
        }

        // COBS-encode with the leading/trailing 0x00 delimiters
        // (BTP/include/btp/stream.hpp: "0x00 || COBS(frame) || 0x00").
        reply_frame_[0] = 0x00U;
        std::size_t cobs_written = 0U;
        if (btp::cobs_encode(encoded, encoded_written, reply_frame_ + 1U,
                             sizeof(reply_frame_) - 2U,
                             &cobs_written) != btp::CobsError::Ok) {
            continue;
        }
        reply_frame_[1U + cobs_written] = 0x00U;

        *frame_out = reply_frame_;
        *frame_size = cobs_written + 2U;
        return true;
    }

    return false;
}

namespace btp_command {

ParseError parse_request(const btp::Header& header,
                         btp::ByteView payload,
                         std::uint32_t local_source_id,
                         std::uint32_t local_boot_id,
                         RequestView* request_out) noexcept {
    if (request_out == nullptr ||
        (payload.data == nullptr && payload.size != 0U)) {
        return ParseError::InvalidEnvelope;
    }
    if (header.type != btp::MessageType::Command) return ParseError::WrongType;
    if (header.object_id != kCommandRequestObjectId) {
        return ParseError::WrongObject;
    }
    if (header.flags != 0U || header.fragment_index != 0U ||
        header.fragment_count != 1U || header.source_id == 0U ||
        header.boot_id == 0U) {
        return ParseError::InvalidEnvelope;
    }

    // The COMMAND_REQUEST payload layout (commands.md section 2.1) is
    // btp::decode_command_request: the 20-octet prefix, the zero flags/reserved
    // words, every id non-zero and "parameter_size consumes the payload
    // exactly" all live in the library now.
    btp::CommandRequest cmd{};
    const btp::MessageError err = btp::decode_command_request(payload.data, payload.size, &cmd);
    if (err != btp::MessageError::Ok) {
        switch (err) {
            case btp::MessageError::PayloadTooShort:
                return ParseError::PayloadTooShort;
            case btp::MessageError::ReservedNotZero:
                return ParseError::InvalidFlags;
            case btp::MessageError::TrailingBytes:
            case btp::MessageError::LengthOverflow:
                return ParseError::SizeMismatch;
            case btp::MessageError::CountTooLarge:
                return ParseError::ParametersTooLarge;
            default:  // ZeroField: a zero target or action id
                return ParseError::InvalidEnvelope;
        }
    }

    if (cmd.target_source_id != local_source_id || cmd.target_boot_id != local_boot_id) {
        return ParseError::WrongTarget;
    }
    if (cmd.parameters.size > kMaxShellCommandSize) {
        return ParseError::ParametersTooLarge;  // stricter local ceiling than the wire's
    }

    request_out->target_source_id = cmd.target_source_id;
    request_out->target_boot_id = cmd.target_boot_id;
    request_out->action_id = cmd.action_id;
    request_out->action_version = cmd.action_version;
    request_out->parameters = cmd.parameters;
    return ParseError::Ok;
}

ParseError copy_shell_command(const RequestView& request,
                              char* output,
                              std::size_t output_capacity) noexcept {
    if (request.action_id != kShellActionId ||
        request.action_version != kShellActionVersion) {
        return ParseError::UnsupportedAction;
    }
    if (request.parameters.data == nullptr || request.parameters.size == 0U ||
        request.parameters.size > kMaxShellCommandSize) {
        return ParseError::InvalidShellText;
    }
    if (output == nullptr || output_capacity <= request.parameters.size) {
        return ParseError::OutputTooSmall;
    }

    for (std::size_t index = 0U; index < request.parameters.size; ++index) {
        const std::uint8_t byte = request.parameters.data[index];
        // A COMMAND_REQUEST represents one TinyShell command, never a stream
        // or a batch. UTF-8 bytes >= 0x80 remain valid and opaque here.
        if (byte == 0U || byte == '\r' || byte == '\n' || byte == 0x7FU ||
            (byte < 0x20U && byte != '\t')) {
            return ParseError::InvalidShellText;
        }
    }

    std::memcpy(output, request.parameters.data, request.parameters.size);
    output[request.parameters.size] = '\0';
    return ParseError::Ok;
}

std::uint32_t source_id_from_mac(const std::uint8_t mac[6]) noexcept {
    if (mac == nullptr) return 0U;
    std::uint32_t source_id =
        (static_cast<std::uint32_t>(mac[2]) << 24U) |
        (static_cast<std::uint32_t>(mac[3]) << 16U) |
        (static_cast<std::uint32_t>(mac[4]) << 8U) |
        static_cast<std::uint32_t>(mac[5]);
    // A factory MAC cannot normally produce zero here, but BTP reserves it.
    if (source_id == 0U) source_id = 1U;
    return source_id;
}

// A CHEAP RADIO FILTER, NOT AUTHENTICATION. Read the whole comment before
// changing anything here.
//
// This used to also require claimed_source_id == source_id_from_mac(
// received_mac) -- the source_id a frame declared had to derive from the MAC
// the radio reported. That condition is gone: it encodes "one peer, one
// identity", and the hub model breaks that. Every frame now arrives over the
// radio from the dongle's MAC, but its source_id is whoever actually wrote it
// -- TraceView talking end to end through the hub, or the dongle itself.
// Under the old rule every relayed frame is rejected, so the two cannot
// coexist.
//
// What is left is the MAC memcmp. It drops traffic from anything that is not
// our one peer before it costs a decode, but it is NOT authorization: an
// ESP-NOW source MAC is forged in one line, and BTP itself says so
// (docs/fragmentation-and-transports.md 3.1, "it does not authenticate
// anything").
//
// The real authorization is the AEAD tag, checked immediately after
// reassembly in ROBOT::handleReceiveStatic: the frame is classified to a
// channel by its cleartext source_id and then opened with RadioSeal::open
// (key L, channel C) or open_e (key E, channel B), both fail-closed, no
// fallback to reading the still-sealed bytes. A forged MAC clears this
// memcmp; it does not produce a 16-octet tag without the key, so it never
// reaches the shell.
bool authorized_source(const std::uint8_t expected_mac[6],
                       const std::uint8_t received_mac[6]) noexcept {
    return expected_mac != nullptr && received_mac != nullptr &&
           std::memcmp(expected_mac, received_mac, 6U) == 0;
}

const char* parse_error_string(ParseError error) noexcept {
    switch (error) {
        case ParseError::Ok: return "ok";
        case ParseError::WrongType: return "not a COMMAND message";
        case ParseError::WrongObject: return "not COMMAND_REQUEST";
        case ParseError::InvalidEnvelope: return "invalid logical envelope";
        case ParseError::PayloadTooShort: return "command payload too short";
        case ParseError::WrongTarget: return "command target does not match this boot";
        case ParseError::InvalidAction: return "invalid action identity";
        case ParseError::InvalidFlags: return "command reserved field is nonzero";
        case ParseError::SizeMismatch: return "command parameter size mismatch";
        case ParseError::ParametersTooLarge: return "command parameters too large";
        case ParseError::UnsupportedAction: return "unsupported action";
        case ParseError::InvalidShellText: return "invalid shell command bytes";
        case ParseError::OutputTooSmall: return "shell output buffer too small";
    }
    return "unknown command parse error";
}

}  // namespace btp_command
