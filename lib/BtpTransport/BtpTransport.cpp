#include <BtpTransport.h>

#include <btp/fragmentation.hpp>

#include <cstring>
#include <limits>

namespace {

std::uint16_t read_u16_le(const std::uint8_t* data) noexcept {
    return static_cast<std::uint16_t>(data[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8U);
}

std::uint32_t read_u32_le(const std::uint8_t* data) noexcept {
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8U) |
           (static_cast<std::uint32_t>(data[2]) << 16U) |
           (static_cast<std::uint32_t>(data[3]) << 24U);
}

}  // namespace

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

bool BtpEndpoint::configure(std::uint32_t source_id,
                            std::uint32_t boot_id) noexcept {
    if (source_id == 0U || boot_id == 0U) return false;

    source_id_ = source_id;
    boot_id_ = boot_id;
    next_sequence_.store(1U, std::memory_order_release);
    return true;
}

void BtpEndpoint::set_send_callback(SendCallback callback) noexcept {
    legacy_callback_.store(callback, std::memory_order_release);
    set_send_callback(callback != nullptr ? legacy_send : default_send,
                      callback != nullptr ? this : nullptr);
}

void BtpEndpoint::set_send_callback(ContextSendCallback callback,
                                    void* context) noexcept {
    send_context_.store(context, std::memory_order_release);
    send_callback_.store(callback != nullptr ? callback : default_send,
                         std::memory_order_release);
}

bool BtpEndpoint::reserve_sequence(std::uint32_t* sequence_out) noexcept {
    if (sequence_out == nullptr || source_id_ == 0U || boot_id_ == 0U) {
        return false;
    }

    std::uint32_t current = next_sequence_.load(std::memory_order_acquire);
    while (current != 0U) {
        const std::uint32_t next =
            current == std::numeric_limits<std::uint32_t>::max()
                ? 0U
                : current + 1U;
        if (next_sequence_.compare_exchange_weak(
                current, next, std::memory_order_acq_rel,
                std::memory_order_acquire)) {
            *sequence_out = current;
            return true;
        }
    }
    return false;
}

bool BtpEndpoint::try_reserve_sequence(std::uint32_t* sequence_out) noexcept {
    if (sequence_out == nullptr || source_id_ == 0U || boot_id_ == 0U) {
        return false;
    }

    std::uint32_t current = next_sequence_.load(std::memory_order_acquire);
    if (current == 0U) return false;
    const std::uint32_t next =
        current == std::numeric_limits<std::uint32_t>::max()
            ? 0U
            : current + 1U;
    if (!next_sequence_.compare_exchange_strong(
            current, next, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
        return false;
    }
    *sequence_out = current;
    return true;
}

bool BtpEndpoint::encode_fragment(btp::MessageType type,
                                  std::uint16_t object_id,
                                  std::uint32_t sequence,
                                  std::uint64_t timestamp_us,
                                  const std::uint8_t* payload,
                                  std::size_t payload_size,
                                  std::uint8_t fragment_index,
                                  std::uint8_t fragment_count,
                                  std::uint8_t* output,
                                  std::size_t output_capacity,
                                  std::size_t* bytes_written,
                                  SealFn seal,
                                  void* seal_context) const noexcept {
    if (source_id_ == 0U || boot_id_ == 0U || sequence == 0U ||
        fragment_count == 0U || fragment_index >= fragment_count ||
        payload_size > btp::kEspNowMaxPayloadSize ||
        (payload == nullptr && payload_size != 0U)) {
        return false;
    }
    // The AEAD tag covers the whole logical payload, never a slice of one
    // (see SealFn's contract) -- a call representing only part of a message
    // cannot seal here.
    if (seal != nullptr && fragment_count != 1U) return false;
    if (seal != nullptr &&
        payload_size + kAeadTagSize > btp::kEspNowMaxPayloadSize) {
        return false;
    }

    const btp::Header header{
        .type = type,
        .flags = static_cast<std::uint16_t>(
            (fragment_count > 1U ? btp::kFlagFragmented : 0U) |
            (seal != nullptr ? btp::kFlagEncrypted : 0U)),
        .source_id = source_id_,
        .boot_id = boot_id_,
        .sequence = sequence,
        .timestamp_us = timestamp_us,
        .object_id = object_id,
        .fragment_index = fragment_index,
        .fragment_count = fragment_count,
    };

    if (seal == nullptr) {
        const btp::Frame frame{header, {payload, payload_size}};
        return btp::encode(frame, btp::TransportProfile::EspNow, output,
                           output_capacity, bytes_written) == btp::Error::Ok;
    }

    // Sealed once, in place of the plaintext -- ENCRYPTED is already set on
    // `header` above, since the AAD is computed from the header AS GIVEN.
    std::uint8_t sealed[btp::kEspNowMaxPayloadSize];
    if (!seal(seal_context, header, static_cast<std::uint16_t>(payload_size),
             payload, sealed)) {
        return false;
    }
    const btp::Frame frame{header, {sealed, payload_size + kAeadTagSize}};
    return btp::encode(frame, btp::TransportProfile::EspNow, output,
                       output_capacity, bytes_written) == btp::Error::Ok;
}

bool BtpEndpoint::send_fragment(btp::MessageType type,
                                std::uint16_t object_id,
                                std::uint32_t sequence,
                                std::uint64_t timestamp_us,
                                const std::uint8_t* payload,
                                std::size_t payload_size,
                                std::uint8_t fragment_index,
                                std::uint8_t fragment_count,
                                SealFn seal,
                                void* seal_context) const noexcept {
    std::uint8_t frame[btp::kEspNowMaxFrameSize];
    std::size_t frame_size = 0U;
    if (!encode_fragment(type, object_id, sequence, timestamp_us, payload,
                         payload_size, fragment_index, fragment_count, frame,
                         sizeof(frame), &frame_size, seal, seal_context)) {
        return false;
    }
    return send_encoded(frame, frame_size);
}

bool BtpEndpoint::send_logical(btp::MessageType type,
                               std::uint16_t object_id,
                               const std::uint8_t* payload,
                               std::size_t payload_size,
                               std::uint64_t timestamp_us,
                               SealFn seal,
                               void* seal_context) noexcept {
    if (payload == nullptr && payload_size != 0U) return false;

    std::uint32_t sequence = 0U;
    if (!reserve_sequence(&sequence)) return false;

    if (seal == nullptr) {
        std::uint8_t count = 0U;
        if (btp::fragment_count(payload_size, btp::TransportProfile::EspNow,
                                &count) != btp::Error::Ok) {
            return false;
        }
        for (std::uint8_t index = 0U; index < count; ++index) {
            const std::size_t offset =
                static_cast<std::size_t>(index) * btp::kEspNowMaxPayloadSize;
            const std::size_t remaining = payload_size - offset;
            const std::size_t fragment_size =
                remaining < btp::kEspNowMaxPayloadSize
                    ? remaining
                    : btp::kEspNowMaxPayloadSize;
            const std::uint8_t* fragment =
                payload == nullptr ? nullptr : payload + offset;
            if (!send_fragment(type, object_id, sequence, timestamp_us,
                               fragment, fragment_size, index, count)) {
                return false;
            }
        }
        return true;
    }

    // Sealed path: seal ONCE over the whole logical message -- canonical
    // header, FRAGMENTED cleared, index 0, count 1 -- then slice the SEALED
    // bytes into wire fragments via btp::make_fragment. Never hand-slice a
    // sealed payload the way the plaintext loop above does: make_fragment is
    // what keeps every fragment's header agreeing with the one that was
    // actually sealed.
    if (payload_size > kMaxLogicalPayloadSize) return false;

    const btp::Header seal_header{
        .type = type,
        .flags = btp::kFlagEncrypted,
        .source_id = source_id_,
        .boot_id = boot_id_,
        .sequence = sequence,
        .timestamp_us = timestamp_us,
        .object_id = object_id,
        .fragment_index = 0U,
        .fragment_count = 1U,
    };

    std::uint8_t sealed[kMaxLogicalPayloadSize + kAeadTagSize];
    if (!seal(seal_context, seal_header,
             static_cast<std::uint16_t>(payload_size), payload, sealed)) {
        return false;
    }
    const std::size_t sealed_size = payload_size + kAeadTagSize;

    std::uint8_t count = 0U;
    if (btp::fragment_count(sealed_size, btp::TransportProfile::EspNow,
                            &count) != btp::Error::Ok) {
        return false;
    }

    btp::Header logical_header = seal_header;
    logical_header.flags = static_cast<std::uint16_t>(
        btp::kFlagEncrypted | (count > 1U ? btp::kFlagFragmented : 0U));
    logical_header.fragment_count = count;

    for (std::uint8_t index = 0U; index < count; ++index) {
        btp::Frame fragment{};
        if (btp::make_fragment(logical_header, {sealed, sealed_size},
                               btp::TransportProfile::EspNow, index,
                               &fragment) != btp::Error::Ok) {
            return false;
        }
        std::uint8_t frame[btp::kEspNowMaxFrameSize];
        std::size_t frame_size = 0U;
        if (btp::encode(fragment, btp::TransportProfile::EspNow, frame,
                        sizeof(frame), &frame_size) != btp::Error::Ok) {
            return false;
        }
        if (!send_encoded(frame, frame_size)) return false;
    }
    return true;
}

bool BtpEndpoint::send_encoded(const std::uint8_t* frame,
                               std::size_t frame_size) const noexcept {
    if (frame == nullptr || frame_size < btp::kV1MinimumFrameSize ||
        frame_size > btp::kEspNowMaxFrameSize) {
        return false;
    }
    const ContextSendCallback context_callback =
        send_callback_.load(std::memory_order_acquire);
    return context_callback(send_context_.load(std::memory_order_acquire),
                            frame, frame_size);
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
    if (payload.size < kRequestPrefixSize) return ParseError::PayloadTooShort;

    RequestView request{
        .target_source_id = read_u32_le(payload.data),
        .target_boot_id = read_u32_le(payload.data + 4U),
        .action_id = read_u16_le(payload.data + 8U),
        .action_version = read_u16_le(payload.data + 10U),
        .parameters = {payload.data + kRequestPrefixSize, 0U},
    };
    const std::uint16_t flags = read_u16_le(payload.data + 12U);
    const std::uint16_t reserved = read_u16_le(payload.data + 14U);
    const std::uint32_t parameter_size = read_u32_le(payload.data + 16U);

    if (request.target_source_id != local_source_id ||
        request.target_boot_id != local_boot_id) {
        return ParseError::WrongTarget;
    }
    if (request.action_id == 0U || request.action_version == 0U) {
        return ParseError::InvalidAction;
    }
    if (flags != 0U || reserved != 0U) return ParseError::InvalidFlags;
    if (parameter_size != payload.size - kRequestPrefixSize) {
        return ParseError::SizeMismatch;
    }
    if (parameter_size > kMaxShellCommandSize) {
        return ParseError::ParametersTooLarge;
    }

    request.parameters.size = parameter_size;
    *request_out = request;
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

// THE ROBOT HAS NO AUTHENTICATION AT THIS POINT. Read the whole comment
// before changing anything here.
//
// This used to also require claimed_source_id == source_id_from_mac(
// received_mac) -- that is, the source_id a frame declared had to be derived
// from the MAC the radio reported. That condition is gone, and NOTHING has
// replaced it yet.
//
// Why it had to go: it encodes "one peer, one identity", and the hub model
// breaks that. Every frame now arrives over the radio from the dongle's MAC,
// but its source_id is whoever actually wrote it -- TraceView talking end to
// end through the hub, or the dongle itself. Under the old rule every
// relayed frame is rejected, so the two cannot coexist.
//
// What is left is the MAC memcmp. That is a cheap radio filter and still
// worth having (it drops traffic from anything that is not our one peer
// before it costs a decode), but it is NOT authorization: an ESP-NOW source
// MAC is forged in one line, and BTP itself says so
// (docs/fragmentation-and-transports.md 3.1, "it does not authenticate
// anything").
//
// Real authorization arrives in topico 30: the AEAD tag over the frame
// becomes the check, at this same call site, and it is strictly stronger --
// a MAC is forged in one line, a 16-octet tag needs the key. Between this
// change and that one the robot executes shell commands for anyone who can
// spoof a MAC. That gap is deliberate and is why topico 28 is a BENCH
// delivery only.
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
