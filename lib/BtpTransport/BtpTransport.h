#ifndef BTP_TRANSPORT_H
#define BTP_TRANSPORT_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <btp/codec.hpp>
#include <btp/endpoint.hpp>

// AEAD sealer for channel C (dongle<->robot, key L -- see
// include/bally_channels.h and lib/RadioSeal, the only real implementation
// of this). This is btp::EndpointSealFn (BTP/include/btp/endpoint.hpp) under a
// project-local name: file scope, not a member of BtpEndpoint, on purpose --
// StatusReporter.h and CommandProcessor.h need to name this type while only
// forward-declaring `class BtpEndpoint`, which a nested member typedef would
// not survive.
//
// Called ONCE on the whole logical message, before any fragmenting -- never
// once per fragment. The associated data is the CANONICAL logical header
// (FRAGMENTED cleared, fragment_index 0, fragment_count 1 --
// BTP/docs/encryption.md section 5), and fragment_count for the actual
// frames on the wire has to be computed from the SEALED size (payload_size +
// kBtpAeadTagSize), not the plaintext size, or a message that fits in one
// ESP-NOW fragment unsealed could need two once sealed and silently lose its
// tail. `header.flags` already carries btp::kFlagEncrypted when this is
// called (the AAD is computed from the header AS GIVEN, so the flag has to
// be set before sealing, not patched in after). `out` has room for exactly
// `payload_size + kBtpAeadTagSize` octets.
//
// Returns false when there is no key to seal with, or the seal genuinely
// fails. Every send path then transmits NOTHING AT ALL -- fail-closed: an
// unsealed frame must never reach the radio as a fallback.
//
// nullptr (the default everywhere below) means "do not seal" -- the state
// every send path is in before it is configured, and the mode the native
// unit tests exercise deliberately (cleartext BTP framing). Real firmware
// always passes a real SealFn: RadioSeal::seal (key L, channel C) for
// StatusReporter and CommandProcessor::send_result's channel-C replies,
// RadioSeal::seal_e (key E, channel B) for CommandProcessor::send_result's
// channel-B replies.
using BtpSealFn = btp::EndpointSealFn;

// BTP/docs/encryption.md section 2: sealing always grows the payload by
// exactly this many octets, regardless of cipher.
constexpr std::size_t kBtpAeadTagSize = btp::kEndpointAeadTagSize;

// The robot's BTP transmit endpoint: btp::Endpoint (identity, the outgoing
// sequence counter and the seal -> fragment -> encode pipeline -- all now in
// BTP/src/endpoint.cpp, tested by BTP/tests/test_endpoint.cpp) plus the two
// things that stay project-local:
//
//   * a STORED send callback. btp::Endpoint takes the transport as a per-call
//     argument; every producer in this firmware sends the same way (through
//     TxScheduler, installed once in main.cpp), so the callback lives here and
//     the call sites do not thread it. The legacy no-context overload is kept
//     because the native tests install a bare `bool(*)(bytes, size)` capture.
//   * the ESP-NOW-only shape. Every frame this robot originates is a channel-C
//     or channel-B ESP-NOW datagram, so the wrappers below pin
//     btp::TransportProfile::EspNow and keep the old positional
//     (type, object_id, ...) signatures, so CommandProcessor / TelemetryPublisher
//     / StatusReporter / ManifestResponder / SubscriptionResponder / Logger /
//     TerminalResponder are untouched.
class BtpEndpoint : public btp::Endpoint {
public:
    using SealFn = BtpSealFn;
    using SendCallback = bool (*)(const std::uint8_t* data, std::size_t size);
    using ContextSendCallback = btp::EndpointSendFn;

    static constexpr std::size_t kAeadTagSize = btp::kEndpointAeadTagSize;

    // Bounds the sealed[] scratch a sealed send_logical() cuts fragments from.
    // The largest logical message this firmware sends is the UTF-8
    // system.monitor telemetry document
    // (TelemetryPublisher::kMaxSystemMonitorPayloadSize) -- a fragmented
    // COMMAND_RESULT / STATUS / MANIFEST_DATA never gets close.
    // RxRouter::kMaxPayloadSize and btp_command::kMaxLogicalRequestSize are
    // independent receive / command bounds and are NOT tied to this one.
    static constexpr std::size_t kMaxLogicalPayloadSize = 1920U;

    void set_send_callback(SendCallback callback) noexcept;
    void set_send_callback(ContextSendCallback callback, void* context) noexcept;

    // A sealed message is sealed ONCE over the canonical header, then sliced
    // for the wire from the sealed bytes; `payload` / `payload_size` are always
    // the PLAINTEXT. A false from `seal` fails the whole send closed.
    bool send_logical(btp::MessageType type, std::uint16_t object_id,
                      const std::uint8_t* payload, std::size_t payload_size,
                      std::uint64_t timestamp_us, SealFn seal = nullptr,
                      void* seal_context = nullptr) noexcept;

    // Same pipeline with a sequence a non-blocking producer already reserved.
    bool send_logical_reserved(btp::MessageType type, std::uint16_t object_id,
                               std::uint32_t sequence,
                               const std::uint8_t* payload,
                               std::size_t payload_size,
                               std::uint64_t timestamp_us, SealFn seal = nullptr,
                               void* seal_context = nullptr) const noexcept;

    // Exactly one physical frame. When `seal` is non-null this frame MUST be
    // the whole logical message (fragment_count == 1): the AEAD tag covers the
    // whole logical payload, never a slice of one.
    bool encode_fragment(btp::MessageType type, std::uint16_t object_id,
                         std::uint32_t sequence, std::uint64_t timestamp_us,
                         const std::uint8_t* payload, std::size_t payload_size,
                         std::uint8_t fragment_index,
                         std::uint8_t fragment_count, std::uint8_t* output,
                         std::size_t output_capacity, std::size_t* bytes_written,
                         SealFn seal = nullptr,
                         void* seal_context = nullptr) const noexcept;

    bool send_fragment(btp::MessageType type, std::uint16_t object_id,
                       std::uint32_t sequence, std::uint64_t timestamp_us,
                       const std::uint8_t* payload, std::size_t payload_size,
                       std::uint8_t fragment_index, std::uint8_t fragment_count,
                       SealFn seal = nullptr,
                       void* seal_context = nullptr) const noexcept;

    // Used by SD playback after a complete BTP frame has been decoded.
    bool send_encoded(const std::uint8_t* frame,
                      std::size_t frame_size) const noexcept;

private:
    static bool default_send(void*, const std::uint8_t*, std::size_t) noexcept;
    static bool legacy_send(void* context, const std::uint8_t* data,
                            std::size_t size) noexcept;

    std::atomic<SendCallback> legacy_callback_{nullptr};
    std::atomic<void*> send_context_{nullptr};
    std::atomic<ContextSendCallback> send_callback_{&default_send};
};

namespace btp_command {

constexpr std::uint16_t kCommandRequestObjectId = 0x0001U;
constexpr std::uint16_t kShellActionId = 0x0001U;
constexpr std::uint16_t kShellActionVersion = 0x0001U;
constexpr std::size_t kRequestPrefixSize = 20U;
constexpr std::size_t kMaxShellCommandSize = 512U;
constexpr std::size_t kMaxLogicalRequestSize =
    kRequestPrefixSize + kMaxShellCommandSize;

enum class ParseError : std::uint8_t {
    Ok,
    WrongType,
    WrongObject,
    InvalidEnvelope,
    PayloadTooShort,
    WrongTarget,
    InvalidAction,
    InvalidFlags,
    SizeMismatch,
    ParametersTooLarge,
    UnsupportedAction,
    InvalidShellText,
    OutputTooSmall
};

struct RequestView {
    std::uint32_t target_source_id;
    std::uint32_t target_boot_id;
    std::uint16_t action_id;
    std::uint16_t action_version;
    btp::ByteView parameters;
};

ParseError parse_request(const btp::Header& header,
                         btp::ByteView payload,
                         std::uint32_t local_source_id,
                         std::uint32_t local_boot_id,
                         RequestView* request_out) noexcept;

ParseError copy_shell_command(const RequestView& request,
                              char* output,
                              std::size_t output_capacity) noexcept;

std::uint32_t source_id_from_mac(const std::uint8_t mac[6]) noexcept;

// Radio-level filter only, NOT authentication -- see the long comment on the
// definition in BtpTransport.cpp. The claimed_source_id parameter this used
// to take was removed rather than left unused on purpose: an ignored
// parameter would let a call site keep passing a source_id and keep
// believing it is being checked, which is precisely the mistake this change
// makes possible. Removing it turns every call site into a compile error
// until someone reads why.
bool authorized_source(const std::uint8_t expected_mac[6],
                       const std::uint8_t received_mac[6]) noexcept;

const char* parse_error_string(ParseError error) noexcept;

}  // namespace btp_command

#endif  // BTP_TRANSPORT_H
