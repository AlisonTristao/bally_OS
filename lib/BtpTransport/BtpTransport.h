#ifndef BTP_TRANSPORT_H
#define BTP_TRANSPORT_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <btp/codec.hpp>

// AEAD sealer for channel C (dongle<->robot, key L -- see
// include/bally_channels.h and lib/RadioSeal, the only real implementation
// of this). Mirrors bally_dongle's BtpTransport::SealFn exactly, so the
// shape below is not a local invention. File scope, not a member of
// BtpEndpoint, on purpose: StatusReporter.h and CommandProcessor.h need to
// name this type while only forward-declaring `class BtpEndpoint`, which a
// nested member typedef would not survive.
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
using BtpSealFn = bool (*)(void* context, const btp::Header& header,
                           std::uint16_t payload_size,
                           const std::uint8_t* plaintext, std::uint8_t* out);

// BTP/docs/encryption.md section 2: sealing always grows the payload by
// exactly this many octets, regardless of cipher.
constexpr std::size_t kBtpAeadTagSize = 16U;

class BtpEndpoint {
public:
    using SendCallback = bool (*)(const std::uint8_t* data, std::size_t size);
    using ContextSendCallback = bool (*)(void* context,
                                         const std::uint8_t* data,
                                         std::size_t size);
    using SealFn = BtpSealFn;
    static constexpr std::size_t kAeadTagSize = kBtpAeadTagSize;

    // Bounds the sealed[] scratch buffer send_logical uses when sealing.
    // Matches RxRouter::kMaxPayloadSize / btp_command::kMaxLogicalRequestSize
    // -- the largest logical message this firmware ever sends today (a
    // fragmented COMMAND_RESULT/STATUS never gets close).
    static constexpr std::size_t kMaxLogicalPayloadSize = 600U;

    bool configure(std::uint32_t source_id, std::uint32_t boot_id) noexcept;
    void set_send_callback(SendCallback callback) noexcept;
    void set_send_callback(ContextSendCallback callback, void* context) noexcept;

    std::uint32_t source_id() const noexcept { return source_id_; }
    std::uint32_t boot_id() const noexcept { return boot_id_; }

    // A sequence is reserved once, when the logical message is created.
    // Zero is never returned and becomes the permanent exhausted sentinel.
    bool reserve_sequence(std::uint32_t* sequence_out) noexcept;

    // Single-CAS variant for hard non-blocking producers. It may fail under
    // contention even when sequences remain; callers should drop/count rather
    // than retry on a time-critical task.
    bool try_reserve_sequence(std::uint32_t* sequence_out) noexcept;

    // Seals (when `seal` is non-null) before fragmenting -- see SealFn's
    // contract on why fragment_count has to be computed from the sealed
    // size, not the plaintext size. `payload`/`payload_size` are always the
    // PLAINTEXT; callers never seal for themselves.
    bool send_logical(btp::MessageType type,
                      std::uint16_t object_id,
                      const std::uint8_t* payload,
                      std::size_t payload_size,
                      std::uint64_t timestamp_us,
                      SealFn seal = nullptr,
                      void* seal_context = nullptr) noexcept;

    // Encodes exactly one physical frame. When `seal` is non-null this frame
    // MUST be the whole logical message (fragment_count == 1): the AEAD tag
    // covers the whole logical payload, never a slice of one, so sealing a
    // multi-fragment call here is refused rather than silently wrong.
    bool encode_fragment(btp::MessageType type,
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
                         SealFn seal = nullptr,
                         void* seal_context = nullptr) const noexcept;

    bool send_fragment(btp::MessageType type,
                       std::uint16_t object_id,
                       std::uint32_t sequence,
                       std::uint64_t timestamp_us,
                       const std::uint8_t* payload,
                       std::size_t payload_size,
                       std::uint8_t fragment_index,
                       std::uint8_t fragment_count,
                       SealFn seal = nullptr,
                       void* seal_context = nullptr) const noexcept;

    // Used by SD playback after a complete BTP frame has been decoded.
    bool send_encoded(const std::uint8_t* frame, std::size_t frame_size) const noexcept;

private:
    static bool default_send(void*, const std::uint8_t*, std::size_t) noexcept;
    static bool legacy_send(void* context, const std::uint8_t* data,
                            std::size_t size) noexcept;

    std::uint32_t source_id_ = 0;
    std::uint32_t boot_id_ = 0;
    std::atomic<std::uint32_t> next_sequence_{0};
    std::atomic<SendCallback> legacy_callback_{nullptr};
    std::atomic<void*> send_context_{nullptr};
    std::atomic<ContextSendCallback> send_callback_{default_send};
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
