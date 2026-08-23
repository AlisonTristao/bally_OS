#ifndef BTP_TRANSPORT_H
#define BTP_TRANSPORT_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <btp/codec.hpp>

class BtpEndpoint {
public:
    using SendCallback = bool (*)(const std::uint8_t* data, std::size_t size);
    using ContextSendCallback = bool (*)(void* context,
                                         const std::uint8_t* data,
                                         std::size_t size);

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

    bool send_logical(btp::MessageType type,
                      std::uint16_t object_id,
                      const std::uint8_t* payload,
                      std::size_t payload_size,
                      std::uint64_t timestamp_us) noexcept;

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
                         std::size_t* bytes_written) const noexcept;

    bool send_fragment(btp::MessageType type,
                       std::uint16_t object_id,
                       std::uint32_t sequence,
                       std::uint64_t timestamp_us,
                       const std::uint8_t* payload,
                       std::size_t payload_size,
                       std::uint8_t fragment_index,
                       std::uint8_t fragment_count) const noexcept;

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
