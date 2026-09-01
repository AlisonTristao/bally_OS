#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <BtpTransport.h>
#include <bally_channels.h>

class CommandProcessor {
public:
    static constexpr std::size_t kCacheCapacity = 16U;
    static constexpr std::uint16_t kCommandResultObjectId = 0x0002U;
    static constexpr std::size_t kMaxResultPayloadSize = 128U;

    enum class IntakeKind : std::uint8_t {
        Ready,
        DuplicateInProgress,
        ResultReady,
        Ignored,
    };

    enum class Status : std::uint8_t {
        Success = 0x00U,
        Rejected = 0x01U,
        Failed = 0x02U,
        Timeout = 0x03U,
        Cancelled = 0x04U,
        Unsupported = 0x05U,
        Busy = 0x06U,
    };

    enum class ErrorCode : std::uint16_t {
        None = 0x0000U,
        MalformedPayload = 0x0001U,
        UnknownObject = 0x0002U,
        InvalidArgument = 0x0003U,
        NotAuthorized = 0x0004U,
        CapacityExhausted = 0x0005U,
        ExecutionTimeout = 0x0006U,
        InternalError = 0x0007U,
        UnsupportedVersion = 0x0008U,
        StaleTargetBoot = 0x0009U,
        RequestConflict = 0x000AU,
    };

    struct WorkItem {
        std::uint8_t cache_slot = 0U;
        char command[btp_command::kMaxShellCommandSize + 1U]{};
    };

    struct ResultView {
        std::uint32_t sequence = 0U;
        std::uint64_t timestamp_us = 0U;
        const std::uint8_t* payload = nullptr;
        std::size_t payload_size = 0U;
        // Which channel the ORIGINAL request arrived on -- send_result()
        // seals the reply with that channel's key, never the other one.
        bally::Channel channel = bally::Channel::C_Link;
    };

    struct Intake {
        IntakeKind kind = IntakeKind::Ignored;
        WorkItem work{};
        ResultView result{};
    };

    struct Stats {
        std::uint32_t accepted;
        std::uint32_t executed;
        std::uint32_t duplicates;
        std::uint32_t replayed;
        std::uint32_t rejected;
        std::uint32_t dropped;
        std::uint32_t unauthorized;
    };

    // `seal_link`/`seal_endpoint` (both default nullptr) are forwarded
    // verbatim to BtpEndpoint::send_fragment -- see BtpSealFn. Each reply is
    // sealed with whichever one matches the ORIGINAL request's channel (see
    // ResultView::channel and bally_channels.h): a channel-C request
    // (dongle, key L) is answered with seal_link, a channel-B request
    // (TraceView, key E) with seal_endpoint. send_result() fails closed
    // rather than seal a reply with the other channel's key, or send it in
    // the clear, whenever at least one of the two has been configured --
    // the real callers pass RadioSeal::seal and RadioSeal::seal_e.
    //
    // Leaving both nullptr (the default) is not "channel B unsupported", it
    // is "no encryption at all": every reply goes out unsealed regardless of
    // channel, exactly as before this parameter existed. That is what the
    // native unit tests exercise -- BTP framing over cleartext -- and is not
    // a mode real firmware should configure.
    void configure(BtpEndpoint& endpoint, BtpSealFn seal_link = nullptr,
                   void* seal_link_context = nullptr,
                   BtpSealFn seal_endpoint = nullptr,
                   void* seal_endpoint_context = nullptr) noexcept;

    // `channel` is the caller's classification of the request (see
    // bally_channels.h::channel_of_peer), made BEFORE this call since only
    // the caller has the header's cleartext source_id and knows which key
    // opened the payload. Defaults to C_Link so a caller that never passes
    // it -- every existing native test -- keeps today's behavior verbatim.
    Intake intake(const btp::Header& header, btp::ByteView payload,
                  std::uint64_t now_us,
                  bally::Channel channel = bally::Channel::C_Link) noexcept;

    // Completes a previously returned WorkItem. TinyShell uses zero for
    // success; every other code becomes FAILED/INTERNAL_ERROR.
    bool complete(std::uint8_t cache_slot, std::uint8_t shell_status,
                  std::uint64_t now_us, ResultView* result_out) noexcept;

    // Used when the FreeRTOS execution queue is full after the dedup slot was
    // reserved. The request stays cached and a retry replays this BUSY result.
    bool reject_busy(std::uint8_t cache_slot, std::uint64_t now_us,
                     ResultView* result_out) noexcept;

    bool send_result(const ResultView& result) noexcept;
    void note_unauthorized() noexcept;
    void note_drop() noexcept;
    Stats stats() const noexcept;

private:
    struct RequestKey {
        std::uint32_t source_id = 0U;
        std::uint32_t boot_id = 0U;
        std::uint32_t sequence = 0U;
    };

    struct CacheEntry {
        bool used = false;
        bool complete = false;
        RequestKey key{};
        std::uint16_t action_id = 0U;
        std::uint16_t action_version = 0U;
        bally::Channel channel = bally::Channel::C_Link;
        std::uint16_t request_size = 0U;
        std::uint8_t request[btp_command::kMaxLogicalRequestSize]{};
        std::uint32_t result_sequence = 0U;
        std::uint64_t result_timestamp_us = 0U;
        std::uint16_t result_size = 0U;
        std::uint8_t result[kMaxResultPayloadSize]{};
    };

    static bool same_key(const RequestKey& left,
                         const RequestKey& right) noexcept;
    // The one field peek kept out of btp::decode_command_request: action_id /
    // action_version are needed for the dedup-conflict and cache-exhausted
    // transient results, which are built before the request is parsed.
    static std::uint16_t read_u16(const std::uint8_t* input) noexcept;
    static const char* parse_message(btp_command::ParseError error) noexcept;
    static Status parse_status(btp_command::ParseError error) noexcept;
    static ErrorCode parse_error_code(btp_command::ParseError error) noexcept;

    ResultView view(const CacheEntry& entry) const noexcept;
    bool finish(CacheEntry& entry, Status status, ErrorCode error,
                const char* message, std::uint64_t now_us,
                ResultView* result_out) noexcept;
    bool make_transient(const RequestKey& key, std::uint16_t action_id,
                        std::uint16_t action_version, Status status,
                        ErrorCode error, const char* message,
                        std::uint64_t now_us, bally::Channel channel,
                        ResultView* result_out) noexcept;
    bool encode_result(const RequestKey& key, std::uint16_t action_id,
                       std::uint16_t action_version, Status status,
                       ErrorCode error, const char* message,
                       std::uint8_t* output, std::size_t capacity,
                       std::uint16_t* size_out) noexcept;

    BtpEndpoint* endpoint_ = nullptr;
    BtpSealFn seal_link_ = nullptr;
    void* seal_link_context_ = nullptr;
    BtpSealFn seal_endpoint_ = nullptr;
    void* seal_endpoint_context_ = nullptr;
    std::atomic_flag cache_lock_ = ATOMIC_FLAG_INIT;
    CacheEntry cache_[kCacheCapacity]{};
    std::uint8_t transient_result_[kMaxResultPayloadSize]{};
    std::uint32_t transient_sequence_ = 0U;
    std::uint64_t transient_timestamp_us_ = 0U;
    std::uint16_t transient_size_ = 0U;

    std::atomic<std::uint32_t> accepted_{0U};
    std::atomic<std::uint32_t> executed_{0U};
    std::atomic<std::uint32_t> duplicates_{0U};
    std::atomic<std::uint32_t> replayed_{0U};
    std::atomic<std::uint32_t> rejected_{0U};
    std::atomic<std::uint32_t> dropped_{0U};
    std::atomic<std::uint32_t> unauthorized_{0U};
};

#endif  // COMMAND_PROCESSOR_H
