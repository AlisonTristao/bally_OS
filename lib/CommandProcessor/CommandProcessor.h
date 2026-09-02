#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <BtpTransport.h>
#include <bally_channels.h>
#include <btp/session.hpp>

class CommandProcessor {
public:
    static constexpr std::size_t kCacheCapacity = 16U;
    // Distinct requester devices whose deduplication high-water marks are
    // tracked at once: the dongle on channel C, plus a few desktop clients
    // relayed on channel B. A new boot_id from a known source reuses its row
    // (btp::DedupCache), so dongle reboots do not consume extra rows.
    static constexpr std::size_t kRequesterCapacity = 4U;
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

    CommandProcessor() noexcept;

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
    // btp::DedupCache stores the verbatim request then the COMMAND_RESULT in
    // one region per slot. The result is prefixed with the 12 octets the
    // replay needs that the payload does not carry -- the result frame's own
    // sequence and timestamp -- so a retransmission is answered byte for byte.
    static constexpr std::size_t kReplayPrefix = 12U;  // sequence:u32 + timestamp_us:u64
    static constexpr std::size_t kSlotStorageBytes =
        btp_command::kMaxLogicalRequestSize + kReplayPrefix + kMaxResultPayloadSize;

    // Per reserved slot, the fields complete()/reject_busy() need to encode a
    // COMMAND_RESULT and that btp::DedupCache does not surface. Rewritten on
    // every Fresh verdict, so it always matches the slot's current occupant.
    struct SlotMeta {
        btp::DedupKey key{};
        std::uint16_t action_id = 0U;
        std::uint16_t action_version = 0U;
        bally::Channel channel = bally::Channel::C_Link;
    };

    // The one field peek kept out of btp::decode_command_request: action_id /
    // action_version are needed for the dedup-conflict and cache-exhausted
    // transient results, which are built before the request is parsed.
    static std::uint16_t read_u16(const std::uint8_t* input) noexcept;
    static std::uint32_t read_u32(const std::uint8_t* input) noexcept;
    static std::uint64_t read_u64(const std::uint8_t* input) noexcept;
    static void write_u32(std::uint8_t* output, std::uint32_t value) noexcept;
    static void write_u64(std::uint8_t* output, std::uint64_t value) noexcept;
    static const char* parse_message(btp_command::ParseError error) noexcept;
    static Status parse_status(btp_command::ParseError error) noexcept;
    static ErrorCode parse_error_code(btp_command::ParseError error) noexcept;

    btp::DedupCache bind_dedup() noexcept;

    // Encode a COMMAND_RESULT (docs/commands.md 2.4) via
    // btp::encode_command_result. A shell result never carries a bytes_u32
    // result body -- only the message.
    bool encode_result(const btp::DedupKey& key, std::uint16_t action_id,
                       std::uint16_t action_version, Status status,
                       ErrorCode error, const char* message,
                       std::uint8_t* output, std::size_t capacity,
                       std::uint16_t* size_out) noexcept;

    // complete()/reject_busy()/the parse-failure path: reserve a sequence,
    // encode the result, prefix it with (sequence, now_us), store it in the
    // dedup slot for replay, and hand back a ResultView over `view_buf` (a
    // stable per-task buffer -- never the slot storage, which can be evicted
    // while send_result runs).
    bool finish(std::size_t slot, Status status, ErrorCode error,
                const char* message, std::uint64_t now_us,
                std::uint8_t* view_buf, ResultView* result_out) noexcept;

    // Conflict / cache-exhausted / evicted: a reply that is NOT cached (it is
    // not a completed execution). Built into rx_view_.
    bool make_transient(const btp::DedupKey& key, std::uint16_t action_id,
                        std::uint16_t action_version, Status status,
                        ErrorCode error, const char* message,
                        std::uint64_t now_us, bally::Channel channel,
                        ResultView* result_out) noexcept;

    // Unpack a stored (sequence, timestamp_us, payload) blob into a ResultView
    // whose payload is copied into rx_view_.
    ResultView replay_view(btp::ByteView stored, bally::Channel channel) noexcept;

    BtpEndpoint* endpoint_ = nullptr;
    BtpSealFn seal_link_ = nullptr;
    void* seal_link_context_ = nullptr;
    BtpSealFn seal_endpoint_ = nullptr;
    void* seal_endpoint_context_ = nullptr;
    std::atomic_flag cache_lock_ = ATOMIC_FLAG_INIT;

    btp::DedupSlot dedup_slots_[kCacheCapacity]{};
    std::uint8_t dedup_bytes_[kCacheCapacity][kSlotStorageBytes]{};
    btp::DedupStorage dedup_storage_[kCacheCapacity]{};
    btp::DedupRequester dedup_requesters_[kRequesterCapacity]{};
    btp::DedupCache dedup_;
    SlotMeta slot_meta_[kCacheCapacity]{};

    // Stable payload storage for a returned ResultView. rx_view_ backs every
    // reply built on the RX task (intake's replay/conflict/exhausted results
    // and reject_busy); shell_view_ backs complete()'s, on the shell task.
    // The two tasks never share a buffer, and each rebuilds its own before the
    // next send_result on that task. Never point a ResultView at the dedup
    // slot storage: a later intake can evict it while send_result runs.
    std::uint8_t rx_view_[kMaxResultPayloadSize]{};
    std::uint8_t shell_view_[kMaxResultPayloadSize]{};

    std::atomic<std::uint32_t> accepted_{0U};
    std::atomic<std::uint32_t> executed_{0U};
    std::atomic<std::uint32_t> duplicates_{0U};
    std::atomic<std::uint32_t> replayed_{0U};
    std::atomic<std::uint32_t> rejected_{0U};
    std::atomic<std::uint32_t> dropped_{0U};
    std::atomic<std::uint32_t> unauthorized_{0U};
};

#endif  // COMMAND_PROCESSOR_H
