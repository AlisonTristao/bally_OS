#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <BtpTransport.h>

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

    void configure(BtpEndpoint& endpoint) noexcept;

    Intake intake(const btp::Header& header, btp::ByteView payload,
                  std::uint64_t now_us) noexcept;

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
        std::uint16_t request_size = 0U;
        std::uint8_t request[btp_command::kMaxLogicalRequestSize]{};
        std::uint32_t result_sequence = 0U;
        std::uint64_t result_timestamp_us = 0U;
        std::uint16_t result_size = 0U;
        std::uint8_t result[kMaxResultPayloadSize]{};
    };

    static bool same_key(const RequestKey& left,
                         const RequestKey& right) noexcept;
    static void write_u16(std::uint8_t* output, std::uint16_t value) noexcept;
    static void write_u32(std::uint8_t* output, std::uint32_t value) noexcept;
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
                        std::uint64_t now_us, ResultView* result_out) noexcept;
    bool encode_result(const RequestKey& key, std::uint16_t action_id,
                       std::uint16_t action_version, Status status,
                       ErrorCode error, const char* message,
                       std::uint8_t* output, std::size_t capacity,
                       std::uint16_t* size_out) noexcept;

    BtpEndpoint* endpoint_ = nullptr;
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
