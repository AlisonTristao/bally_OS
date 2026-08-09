#ifndef TX_SCHEDULER_H
#define TX_SCHEDULER_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <btp/codec.hpp>

class TxScheduler {
public:
    using RadioSendCallback = bool (*)(void* context,
                                       const std::uint8_t* data,
                                       std::size_t size);

    enum class Priority : std::uint8_t {
        CommandResult = 0,
        Status = 1,
        CriticalLog = 2,
        Telemetry = 3,
        Debug = 4,
        Count = 5,
    };

    struct Stats {
        std::uint32_t accepted;
        std::uint32_t delivered;
        std::uint32_t timeouts;
        std::uint32_t dropped;
        std::uint32_t delivery_failed;
        std::uint32_t queued_by_priority[5];
        std::uint32_t dropped_by_priority[5];
    };

    static constexpr std::uint64_t kDefaultDeliveryTimeoutMs = 250U;

    void configure(RadioSendCallback callback, void* context,
                   std::uint64_t delivery_timeout_ms =
                       kDefaultDeliveryTimeoutMs) noexcept;

    // Adapter installed as BtpEndpoint's send callback. It validates and
    // copies the complete encoded frame into a queue dedicated to its class.
    static bool enqueue_callback(void* context, const std::uint8_t* data,
                                 std::size_t size) noexcept;
    bool enqueue(const std::uint8_t* data, std::size_t size) noexcept;

    // Starts at most one ESP-NOW transmission. Only one frame may await a
    // callback, which makes callback correlation unambiguous on IDF versions
    // that do not expose a per-send token.
    bool pump(std::uint64_t now_ms) noexcept;
    void on_delivery(bool delivered) noexcept;

    bool idle() const noexcept;
    std::size_t queued_count(Priority priority) const noexcept;
    Stats stats() const noexcept;

private:
    static constexpr std::size_t kPriorityCount =
        static_cast<std::size_t>(Priority::Count);
    static constexpr std::size_t kQueueCapacity[kPriorityCount] = {
        8U, 4U, 8U, 16U, 8U,
    };
    static constexpr std::size_t kMaxCapacity = 16U;

    struct EncodedFrame {
        std::uint16_t size = 0U;
        std::uint8_t bytes[btp::kEspNowMaxFrameSize]{};
    };

    struct Queue {
        EncodedFrame frames[kMaxCapacity]{};
        std::size_t read = 0U;
        std::size_t write = 0U;
        std::size_t count = 0U;
        mutable std::atomic_flag lock = ATOMIC_FLAG_INIT;
    };

    static Priority classify(const btp::DecodedFrame& frame) noexcept;
    bool pop(Priority priority, EncodedFrame* output) noexcept;
    static std::size_t index(Priority priority) noexcept {
        return static_cast<std::size_t>(priority);
    }

    Queue queues_[kPriorityCount]{};
    RadioSendCallback radio_send_ = nullptr;
    void* radio_context_ = nullptr;
    std::uint64_t delivery_timeout_ms_ = kDefaultDeliveryTimeoutMs;

    EncodedFrame pending_{};
    std::uint64_t pending_since_ms_ = 0U;
    std::atomic<bool> awaiting_delivery_{false};
    // 0=no callback, 1=success, 2=failure. Written by the Wi-Fi callback and
    // consumed by pump(); the callback never locks or starts another send.
    std::atomic<std::uint8_t> delivery_event_{0U};

    std::atomic<std::uint32_t> accepted_{0U};
    std::atomic<std::uint32_t> delivered_{0U};
    std::atomic<std::uint32_t> timeouts_{0U};
    std::atomic<std::uint32_t> dropped_{0U};
    std::atomic<std::uint32_t> delivery_failed_{0U};
    std::atomic<std::uint32_t> queued_by_priority_[kPriorityCount]{};
    std::atomic<std::uint32_t> dropped_by_priority_[kPriorityCount]{};
};

#endif  // TX_SCHEDULER_H
