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
        // Interactive shell I/O (TERMINAL_IN/OUT). Kept above the log/
        // telemetry/debug bulk and its own queue, so a command's mirrored
        // output does not fight for the 8-deep Debug queue with every LOG
        // line the same command emits -- pump() sends one frame per pass at
        // strict priority, and Debug is last.
        Terminal = 2,
        CriticalLog = 3,
        Telemetry = 4,
        Debug = 5,
        Count = 6,
    };

    struct Stats {
        std::uint32_t accepted;
        std::uint32_t delivered;
        std::uint32_t timeouts;
        std::uint32_t dropped;
        std::uint32_t delivery_failed;
        std::uint32_t queued_by_priority[6];
        std::uint32_t dropped_by_priority[6];
        // Telemetry's own fire-and-forget lane (pump_telemetry()) -- not
        // folded into accepted/delivered/dropped above, which only ever
        // reflect the confirm-gated classes pump() drives.
        std::uint32_t telemetry_sent;
        std::uint32_t telemetry_dropped;
    };

    static constexpr std::uint64_t kDefaultDeliveryTimeoutMs = 250U;

    void configure(RadioSendCallback callback, void* context,
                   std::uint64_t delivery_timeout_ms =
                       kDefaultDeliveryTimeoutMs) noexcept;

    // Second, independent send path used ONLY by the Telemetry priority
    // class -- see pump_telemetry(). Kept separate from configure() above
    // because telemetry fire-and-forgets to a different destination
    // (ESP-NOW broadcast, not the paired dongle's unicast peer) and never
    // waits for delivery, so it has no business sharing pump()'s
    // pending_/awaiting_delivery_ state machine.
    void configure_telemetry(RadioSendCallback callback,
                             void* context) noexcept;

    // Adapter installed as BtpEndpoint's send callback. It validates and
    // copies the complete encoded frame into a queue dedicated to its class.
    static bool enqueue_callback(void* context, const std::uint8_t* data,
                                 std::size_t size) noexcept;
    bool enqueue(const std::uint8_t* data, std::size_t size) noexcept;

    // Starts at most one ESP-NOW transmission, for every class EXCEPT
    // Telemetry (see pump_telemetry() for that one). Only one frame may
    // await a callback, which makes callback correlation unambiguous on IDF
    // versions that do not expose a per-send token.
    bool pump(std::uint64_t now_ms) noexcept;
    void on_delivery(bool delivered) noexcept;

    // Drains the Telemetry queue directly, bypassing pending_/
    // awaiting_delivery_ entirely: telemetry is a state stream (a newer
    // sample always supersedes an older one, and BTP's sequence field
    // already exposes gaps downstream), so unlike pump() it never waits for
    // a delivery confirmation or times out waiting for one -- it sends
    // everything queued, back to back, until the queue empties or the radio
    // itself pushes back (telemetry_radio_send_ returning false -- e.g. the
    // driver's own TX queue is full). Meant to be driven by its own
    // high-rate esp_timer, independent of pump()'s tick-driven cadence.
    bool pump_telemetry() noexcept;

    bool idle() const noexcept;
    std::size_t queued_count(Priority priority) const noexcept;
    Stats stats() const noexcept;

private:
    static constexpr std::size_t kPriorityCount =
        static_cast<std::size_t>(Priority::Count);
    // Status used to sit at 4 back when CONTROL meant only single-frame
    // STATUS heartbeats. classify() (TxScheduler.cpp) also routes
    // MANIFEST_DATA here (also CONTROL), and BallyRobot::kMaxManifestScratchBytes
    // (1900 plaintext + 16 AEAD tag = 1916) can need up to
    // ceil(1916 / btp::kEspNowMaxPayloadSize) = 10 ESP-NOW fragments --
    // Endpoint::send_logical_impl's fragmentation loop aborts the whole send
    // the instant one fragment fails to enqueue (no partial retry), so a
    // 4-deep queue silently dropped fragments 4-8 of any manifest big enough
    // to need more than 4, and the robot's catalog never reached the dongle
    // (see this repo's manifest-capacity fix; ManifestCatalog measured 1710
    // plaintext / 1726 sealed / 9 fragments once robot.sensors/robot.flags
    // existed -- already past 4). 10 would match that scratch ceiling exactly,
    // but Status is shared with STATUS heartbeats and SUBSCRIBE_RESULT/
    // UNSUBSCRIBE_RESULT, either of which landing mid-manifest-send eats back
    // into that same margin -- and every Queue below already reserves
    // kMaxCapacity frames regardless of its priority's cap, so rounding up to
    // the full 16 instead of the bare per-message minimum costs no additional
    // RAM either way.
    static constexpr std::size_t kQueueCapacity[kPriorityCount] = {
        //  cmd  status  terminal  crit-log  telemetry  debug
        8U,  16U,     16U,      8U,       16U,       8U,
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

    // pump_telemetry()'s own send path -- deliberately not shared with
    // radio_send_/radio_context_ above, since it targets a different
    // destination (broadcast) and is never gated on delivery.
    RadioSendCallback telemetry_radio_send_ = nullptr;
    void* telemetry_radio_context_ = nullptr;
    std::atomic<std::uint32_t> telemetry_sent_{0U};
    std::atomic<std::uint32_t> telemetry_dropped_{0U};

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
