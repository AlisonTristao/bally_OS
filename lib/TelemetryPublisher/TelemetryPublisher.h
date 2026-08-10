#ifndef TELEMETRY_PUBLISHER_H
#define TELEMETRY_PUBLISHER_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <btp/codec.hpp>

class BtpEndpoint;

class TelemetryPublisher {
public:
    static constexpr std::uint16_t kProtocolTestTopicId = 0x0001U;
    static constexpr std::uint16_t kRobotStateTopicId = 0x0002U;
    static constexpr std::uint16_t kSchemaVersion = 1U;
    static constexpr std::size_t kQueueCapacity = 16U;
    static constexpr std::size_t kProtocolTestPayloadSize = 10U;
    static constexpr std::size_t kRobotStatePayloadSize = 3U;

    enum class Encoding : std::uint8_t { PackedLe = 0x05U };
    enum class WireType : std::uint8_t { Uint8, Uint32, Float32 };

    struct FieldSchema {
        std::uint16_t field_id;
        std::uint16_t order;
        const char* name;
        WireType type;
        const char* unit;
        float scale;
        float offset;
        std::uint16_t element_count;
        bool nullable;
    };

    struct TopicSchema {
        std::uint16_t topic_id;
        std::uint16_t schema_version;
        const char* name;
        Encoding encoding;
        const FieldSchema* fields;
        std::size_t field_count;
        std::size_t payload_size;
        // Declared publish rate for the manifest (COMMANDS_AND_ACTIONS.md
        // section 6.2's max_rate_millihz); zero means "not periodic" (e.g.
        // robot.state, published on transitions only).
        std::uint32_t max_rate_millihz;
    };

    enum class PublishResult : std::uint8_t {
        Queued,
        NotConfigured,
        QueueFull,
        InvalidValue,
        SequenceUnavailable,
    };

    struct Stats {
        std::uint32_t queued;
        std::uint32_t sent;
        std::uint32_t dropped_full;
        std::uint32_t dropped_invalid;
        std::uint32_t send_failed;
    };

    // Topico 17 (assinaturas e controle de taxa): one entry per row of
    // kSchemas, indexed the same way schemas() enumerates them. This is the
    // single place that tracks "does anyone currently want this topic and at
    // what rate" -- SubscriptionResponder only encodes/decodes the wire
    // payloads and calls into subscribe()/unsubscribe() here, so publish
    // behavior can never drift from what was actually granted (same pattern
    // ManifestResponder already uses against schemas()).
    struct TopicStats {
        std::uint16_t topic_id;
        std::uint16_t subscriber_count;  // 0 or 1: this leaf has exactly one
                                          // authorized peer (the dongle),
                                          // which already aggregates its own
                                          // downstream desktop clients.
        std::uint32_t effective_rate_millihz;
        std::uint64_t bytes_sent_total;
        std::uint64_t samples_dropped_total;
    };

    struct SubscribeOutcome {
        bool topic_known = false;
        std::uint32_t subscription_id = 0U;
        std::uint32_t effective_rate_millihz = 0U;
        std::uint32_t granted_lease_ms = 0U;
    };

    enum class UnsubscribeOutcome : std::uint8_t { Removed, NotFound, UnknownTopic };

    void configure(BtpEndpoint& endpoint) noexcept;

    // Applies the schema's max_rate_millihz cap (COMMANDS_AND_ACTIONS.md
    // section 6.2/7): effective_rate_millihz never exceeds the smaller of
    // requested_rate_millihz and the schema's max (when the schema is
    // periodic; a max of zero means "not periodic", so the request is
    // accepted but the rate stays informational and does not gate publish --
    // see robot.state). A repeat of the same (request_source_id,
    // request_boot_id, topic_id, requested_rate_millihz, requested_lease_ms)
    // while a matching subscription is still active returns the same
    // subscription_id (idempotent retry); anything else atomically replaces
    // the topic's single subscription with a new subscription_id. Unknown
    // topic_id returns topic_known=false and must not be turned into a wire
    // SUBSCRIBE_RESULT with a nonzero subscription_id.
    SubscribeOutcome subscribe(std::uint16_t topic_id,
                               std::uint32_t request_source_id,
                               std::uint32_t request_boot_id,
                               std::uint32_t requested_rate_millihz,
                               std::uint32_t requested_lease_ms,
                               std::uint64_t now_us) noexcept;

    UnsubscribeOutcome unsubscribe(std::uint32_t subscription_id) noexcept;

    // Clears any subscription whose lease has elapsed. PASSO 6: a robot that
    // stops hearing from the dongle (no renewed SUBSCRIBE) falls back to "not
    // publishing" for that topic instead of leaking a stale high rate
    // forever.
    void expire_subscriptions(std::uint64_t now_us) noexcept;

    // True when topic_id is periodic (nonzero schema max_rate_millihz) and
    // currently has a live, unexpired subscription. Non-periodic topics
    // (robot.state) are always "active" in the sense that their
    // event-triggered publish is never gated by subscription state.
    bool topic_active(std::uint16_t topic_id) const noexcept;

    // Publish period derived from the topic's effective rate, in
    // microseconds. Returns 0 when the topic is periodic and has no active
    // subscription (caller must not publish), or when the topic is unknown.
    std::uint64_t topic_period_us(std::uint16_t topic_id) const noexcept;

    std::size_t topic_stats(TopicStats* out, std::size_t max_count) const noexcept;

    // Producer side of a bounded SPSC queue. The control task is the sole
    // producer; flush() is called by the communication task. A full queue
    // drops the new sample immediately and never waits.
    PublishResult publish_protocol_test(std::uint32_t counter,
                                        float value,
                                        std::uint64_t timestamp_us) noexcept;
    PublishResult publish_robot_state(std::uint8_t state,
                                      std::uint64_t timestamp_us) noexcept;

    // Sends and removes at most max_samples. A radio rejection drops that
    // sample and is counted, so one bad sample cannot stall the queue.
    std::size_t flush(std::size_t max_samples) noexcept;

    std::size_t queued_count() const noexcept;
    Stats stats() const noexcept;

    static const TopicSchema* schemas(std::size_t* count) noexcept;
    static bool pack_protocol_test(std::uint32_t counter,
                                   float value,
                                   std::uint8_t output[kProtocolTestPayloadSize]) noexcept;
    static void pack_robot_state(
        std::uint8_t state,
        std::uint8_t output[kRobotStatePayloadSize]) noexcept;

private:
    static constexpr std::size_t kMaxPayloadSize = kProtocolTestPayloadSize;
    static constexpr std::size_t kMaxTopics = 2U;  // matches kSchemas today
    // Leases are requester-supplied but bounded locally so an abandoned
    // client (dongle rebooted without UNSUBSCRIBE) cannot pin a rate forever.
    static constexpr std::uint32_t kMinLeaseMs = 1000U;
    static constexpr std::uint32_t kMaxLeaseMs = 300000U;  // 5 minutes

    struct Sample {
        std::uint64_t timestamp_us;
        std::uint32_t sequence;
        std::uint16_t topic_id;
        std::uint16_t payload_size;
        std::uint8_t payload[kMaxPayloadSize];
    };

    // Runtime subscription/rate/byte-counter state for one topic row of
    // kSchemas (topico 17). Index-aligned with schemas(), not a separate map.
    struct TopicRuntime {
        std::uint16_t topic_id = 0U;
        bool subscribed = false;
        std::uint32_t subscription_id = 0U;
        std::uint32_t effective_rate_millihz = 0U;
        std::uint64_t lease_deadline_us = 0U;
        std::uint32_t request_source_id = 0U;
        std::uint32_t request_boot_id = 0U;
        std::uint32_t requested_rate_millihz = 0U;
        std::uint32_t requested_lease_ms = 0U;
        std::uint64_t bytes_sent_total = 0U;
        std::uint64_t samples_dropped_total = 0U;
    };

    PublishResult enqueue(std::uint16_t topic_id,
                          const std::uint8_t* payload,
                          std::size_t payload_size,
                          std::uint64_t timestamp_us) noexcept;
    void count_invalid() noexcept;
    int find_topic_index(std::uint16_t topic_id) const noexcept;
    void init_runtime_if_needed() noexcept;

    BtpEndpoint* endpoint_ = nullptr;
    Sample queue_[kQueueCapacity]{};
    TopicRuntime runtime_[kMaxTopics]{};
    bool runtime_initialized_ = false;
    std::uint32_t next_subscription_id_ = 1U;
    // Guards runtime_[] and next_subscription_id_ only. Independent of, and
    // never held across, anything touching BtpEndpoint/TxScheduler -- so it
    // cannot introduce a wait on the radio/priority path (PASSO 7). Writers
    // (subscribe/unsubscribe/expire_subscriptions, rare: ESP-NOW control rx
    // and a periodic sweep) and readers (topic_active/topic_period_us, one
    // check per sampleTelemetry() tick) use a short bounded spin rather than
    // blocking forever; a reader that loses the race treats the topic as
    // inactive for that one tick rather than stalling.
    mutable std::atomic_flag runtime_lock_ = ATOMIC_FLAG_INIT;
    std::atomic<std::uint32_t> write_index_{0U};
    std::atomic<std::uint32_t> read_index_{0U};
    std::atomic<std::uint32_t> queued_total_{0U};
    std::atomic<std::uint32_t> sent_total_{0U};
    std::atomic<std::uint32_t> dropped_full_{0U};
    std::atomic<std::uint32_t> dropped_invalid_{0U};
    std::atomic<std::uint32_t> send_failed_{0U};
};

#endif  // TELEMETRY_PUBLISHER_H
