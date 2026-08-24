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

    // Reserved subscriber identity for a subscription created ON the robot
    // (the "telemetry -sub" shell command) rather than by a SUBSCRIBE arriving
    // over the radio.
    //
    // Subscriptions are keyed by (source_id, boot_id, topic_id), and that pair
    // normally comes from the BTP envelope. A shell caller has no envelope, so
    // it needs an identity that no real peer can present -- source_id derives
    // from a factory MAC and is never zero, which is what makes zero safe to
    // claim here. Without it, the dongle's own drop_session() on reconnect
    // would take the bench's subscriptions down along with its own.
    static constexpr std::uint32_t kLocalSubscriberSourceId = 0U;
    static constexpr std::uint32_t kLocalSubscriberBootId = 0U;
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
        // Declared publish rate for the manifest (commands.md section 3.3's
        // max_rate_millihz); zero means "not periodic" (e.g.
        // robot.state, published on transitions only).
        std::uint32_t max_rate_millihz;
        // Topico 17 PASSO 4 ("robo aplica min/max/default do schema"). Only
        // max_rate_millihz travels on the wire (section 3.3 topic record);
        // these two are local publisher policy and are deliberately NOT
        // serialized into the manifest, because v1 has no field for them and
        // inventing one would be a wire change.
        //
        // min_rate_millihz: slowest rate this publisher is willing to
        //   schedule. A slower request is REJECTED (INVALID_ARGUMENT) instead
        //   of being silently sped up, because section 4 says the effective
        //   rate MUST NOT exceed the requested one -- clamping *up* is not
        //   allowed, so the only honest answers are "reject" or "publish
        //   slower than the client can use". Zero disables the floor.
        // default_rate_millihz: nominal rate, used as the cap when the topic
        //   declares no periodic max (max_rate_millihz == 0). For such a
        //   topic the granted rate is informational only (delivery stays
        //   event-driven), but capping it keeps SUBSCRIBE_RESULT from echoing
        //   an arbitrary client number back as if the robot had promised it.
        //   Zero means "no cap for a non-periodic topic".
        std::uint32_t min_rate_millihz;
        std::uint32_t default_rate_millihz;
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
        // Number of live (unexpired) subscriptions for this topic. The robot
        // has a single authorized ESP-NOW peer, but that peer is a gateway
        // that may forward several independent desktop sessions, so this is
        // a real aggregate over (subscriber session, topic_id) and not a
        // 0/1 flag.
        std::uint16_t subscriber_count;
        // Aggregate publish rate: the maximum granted rate over the live
        // subscriptions of this topic. Zero means "not being published now"
        // (STATUS section 5.1 defines the field exactly that way).
        std::uint32_t effective_rate_millihz;
        std::uint64_t bytes_sent_total;
        std::uint64_t samples_dropped_total;
    };

    struct SubscribeOutcome {
        bool topic_known = false;
        // Requested rate resolved below the schema's min_rate_millihz; the
        // caller turns this into REJECTED/INVALID_ARGUMENT.
        bool rate_below_minimum = false;
        // Subscription table full; the caller turns this into
        // REJECTED/CAPACITY_EXHAUSTED.
        bool capacity_exhausted = false;
        std::uint32_t subscription_id = 0U;
        std::uint32_t effective_rate_millihz = 0U;
        std::uint32_t granted_lease_ms = 0U;
    };

    struct RateResolution {
        bool below_minimum = false;
        std::uint32_t effective_rate_millihz = 0U;
    };

    enum class UnsubscribeOutcome : std::uint8_t { Removed, NotFound, UnknownTopic };

    void configure(BtpEndpoint& endpoint) noexcept;

    // Pure rate policy, exposed for testing: applies min/default/max of the
    // schema to a requested rate (see TopicSchema above). Never returns a
    // rate above `requested_rate_millihz`.
    static RateResolution resolve_effective_rate(
        const TopicSchema& schema, std::uint32_t requested_rate_millihz) noexcept;

    static const TopicSchema* find_schema(std::uint16_t topic_id) noexcept;

    // Creates, renews or replaces the subscription of one subscriber session
    // for one topic. A subscription is keyed by
    // (request_source_id, request_boot_id, topic_id) -- the session identity
    // of commands.md section 4 -- so several sessions can hold
    // independent subscriptions to the same topic and the topic only stops
    // being published when the *last* one goes away (PASSO 5).
    //
    // Repeating the same request bytes from the same identity returns the
    // same subscription_id and only pushes the lease deadline forward
    // ("repeating the identical request returns the same subscription rather
    // than creating another"); different bytes atomically replace that
    // session's subscription for that topic with a new subscription_id ("a
    // new sequence atomically creates or replaces").
    //
    // PASSO 6 (session disconnect): a SUBSCRIBE from a source_id whose
    // boot_id changed means the previous session of that peer is gone, so
    // every subscription still held by the old boot_id is dropped here
    // before the new one is granted -- a rebooted client never leaves the
    // robot publishing for a session that no longer exists.
    SubscribeOutcome subscribe(std::uint16_t topic_id,
                               std::uint32_t request_source_id,
                               std::uint32_t request_boot_id,
                               std::uint32_t requested_rate_millihz,
                               std::uint32_t requested_lease_ms,
                               std::uint64_t now_us) noexcept;

    UnsubscribeOutcome unsubscribe(std::uint32_t subscription_id) noexcept;

    // Drops every subscription held by one subscriber session. PASSO 6's
    // explicit disconnect path: the ESP-NOW leg has no SESSION_CLOSE (that
    // exchange belongs to the dongle's serial transport,
    // session-and-terminal.md section 4), so on this side a session ends
    // either by lease expiry, by the peer coming back with a new boot_id, or
    // by this call. Returns how many subscriptions were removed.
    std::size_t drop_session(std::uint32_t subscriber_source_id,
                             std::uint32_t subscriber_boot_id) noexcept;

    // Clears any subscription whose lease has elapsed. PASSO 6: a robot that
    // stops hearing from the dongle (no renewed SUBSCRIBE) falls back to "not
    // publishing" for that topic instead of leaking a stale high rate
    // forever.
    void expire_subscriptions(std::uint64_t now_us) noexcept;

    // True when topic_id currently has at least one live, unexpired
    // subscription. Non-periodic topics (robot.state) are published on
    // events regardless of this, so callers only gate periodic topics on it.
    bool topic_active(std::uint16_t topic_id) const noexcept;

    // Live subscription count for one topic (0 when unknown/unsubscribed).
    std::uint16_t topic_subscriber_count(std::uint16_t topic_id) const noexcept;

    // Aggregate publish rate for one topic: the maximum granted rate across
    // its live subscriptions, so a slow subscriber never throttles a fast
    // one and no subscriber ever receives less than it asked for.
    std::uint32_t topic_effective_rate_millihz(std::uint16_t topic_id) const noexcept;

    std::size_t active_subscription_count() const noexcept;

    // Publish period derived from the topic's aggregate effective rate, in
    // microseconds. Returns 0 when the topic has no live subscription
    // (caller must not publish), or when the topic is unknown.
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
public:
    // Bounded subscription table: kMaxTopics topics times a handful of
    // concurrent desktop sessions behind the single ESP-NOW peer. A request
    // that would exceed it is answered CAPACITY_EXHAUSTED instead of
    // evicting somebody else's subscription.
    static constexpr std::size_t kMaxSubscriptions = 8U;

private:
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

    // Per-topic byte/drop counters for STATUS section 5.1. Index-aligned with
    // schemas(), not a separate map. Subscriptions themselves live in
    // subscriptions_ below, because a topic can now have several.
    struct TopicRuntime {
        std::uint16_t topic_id = 0U;
        std::uint64_t bytes_sent_total = 0U;
        std::uint64_t samples_dropped_total = 0U;
    };

    // One row per (subscriber session, topic_id). Statically sized: no
    // allocation anywhere on the control or radio path.
    struct Subscription {
        bool active = false;
        std::uint16_t topic_id = 0U;
        std::uint32_t subscription_id = 0U;
        std::uint32_t subscriber_source_id = 0U;
        std::uint32_t subscriber_boot_id = 0U;
        std::uint32_t requested_rate_millihz = 0U;  // raw request bytes, kept
        std::uint32_t requested_lease_ms = 0U;      // to detect an exact retry
        std::uint32_t effective_rate_millihz = 0U;
        std::uint64_t lease_deadline_us = 0U;
    };

    PublishResult enqueue(std::uint16_t topic_id,
                          const std::uint8_t* payload,
                          std::size_t payload_size,
                          std::uint64_t timestamp_us) noexcept;
    void count_invalid() noexcept;
    int find_topic_index(std::uint16_t topic_id) const noexcept;
    void init_runtime_if_needed() noexcept;
    // Callers must already hold runtime_lock_.
    std::uint16_t locked_subscriber_count(std::uint16_t topic_id) const noexcept;
    std::uint32_t locked_topic_rate(std::uint16_t topic_id) const noexcept;

    BtpEndpoint* endpoint_ = nullptr;
    Sample queue_[kQueueCapacity]{};
    TopicRuntime runtime_[kMaxTopics]{};
    Subscription subscriptions_[kMaxSubscriptions]{};
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
