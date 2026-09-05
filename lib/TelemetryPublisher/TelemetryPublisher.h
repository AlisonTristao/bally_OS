#ifndef TELEMETRY_PUBLISHER_H
#define TELEMETRY_PUBLISHER_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <btp/codec.hpp>
#include <btp/subscription.hpp>  // btp::SubscriptionTable
#include <btp/telemetry.hpp>  // btp::WireType, btp::FieldSpec, btp::SampleWriter

// Needed for BtpSealFn (a file-scope type alias, not a member of BtpEndpoint),
// which configure() below names -- same reason StatusReporter.h already
// includes this.
#include <BtpTransport.h>

class BtpEndpoint;

class TelemetryPublisher {
public:
    static constexpr std::uint16_t kProtocolTestTopicId = 0x0001U;
    static constexpr std::uint16_t kRobotStateTopicId = 0x0002U;
    static constexpr std::uint16_t kSystemMonitorTopicId = 0x0003U;

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
    // The system.monitor document is the full `sys -health` report
    // (SystemMonitor::getFullReport): banner + CPU + memory + the complete
    // per-task table. TELEMETRY reserves the first two payload octets for
    // schema_version, leaving this much for the UTF-8 text itself. Sized for
    // ~22 tasks; a longer report is line-truncated by getTelemetryReport().
    // BtpEndpoint::kMaxLogicalPayloadSize must stay >= the payload size below.
    static constexpr std::size_t kMaxSystemMonitorTextSize = 1800U;
    static constexpr std::size_t kMaxSystemMonitorPayloadSize =
        kMaxSystemMonitorTextSize + 2U;

    enum class Encoding : std::uint8_t { Utf8 = 0x02U, PackedLe = 0x05U };

    // The field's wire type is btp::WireType now (telemetry.md section 13's
    // octet values). ManifestResponder serializes it straight into the field
    // record; TelemetryPublisher::pack_* feeds it to btp::SampleWriter.
    struct FieldSchema {
        std::uint16_t field_id;
        std::uint16_t order;
        const char* name;
        btp::WireType type;
        const char* unit;
        float scale;
        float offset;
        std::uint16_t element_count;
        bool nullable;

        // The subset btp::SampleWriter / btp::SampleReader consume.
        btp::FieldSpec spec() const noexcept {
            btp::FieldSpec s{};
            s.field_id = field_id;
            s.order = order;
            s.type = static_cast<std::uint8_t>(type);
            s.flags = nullable ? btp::kFieldNullable : std::uint8_t{0};
            s.element_count = element_count;
            s.max_element_count = 0U;
            s.scale = static_cast<double>(scale);
            s.offset = static_cast<double>(offset);
            return s;
        }
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
    // kSchemas, indexed the same way schemas() enumerates them. "Does anyone
    // currently want this topic and at what rate" now lives in node_'s own
    // btp::SubscriptionTable (bind_subscriptions()); this struct only adds
    // the two counters that table cannot know -- bytes actually sent and
    // samples dropped -- so publish behavior can never drift from what was
    // actually granted.
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

    // `seal`/`seal_context` (default nullptr) are forwarded verbatim to
    // BtpEndpoint::send_fragment in flush() -- see BtpSealFn. TELEMETRY is
    // channel B (TraceView<->robot, key E -- bally_channels.h): the dongle
    // relays it and never reads it, so its real caller passes
    // RadioSeal::seal_e. Both left nullptr (the default) means every sample
    // goes out in the clear, exactly as before this parameter existed -- the
    // mode the native unit tests exercise. Fail-closed once a seal is set: a
    // sample whose seal fails (no key E loaded) is dropped and counted as a
    // send failure, never sent unsealed.
    void configure(BtpEndpoint& endpoint, BtpSealFn seal = nullptr,
                   void* seal_context = nullptr) noexcept;

    // Binds the btp::SubscriptionTable that now grants/renews/expires every
    // subscription (btp::Node::enable_subscriptions(), wired automatically by
    // node_'s own StaticNode<> constructor -- see BallyRobot.h's node_
    // comment) -- called once, separately from configure(), because node_
    // does not exist yet at configure()'s own call site
    // (configureProtocolIdentity(), before bindProtocolTransport() emplaces
    // it). Every method below reads this table instead of keeping its own;
    // topic_active()/topic_subscriber_count()/topic_effective_rate_millihz()/
    // topic_period_us() all return their old "nothing bound yet" answers
    // (false / 0) until this runs.
    void bind_subscriptions(const btp::SubscriptionTable& subscriptions) noexcept;

    static const TopicSchema* find_schema(std::uint16_t topic_id) noexcept;

    // True when topic_id currently has at least one live, unexpired
    // subscription (btp::SubscriptionTable::subscriber_count() -- expire()
    // reflects whatever it was last called with, see bind_subscriptions()'s
    // comment and BallyRobot.cpp's sampleTelemetry(), which calls it every
    // pass before this). Non-periodic topics (robot.state) are published on
    // events regardless of this, so callers only gate periodic topics on it.
    bool topic_active(std::uint16_t topic_id) const noexcept;

    // Live subscription count for one topic (0 when unknown/unsubscribed).
    std::uint16_t topic_subscriber_count(std::uint16_t topic_id) const noexcept;

    // Aggregate publish rate for one topic: the maximum granted rate across
    // its live subscriptions, so a slow subscriber never throttles a fast
    // one and no subscriber ever receives less than it asked for.
    std::uint32_t topic_effective_rate_millihz(std::uint16_t topic_id) const noexcept;

    // Total live subscriptions across every known topic (schemas()'s own
    // table) -- a sum of topic_subscriber_count(), not a count this class
    // keeps itself any more.
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
    // Publishes one complete replace-in-place dashboard document (the full
    // `sys -health` report). `text` is UTF-8 without a terminator and must fit
    // kMaxSystemMonitorTextSize. Unlike the numeric topics this does NOT go
    // through queue_[]: the document is large and low-rate (up to 1 Hz), so it
    // gets its own single staging slot. A document produced while the previous
    // one is still unflushed is dropped (QueueFull) -- the next tick refreshes
    // it -- rather than racing flush().
    PublishResult publish_system_monitor(const char* text, std::size_t length,
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
    static bool pack_system_monitor(
        const char* text, std::size_t length,
        std::uint8_t output[kMaxSystemMonitorPayloadSize],
        std::size_t* bytes_written) noexcept;

private:
    // queue_[] only ever holds the small numeric samples now (system.monitor
    // has its own staging slot, see monitor_stage_), so it is sized for them
    // and not the ~1.8 KB UTF-8 document -- that is what keeps queue_[16] at a
    // few hundred bytes instead of ~29 KB of mostly-idle static RAM.
    static constexpr std::size_t kMaxPayloadSize = kProtocolTestPayloadSize;
    static constexpr std::size_t kMaxTopics = 3U;  // matches kSchemas today
public:
    // kMaxTopics topics times a handful of concurrent desktop sessions behind
    // the single ESP-NOW peer -- node_'s own btp::SubscriptionTable is what
    // actually enforces this now (BallyRobot.h's node_ MaxSubscriptions
    // template argument), sized from this same constant so the two can never
    // drift apart. A request past capacity is answered CAPACITY_EXHAUSTED
    // instead of evicting somebody else's subscription.
    static constexpr std::size_t kMaxSubscriptions = 8U;

private:
    struct Sample {
        std::uint64_t timestamp_us;
        std::uint32_t sequence;
        std::uint16_t topic_id;
        std::uint16_t payload_size;
        std::uint8_t payload[kMaxPayloadSize];
    };

    // Out-of-band staging for the one large, low-rate, replace-in-place
    // topic. Single-producer (publish_system_monitor, control task) /
    // single-consumer (flush, comms task) handoff over `pending`: the
    // producer fills the fields then release-stores true; the consumer
    // acquire-loads, sends, then release-stores false. Neither side touches
    // the payload while the other owns it.
    struct MonitorStage {
        std::atomic<bool> pending{false};
        std::uint64_t timestamp_us = 0U;
        std::uint32_t sequence = 0U;
        std::uint16_t payload_size = 0U;
        std::uint8_t payload[kMaxSystemMonitorPayloadSize]{};
    };

    // Per-topic byte/drop counters for STATUS section 5.1. Index-aligned with
    // schemas(), not a separate map. Subscriptions themselves live in
    // btp::SubscriptionTable now (subscriptions_ below), because a topic can
    // have several.
    struct TopicRuntime {
        std::uint16_t topic_id = 0U;
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
    BtpSealFn seal_ = nullptr;
    void* seal_context_ = nullptr;
    Sample queue_[kQueueCapacity]{};
    MonitorStage monitor_stage_{};
    TopicRuntime runtime_[kMaxTopics]{};
    // nullptr until bind_subscriptions() runs (see its own comment) --
    // topic_active()/topic_subscriber_count()/topic_effective_rate_millihz()/
    // topic_period_us() all answer their "nothing bound" default (false / 0)
    // until then, the same posture an empty subscriptions_[] used to have.
    // Read from the state-machine task (sampleTelemetry()) and, through
    // node_->receive()/expire(), written from the Wi-Fi RX task -- the same
    // tolerated cross-task race node_ itself already has (see BallyRobot.h's
    // node_ comment): a torn read sees a grant/expiry as of a moment earlier,
    // never a corrupt one (SubscriptionRecord is plain fixed-size fields, no
    // pointers), and self-heals on the next tick. No lock, on purpose --
    // deliberately not reintroducing runtime_lock_'s old role here, since
    // that would mean stalling the Wi-Fi RX task's SUBSCRIBE handling behind
    // whatever the state-machine task is doing at the same moment.
    const btp::SubscriptionTable* subscriptions_ = nullptr;
    bool runtime_initialized_ = false;
    // Guards runtime_[] only. Independent of, and never held across, anything
    // touching BtpEndpoint/TxScheduler -- so it cannot introduce a wait on
    // the radio/priority path (PASSO 7). A short bounded spin rather than
    // blocking forever; a reader that loses the race treats a topic's stats
    // as unavailable for that one tick rather than stalling.
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
