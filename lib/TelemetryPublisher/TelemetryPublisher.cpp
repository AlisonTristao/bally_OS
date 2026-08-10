#include <TelemetryPublisher.h>

#include <BtpTransport.h>

#include <cmath>
#include <cstring>
#include <limits>

namespace {

constexpr TelemetryPublisher::FieldSchema kProtocolTestFields[] = {
    {1U, 0U, "counter", TelemetryPublisher::WireType::Uint32, "1", 1.0F,
     0.0F, 1U, false},
    {2U, 1U, "value", TelemetryPublisher::WireType::Float32, "1", 1.0F,
     0.0F, 1U, false},
};

constexpr TelemetryPublisher::FieldSchema kRobotStateFields[] = {
    {1U, 0U, "state", TelemetryPublisher::WireType::Uint8, "1", 1.0F,
     0.0F, 1U, false},
};

constexpr TelemetryPublisher::TopicSchema kSchemas[] = {
    {TelemetryPublisher::kProtocolTestTopicId,
     TelemetryPublisher::kSchemaVersion,
     "protocol.test",
     TelemetryPublisher::Encoding::PackedLe,
     kProtocolTestFields,
     sizeof(kProtocolTestFields) / sizeof(kProtocolTestFields[0]),
     TelemetryPublisher::kProtocolTestPayloadSize,
     50000U},  // 50 Hz (topico 10/15)
    {TelemetryPublisher::kRobotStateTopicId,
     TelemetryPublisher::kSchemaVersion,
     "robot.state",
     TelemetryPublisher::Encoding::PackedLe,
     kRobotStateFields,
     sizeof(kRobotStateFields) / sizeof(kRobotStateFields[0]),
     TelemetryPublisher::kRobotStatePayloadSize,
     0U},  // published on state transitions, not periodic
};

void write_u16_le(std::uint8_t* output, std::uint16_t value) noexcept {
    output[0] = static_cast<std::uint8_t>(value);
    output[1] = static_cast<std::uint8_t>(value >> 8U);
}

void write_u32_le(std::uint8_t* output, std::uint32_t value) noexcept {
    output[0] = static_cast<std::uint8_t>(value);
    output[1] = static_cast<std::uint8_t>(value >> 8U);
    output[2] = static_cast<std::uint8_t>(value >> 16U);
    output[3] = static_cast<std::uint8_t>(value >> 24U);
}

// Bounded spin: this lock is only ever held for a handful of field
// reads/writes (no allocation, no radio call), so a few hundred relaxed
// attempts is generous headroom while still guaranteeing a hot-path caller
// (topic_active/topic_period_us, called once per sampleTelemetry() tick)
// never blocks indefinitely.
constexpr int kLockSpinAttempts = 256;

class RuntimeLockGuard {
public:
    explicit RuntimeLockGuard(std::atomic_flag& flag) noexcept : flag_(flag) {
        for (int i = 0; i < kLockSpinAttempts; ++i) {
            if (!flag_.test_and_set(std::memory_order_acquire)) {
                acquired_ = true;
                return;
            }
        }
    }
    ~RuntimeLockGuard() noexcept {
        if (acquired_) flag_.clear(std::memory_order_release);
    }
    bool acquired() const noexcept { return acquired_; }

private:
    std::atomic_flag& flag_;
    bool acquired_ = false;
};

}  // namespace

void TelemetryPublisher::configure(BtpEndpoint& endpoint) noexcept {
    endpoint_ = &endpoint;
}

TelemetryPublisher::PublishResult TelemetryPublisher::publish_protocol_test(
    std::uint32_t counter,
    float value,
    std::uint64_t timestamp_us) noexcept {
    std::uint8_t payload[kProtocolTestPayloadSize];
    if (!pack_protocol_test(counter, value, payload)) {
        count_invalid();
        return PublishResult::InvalidValue;
    }
    return enqueue(kProtocolTestTopicId, payload, sizeof(payload), timestamp_us);
}

TelemetryPublisher::PublishResult TelemetryPublisher::publish_robot_state(
    std::uint8_t state,
    std::uint64_t timestamp_us) noexcept {
    std::uint8_t payload[kRobotStatePayloadSize];
    pack_robot_state(state, payload);
    return enqueue(kRobotStateTopicId, payload, sizeof(payload), timestamp_us);
}

std::size_t TelemetryPublisher::flush(std::size_t max_samples) noexcept {
    if (endpoint_ == nullptr) return 0U;

    std::size_t processed = 0U;
    while (processed < max_samples) {
        const std::uint32_t read = read_index_.load(std::memory_order_relaxed);
        const std::uint32_t write = write_index_.load(std::memory_order_acquire);
        if (read == write) break;

        const Sample& sample = queue_[read % kQueueCapacity];
        const bool sent = endpoint_->send_fragment(
            btp::MessageType::Telemetry, sample.topic_id, sample.sequence,
            sample.timestamp_us, sample.payload, sample.payload_size, 0U, 1U);
        if (sent) {
            sent_total_.fetch_add(1U, std::memory_order_relaxed);
            RuntimeLockGuard guard(runtime_lock_);
            if (guard.acquired()) {
                init_runtime_if_needed();
                const int idx = find_topic_index(sample.topic_id);
                if (idx >= 0) {
                    runtime_[static_cast<std::size_t>(idx)].bytes_sent_total +=
                        sample.payload_size;
                }
            }
        } else {
            send_failed_.fetch_add(1U, std::memory_order_relaxed);
        }

        read_index_.store(read + 1U, std::memory_order_release);
        ++processed;
    }
    return processed;
}

std::size_t TelemetryPublisher::queued_count() const noexcept {
    const std::uint32_t write = write_index_.load(std::memory_order_acquire);
    const std::uint32_t read = read_index_.load(std::memory_order_acquire);
    return static_cast<std::size_t>(write - read);
}

TelemetryPublisher::Stats TelemetryPublisher::stats() const noexcept {
    return {
        queued_total_.load(std::memory_order_relaxed),
        sent_total_.load(std::memory_order_relaxed),
        dropped_full_.load(std::memory_order_relaxed),
        dropped_invalid_.load(std::memory_order_relaxed),
        send_failed_.load(std::memory_order_relaxed),
    };
}

const TelemetryPublisher::TopicSchema* TelemetryPublisher::schemas(
    std::size_t* count) noexcept {
    if (count != nullptr) *count = sizeof(kSchemas) / sizeof(kSchemas[0]);
    return kSchemas;
}

bool TelemetryPublisher::pack_protocol_test(
    std::uint32_t counter,
    float value,
    std::uint8_t output[kProtocolTestPayloadSize]) noexcept {
    static_assert(sizeof(float) == sizeof(std::uint32_t),
                  "BTP float32 requires a 32-bit float");
    static_assert(std::numeric_limits<float>::is_iec559,
                  "BTP float32 requires IEEE-754");
    if (output == nullptr || !std::isfinite(value)) return false;

    std::uint32_t value_bits = 0U;
    std::memcpy(&value_bits, &value, sizeof(value_bits));
    write_u16_le(output, kSchemaVersion);
    write_u32_le(output + 2U, counter);
    write_u32_le(output + 6U, value_bits);
    return true;
}

void TelemetryPublisher::pack_robot_state(
    std::uint8_t state,
    std::uint8_t output[kRobotStatePayloadSize]) noexcept {
    if (output == nullptr) return;
    write_u16_le(output, kSchemaVersion);
    output[2] = state;
}

TelemetryPublisher::PublishResult TelemetryPublisher::enqueue(
    std::uint16_t topic_id,
    const std::uint8_t* payload,
    std::size_t payload_size,
    std::uint64_t timestamp_us) noexcept {
    if (endpoint_ == nullptr) {
        count_invalid();
        return PublishResult::NotConfigured;
    }
    if (payload == nullptr || payload_size == 0U ||
        payload_size > kMaxPayloadSize || topic_id == 0U) {
        count_invalid();
        return PublishResult::InvalidValue;
    }

    const std::uint32_t write = write_index_.load(std::memory_order_relaxed);
    const std::uint32_t read = read_index_.load(std::memory_order_acquire);
    if (write - read >= kQueueCapacity) {
        dropped_full_.fetch_add(1U, std::memory_order_relaxed);
        RuntimeLockGuard guard(runtime_lock_);
        if (guard.acquired()) {
            init_runtime_if_needed();
            const int idx = find_topic_index(topic_id);
            if (idx >= 0) {
                runtime_[static_cast<std::size_t>(idx)].samples_dropped_total += 1U;
            }
        }
        return PublishResult::QueueFull;
    }

    std::uint32_t sequence = 0U;
    if (!endpoint_->try_reserve_sequence(&sequence)) {
        count_invalid();
        return PublishResult::SequenceUnavailable;
    }

    Sample& sample = queue_[write % kQueueCapacity];
    sample.timestamp_us = timestamp_us;
    sample.sequence = sequence;
    sample.topic_id = topic_id;
    sample.payload_size = static_cast<std::uint16_t>(payload_size);
    std::memcpy(sample.payload, payload, payload_size);

    write_index_.store(write + 1U, std::memory_order_release);
    queued_total_.fetch_add(1U, std::memory_order_relaxed);
    return PublishResult::Queued;
}

void TelemetryPublisher::count_invalid() noexcept {
    dropped_invalid_.fetch_add(1U, std::memory_order_relaxed);
}

int TelemetryPublisher::find_topic_index(std::uint16_t topic_id) const noexcept {
    for (std::size_t i = 0U; i < kMaxTopics; ++i) {
        if (runtime_[i].topic_id == topic_id) return static_cast<int>(i);
    }
    return -1;
}

void TelemetryPublisher::init_runtime_if_needed() noexcept {
    if (runtime_initialized_) return;
    std::size_t count = 0U;
    const TopicSchema* schema_list = schemas(&count);
    for (std::size_t i = 0U; i < count && i < kMaxTopics; ++i) {
        runtime_[i].topic_id = schema_list[i].topic_id;
    }
    runtime_initialized_ = true;
}

TelemetryPublisher::SubscribeOutcome TelemetryPublisher::subscribe(
    std::uint16_t topic_id,
    std::uint32_t request_source_id,
    std::uint32_t request_boot_id,
    std::uint32_t requested_rate_millihz,
    std::uint32_t requested_lease_ms,
    std::uint64_t now_us) noexcept {
    SubscribeOutcome outcome{};

    std::size_t schema_count = 0U;
    const TopicSchema* schema_list = schemas(&schema_count);
    const TopicSchema* schema = nullptr;
    for (std::size_t i = 0U; i < schema_count; ++i) {
        if (schema_list[i].topic_id == topic_id) {
            schema = &schema_list[i];
            break;
        }
    }
    if (schema == nullptr) return outcome;  // topic_known stays false

    RuntimeLockGuard guard(runtime_lock_);
    if (!guard.acquired()) return outcome;
    init_runtime_if_needed();

    const int idx = find_topic_index(topic_id);
    if (idx < 0) return outcome;
    TopicRuntime& runtime = runtime_[static_cast<std::size_t>(idx)];

    const std::uint32_t clamped_lease_ms =
        (requested_lease_ms < kMinLeaseMs) ? kMinLeaseMs
        : (requested_lease_ms > kMaxLeaseMs) ? kMaxLeaseMs
                                              : requested_lease_ms;
    // Effective rate never exceeds what was asked nor the schema's max
    // (COMMANDS_AND_ACTIONS.md section 7). A schema max of zero means "not
    // periodic" (robot.state): the request is accepted and the rate is
    // echoed back informationally, but nothing in sampleTelemetry() uses it
    // to gate an event-driven publish.
    std::uint32_t effective_rate = requested_rate_millihz;
    if (schema->max_rate_millihz != 0U && effective_rate > schema->max_rate_millihz) {
        effective_rate = schema->max_rate_millihz;
    }
    if (effective_rate == 0U) effective_rate = 1U;  // never grant a zero rate

    const bool sameRequester = runtime.subscribed &&
        runtime.request_source_id == request_source_id &&
        runtime.request_boot_id == request_boot_id &&
        runtime.requested_rate_millihz == requested_rate_millihz &&
        runtime.requested_lease_ms == requested_lease_ms;

    if (!sameRequester) {
        runtime.subscription_id = next_subscription_id_++;
        if (next_subscription_id_ == 0U) next_subscription_id_ = 1U;  // never 0
    }

    runtime.subscribed = true;
    runtime.effective_rate_millihz = effective_rate;
    runtime.lease_deadline_us = now_us + (static_cast<std::uint64_t>(clamped_lease_ms) * 1000ULL);
    runtime.request_source_id = request_source_id;
    runtime.request_boot_id = request_boot_id;
    runtime.requested_rate_millihz = requested_rate_millihz;
    runtime.requested_lease_ms = clamped_lease_ms;

    outcome.topic_known = true;
    outcome.subscription_id = runtime.subscription_id;
    outcome.effective_rate_millihz = effective_rate;
    outcome.granted_lease_ms = clamped_lease_ms;
    return outcome;
}

TelemetryPublisher::UnsubscribeOutcome TelemetryPublisher::unsubscribe(
    std::uint32_t subscription_id) noexcept {
    RuntimeLockGuard guard(runtime_lock_);
    if (!guard.acquired()) return UnsubscribeOutcome::NotFound;
    init_runtime_if_needed();

    for (std::size_t i = 0U; i < kMaxTopics; ++i) {
        TopicRuntime& runtime = runtime_[i];
        if (runtime.subscribed && runtime.subscription_id == subscription_id) {
            runtime.subscribed = false;
            runtime.effective_rate_millihz = 0U;
            runtime.lease_deadline_us = 0U;
            return UnsubscribeOutcome::Removed;
        }
    }
    // Removing an already-absent subscription is a success per
    // COMMANDS_AND_ACTIONS.md section 7 ("torna retries idempotentes"), not
    // an UnknownTopic error -- the caller cannot tell "already gone" apart
    // from "never existed" from subscription_id alone, and both map to the
    // same SUCCESS/NONE result on the wire.
    return UnsubscribeOutcome::NotFound;
}

void TelemetryPublisher::expire_subscriptions(std::uint64_t now_us) noexcept {
    RuntimeLockGuard guard(runtime_lock_);
    if (!guard.acquired()) return;
    init_runtime_if_needed();

    for (std::size_t i = 0U; i < kMaxTopics; ++i) {
        TopicRuntime& runtime = runtime_[i];
        if (runtime.subscribed && now_us >= runtime.lease_deadline_us) {
            runtime.subscribed = false;
            runtime.effective_rate_millihz = 0U;
        }
    }
}

bool TelemetryPublisher::topic_active(std::uint16_t topic_id) const noexcept {
    RuntimeLockGuard guard(runtime_lock_);
    if (!guard.acquired()) return false;
    const_cast<TelemetryPublisher*>(this)->init_runtime_if_needed();

    const int idx = find_topic_index(topic_id);
    if (idx < 0) return false;
    return runtime_[static_cast<std::size_t>(idx)].subscribed;
}

std::uint64_t TelemetryPublisher::topic_period_us(std::uint16_t topic_id) const noexcept {
    RuntimeLockGuard guard(runtime_lock_);
    if (!guard.acquired()) return 0U;
    const_cast<TelemetryPublisher*>(this)->init_runtime_if_needed();

    const int idx = find_topic_index(topic_id);
    if (idx < 0) return 0U;
    const TopicRuntime& runtime = runtime_[static_cast<std::size_t>(idx)];
    if (!runtime.subscribed || runtime.effective_rate_millihz == 0U) return 0U;
    // period_us = 1e9 / rate_millihz (rate_hz = rate_millihz / 1000).
    return 1000000000ULL / static_cast<std::uint64_t>(runtime.effective_rate_millihz);
}

std::size_t TelemetryPublisher::topic_stats(TopicStats* out, std::size_t max_count) const noexcept {
    if (out == nullptr) return 0U;
    RuntimeLockGuard guard(runtime_lock_);
    if (!guard.acquired()) return 0U;
    const_cast<TelemetryPublisher*>(this)->init_runtime_if_needed();

    std::size_t written = 0U;
    for (std::size_t i = 0U; i < kMaxTopics && written < max_count; ++i) {
        const TopicRuntime& runtime = runtime_[i];
        if (runtime.topic_id == 0U) continue;
        out[written].topic_id = runtime.topic_id;
        out[written].subscriber_count = runtime.subscribed ? 1U : 0U;
        out[written].effective_rate_millihz = runtime.effective_rate_millihz;
        out[written].bytes_sent_total = runtime.bytes_sent_total;
        out[written].samples_dropped_total = runtime.samples_dropped_total;
        ++written;
    }
    return written;
}
