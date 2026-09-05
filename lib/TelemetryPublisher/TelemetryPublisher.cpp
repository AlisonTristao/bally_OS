#include <TelemetryPublisher.h>

#include <BtpTransport.h>

#include <cstring>

namespace {

constexpr TelemetryPublisher::FieldSchema kProtocolTestFields[] = {
    {1U, 0U, "counter", btp::WireType::Uint32, "1", 1.0F, 0.0F, 1U, false},
    {2U, 1U, "value", btp::WireType::Float32, "1", 1.0F, 0.0F, 1U, false},
};

constexpr TelemetryPublisher::FieldSchema kRobotStateFields[] = {
    {1U, 0U, "state", btp::WireType::Uint8, "1", 1.0F, 0.0F, 1U, false},
};

constexpr TelemetryPublisher::TopicSchema kSchemas[] = {
    {TelemetryPublisher::kProtocolTestTopicId,
     TelemetryPublisher::kSchemaVersion,
     "protocol.test",
     TelemetryPublisher::Encoding::PackedLe,
     kProtocolTestFields,
     sizeof(kProtocolTestFields) / sizeof(kProtocolTestFields[0]),
     TelemetryPublisher::kProtocolTestPayloadSize,
     50000U,  // max: 50 Hz (topico 10/15)
     100U,    // min: 0.1 Hz -- one sample every 10 s is the slowest schedule
              // worth keeping a subscription alive for
     10000U},  // default/nominal: 10 Hz (unused while max is nonzero)
    {TelemetryPublisher::kRobotStateTopicId,
     TelemetryPublisher::kSchemaVersion,
     "robot.state",
     TelemetryPublisher::Encoding::PackedLe,
     kRobotStateFields,
     sizeof(kRobotStateFields) / sizeof(kRobotStateFields[0]),
     TelemetryPublisher::kRobotStatePayloadSize,
     0U,      // max: published on state transitions, not periodic
     0U,      // min: no floor -- an event-driven topic has no schedule
     10000U},  // default: caps the informational rate echoed back at the
               // state machine's own 10 Hz-ish transition ceiling
    {TelemetryPublisher::kSystemMonitorTopicId,
     TelemetryPublisher::kSchemaVersion,
     "system.monitor",
     TelemetryPublisher::Encoding::Utf8,
     nullptr,
     0U,  // UTF8 is the complete topic body; it has no structured fields
     TelemetryPublisher::kMaxSystemMonitorPayloadSize,
     1000U,  // max: 1 Hz (one report per second)
     1U,    // min: 0.001 Hz; slower dashboard periods remain valid
     333U},  // default: approximately 0.33 Hz (one report every 3 s)
};

// system.monitor (UTF8) is the one topic btp::SampleWriter does not build --
// its body is opaque text, not schema fields -- so its two-octet
// schema_version prefix is still written here.
void write_u16_le(std::uint8_t* output, std::uint16_t value) noexcept {
    output[0] = static_cast<std::uint8_t>(value);
    output[1] = static_cast<std::uint8_t>(value >> 8U);
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

void TelemetryPublisher::configure(BtpEndpoint& endpoint, BtpSealFn seal,
                                   void* seal_context) noexcept {
    endpoint_ = &endpoint;
    seal_ = seal;
    seal_context_ = seal_context;
}

void TelemetryPublisher::bind_subscriptions(
    const btp::SubscriptionTable& subscriptions) noexcept {
    subscriptions_ = &subscriptions;
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

TelemetryPublisher::PublishResult TelemetryPublisher::publish_system_monitor(
    const char* text,
    std::size_t length,
    std::uint64_t timestamp_us) noexcept {
    if (endpoint_ == nullptr) {
        count_invalid();
        return PublishResult::NotConfigured;
    }

    // Replace-in-place: if flush() has not taken the previous document yet,
    // drop this one (a fresh report follows within ~3 s) rather than overwrite
    // a buffer the consumer may be reading.
    if (monitor_stage_.pending.load(std::memory_order_acquire)) {
        dropped_full_.fetch_add(1U, std::memory_order_relaxed);
        RuntimeLockGuard guard(runtime_lock_);
        if (guard.acquired()) {
            init_runtime_if_needed();
            const int idx = find_topic_index(kSystemMonitorTopicId);
            if (idx >= 0) {
                runtime_[static_cast<std::size_t>(idx)].samples_dropped_total += 1U;
            }
        }
        return PublishResult::QueueFull;
    }

    // Not pending -> the producer owns monitor_stage_.payload. pack validates
    // fully before writing a byte, so a rejected document leaves it untouched.
    std::size_t payload_size = 0U;
    if (!pack_system_monitor(text, length, monitor_stage_.payload,
                             &payload_size)) {
        count_invalid();
        return PublishResult::InvalidValue;
    }

    std::uint32_t sequence = 0U;
    if (!endpoint_->try_reserve_sequence(&sequence)) {
        count_invalid();
        return PublishResult::SequenceUnavailable;
    }

    monitor_stage_.timestamp_us = timestamp_us;
    monitor_stage_.sequence = sequence;
    monitor_stage_.payload_size = static_cast<std::uint16_t>(payload_size);
    monitor_stage_.pending.store(true, std::memory_order_release);

    queued_total_.fetch_add(1U, std::memory_order_relaxed);
    return PublishResult::Queued;
}

std::size_t TelemetryPublisher::flush(std::size_t max_samples) noexcept {
    if (endpoint_ == nullptr) return 0U;

    std::size_t processed = 0U;

    // The out-of-band system.monitor document is one logical sample no matter
    // how many wire fragments it needs. Drain it before the numeric queue:
    // its sequence was reserved earlier (low-rate topic), so sending it first
    // keeps sequences ascending on the wire in the common case. send_logical_
    // reserved() seals the whole document once, then fragments it; a failed
    // seal stays fail-closed.
    if (processed < max_samples &&
        monitor_stage_.pending.load(std::memory_order_acquire)) {
        const bool sent = endpoint_->send_logical_reserved(
            btp::MessageType::Telemetry, kSystemMonitorTopicId,
            monitor_stage_.sequence, monitor_stage_.payload,
            monitor_stage_.payload_size, monitor_stage_.timestamp_us, seal_,
            seal_context_);
        if (sent) {
            sent_total_.fetch_add(1U, std::memory_order_relaxed);
            RuntimeLockGuard guard(runtime_lock_);
            if (guard.acquired()) {
                init_runtime_if_needed();
                const int idx = find_topic_index(kSystemMonitorTopicId);
                if (idx >= 0) {
                    runtime_[static_cast<std::size_t>(idx)].bytes_sent_total +=
                        monitor_stage_.payload_size;
                }
            }
        } else {
            send_failed_.fetch_add(1U, std::memory_order_relaxed);
        }
        monitor_stage_.pending.store(false, std::memory_order_release);
        ++processed;
    }

    while (processed < max_samples) {
        const std::uint32_t read = read_index_.load(std::memory_order_relaxed);
        const std::uint32_t write = write_index_.load(std::memory_order_acquire);
        if (read == write) break;

        const Sample& sample = queue_[read % kQueueCapacity];
        // queue_[] now only carries the few-octet numeric samples: one sealed
        // fragment each, sequence already reserved by the producer.
        const bool sent = endpoint_->send_fragment(
            btp::MessageType::Telemetry, sample.topic_id, sample.sequence,
            sample.timestamp_us, sample.payload, sample.payload_size, 0U, 1U,
            seal_, seal_context_);
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
    const std::size_t staged =
        monitor_stage_.pending.load(std::memory_order_acquire) ? 1U : 0U;
    return static_cast<std::size_t>(write - read) + staged;
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
    if (output == nullptr) return false;

    // btp::SampleWriter writes the schema_version prefix, the two fields in
    // `order` and (had there been one) the nullable bitmap; it also rejects a
    // non-finite `value` (telemetry.md section 14.1).
    const btp::FieldSpec specs[] = {kProtocolTestFields[0].spec(),
                                    kProtocolTestFields[1].spec()};
    btp::SampleWriter writer(output, kProtocolTestPayloadSize, specs, 2U);
    std::size_t written = 0U;
    return writer.begin(kSchemaVersion) == btp::MessageError::Ok &&
           writer.put_u64(counter) == btp::MessageError::Ok &&
           writer.put_f64(static_cast<double>(value)) == btp::MessageError::Ok &&
           writer.finish(&written) == btp::MessageError::Ok;
}

void TelemetryPublisher::pack_robot_state(
    std::uint8_t state,
    std::uint8_t output[kRobotStatePayloadSize]) noexcept {
    if (output == nullptr) return;
    const btp::FieldSpec specs[] = {kRobotStateFields[0].spec()};
    btp::SampleWriter writer(output, kRobotStatePayloadSize, specs, 1U);
    std::size_t written = 0U;
    if (writer.begin(kSchemaVersion) == btp::MessageError::Ok &&
        writer.put_u64(state) == btp::MessageError::Ok) {
        (void)writer.finish(&written);
    }
}

bool TelemetryPublisher::pack_system_monitor(
    const char* text,
    std::size_t length,
    std::uint8_t output[kMaxSystemMonitorPayloadSize],
    std::size_t* bytes_written) noexcept {
    if (output == nullptr || bytes_written == nullptr ||
        (text == nullptr && length != 0U) ||
        length > kMaxSystemMonitorTextSize) {
        return false;
    }
    write_u16_le(output, kSchemaVersion);
    if (length != 0U) {
        std::memcpy(output + 2U, text, length);
    }
    *bytes_written = length + 2U;
    return true;
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

const TelemetryPublisher::TopicSchema* TelemetryPublisher::find_schema(
    std::uint16_t topic_id) noexcept {
    std::size_t schema_count = 0U;
    const TopicSchema* schema_list = schemas(&schema_count);
    for (std::size_t i = 0U; i < schema_count; ++i) {
        if (schema_list[i].topic_id == topic_id) return &schema_list[i];
    }
    return nullptr;
}

bool TelemetryPublisher::topic_active(std::uint16_t topic_id) const noexcept {
    return topic_subscriber_count(topic_id) != 0U;
}

std::uint16_t TelemetryPublisher::topic_subscriber_count(
    std::uint16_t topic_id) const noexcept {
    if (topic_id == 0U || subscriptions_ == nullptr) return 0U;
    return static_cast<std::uint16_t>(subscriptions_->subscriber_count(topic_id));
}

std::uint32_t TelemetryPublisher::topic_effective_rate_millihz(
    std::uint16_t topic_id) const noexcept {
    if (topic_id == 0U || subscriptions_ == nullptr) return 0U;
    return subscriptions_->aggregate_rate_millihz(topic_id);
}

std::size_t TelemetryPublisher::active_subscription_count() const noexcept {
    if (subscriptions_ == nullptr) return 0U;
    std::size_t schema_count = 0U;
    const TopicSchema* schema_list = schemas(&schema_count);
    std::size_t count = 0U;
    for (std::size_t i = 0U; i < schema_count; ++i) {
        count += subscriptions_->subscriber_count(schema_list[i].topic_id);
    }
    return count;
}

std::uint64_t TelemetryPublisher::topic_period_us(std::uint16_t topic_id) const noexcept {
    const std::uint32_t rate = topic_effective_rate_millihz(topic_id);
    if (rate == 0U) return 0U;
    // period_us = 1e9 / rate_millihz (rate_hz = rate_millihz / 1000).
    return 1000000000ULL / static_cast<std::uint64_t>(rate);
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
        out[written].subscriber_count = topic_subscriber_count(runtime.topic_id);
        out[written].effective_rate_millihz = topic_effective_rate_millihz(runtime.topic_id);
        out[written].bytes_sent_total = runtime.bytes_sent_total;
        out[written].samples_dropped_total = runtime.samples_dropped_total;
        ++written;
    }
    return written;
}
