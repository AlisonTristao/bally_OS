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
     TelemetryPublisher::kProtocolTestPayloadSize},
    {TelemetryPublisher::kRobotStateTopicId,
     TelemetryPublisher::kSchemaVersion,
     "robot.state",
     TelemetryPublisher::Encoding::PackedLe,
     kRobotStateFields,
     sizeof(kRobotStateFields) / sizeof(kRobotStateFields[0]),
     TelemetryPublisher::kRobotStatePayloadSize},
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
