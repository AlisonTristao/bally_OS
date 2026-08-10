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

    void configure(BtpEndpoint& endpoint) noexcept;

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

    struct Sample {
        std::uint64_t timestamp_us;
        std::uint32_t sequence;
        std::uint16_t topic_id;
        std::uint16_t payload_size;
        std::uint8_t payload[kMaxPayloadSize];
    };

    PublishResult enqueue(std::uint16_t topic_id,
                          const std::uint8_t* payload,
                          std::size_t payload_size,
                          std::uint64_t timestamp_us) noexcept;
    void count_invalid() noexcept;

    BtpEndpoint* endpoint_ = nullptr;
    Sample queue_[kQueueCapacity]{};
    std::atomic<std::uint32_t> write_index_{0U};
    std::atomic<std::uint32_t> read_index_{0U};
    std::atomic<std::uint32_t> queued_total_{0U};
    std::atomic<std::uint32_t> sent_total_{0U};
    std::atomic<std::uint32_t> dropped_full_{0U};
    std::atomic<std::uint32_t> dropped_invalid_{0U};
    std::atomic<std::uint32_t> send_failed_{0U};
};

#endif  // TELEMETRY_PUBLISHER_H
