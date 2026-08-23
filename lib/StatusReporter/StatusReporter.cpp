#include <StatusReporter.h>

#include <BtpTransport.h>
#include <TelemetryPublisher.h>

namespace {

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

void write_u64_le(std::uint8_t* output, std::uint64_t value) noexcept {
    for (std::size_t i = 0U; i < 8U; ++i) {
        output[i] = static_cast<std::uint8_t>(value >> (8U * i));
    }
}

}  // namespace

std::size_t StatusReporter::serialize(std::uint16_t flags,
                                      std::uint64_t uptime_us,
                                      const Counters& counters,
                                      const TopicRecord* topics,
                                      std::size_t topic_count,
                                      std::uint8_t* output,
                                      std::size_t output_capacity,
                                      bool version1) noexcept {
    if (output == nullptr || output_capacity < kBaseSize) return 0U;
    if (version1) topic_count = 0U;
    if (topics == nullptr) topic_count = 0U;
    if (topic_count > kMaxTopicRecords) topic_count = kMaxTopicRecords;

    // Section 5, unchanged layout in version 2.
    write_u16_le(output + 0U, version1 ? kStatusVersion1 : kStatusVersion2);
    // "flags | bit 0 DEGRADED, demais zero": every reserved bit MUST be zero
    // on the wire (model.md section 7), so anything else the caller passes
    // is dropped
    // here rather than emitted and rejected by the peer.
    write_u16_le(output + 2U, static_cast<std::uint16_t>(flags & kFlagDegraded));
    write_u64_le(output + 4U, uptime_us);
    write_u64_le(output + 12U, counters.frames_rx);
    write_u64_le(output + 20U, counters.frames_tx);
    write_u64_le(output + 28U, counters.frames_dropped);
    write_u64_le(output + 36U, counters.crc_errors);
    write_u64_le(output + 44U, counters.decode_errors);
    write_u64_le(output + 52U, counters.reassembly_completed);
    write_u64_le(output + 60U, counters.reassembly_timeouts);
    write_u64_le(output + 68U, counters.reassembly_rejected);
    write_u64_le(output + 76U, counters.command_duplicates);
    write_u64_le(output + 84U, counters.telemetry_dropped);

    // "Uma mensagem com status_version=1 MUST NOT incluir esses campos."
    if (version1) return kBaseSize;

    if (output_capacity < kBaseSize + kTopicStatusCountSize) return 0U;

    std::size_t written = 0U;
    std::size_t offset = kBaseSize + kTopicStatusCountSize;
    for (std::size_t i = 0U; i < topic_count; ++i) {
        const TopicRecord& record = topics[i];
        if (record.source_id == 0U || record.topic_id == 0U) continue;

        // (source_id, topic_id) MUST be unique inside one STATUS message.
        bool duplicate = false;
        for (std::size_t j = 0U; j < i; ++j) {
            if (topics[j].source_id == record.source_id &&
                topics[j].topic_id == record.topic_id) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) continue;

        if (offset + kTopicStatusRecordSize > output_capacity) break;
        write_u32_le(output + offset + 0U, record.source_id);
        write_u16_le(output + offset + 4U, record.topic_id);
        write_u16_le(output + offset + 6U, record.subscriber_count);
        write_u32_le(output + offset + 8U, record.effective_rate_millihz);
        write_u64_le(output + offset + 12U, record.bytes_total);
        write_u64_le(output + offset + 20U, record.samples_dropped_total);
        offset += kTopicStatusRecordSize;
        ++written;
    }

    write_u16_le(output + kBaseSize, static_cast<std::uint16_t>(written));
    return offset;
}

void StatusReporter::configure(BtpEndpoint& endpoint,
                               TelemetryPublisher& publisher) noexcept {
    endpoint_ = &endpoint;
    publisher_ = &publisher;
}

bool StatusReporter::publish(std::uint16_t flags,
                             std::uint64_t uptime_us,
                             const Counters& counters,
                             std::uint64_t timestamp_us) noexcept {
    if (endpoint_ == nullptr || publisher_ == nullptr) return false;

    TelemetryPublisher::TopicStats stats[kMaxTopicRecords]{};
    const std::size_t stats_count = publisher_->topic_stats(stats, kMaxTopicRecords);

    TopicRecord records[kMaxTopicRecords]{};
    for (std::size_t i = 0U; i < stats_count; ++i) {
        records[i].source_id = endpoint_->source_id();
        records[i].topic_id = stats[i].topic_id;
        records[i].subscriber_count = stats[i].subscriber_count;
        records[i].effective_rate_millihz = stats[i].effective_rate_millihz;
        records[i].bytes_total = stats[i].bytes_sent_total;
        records[i].samples_dropped_total = stats[i].samples_dropped_total;
    }

    std::uint8_t payload[kMaxPayloadSize];
    const std::size_t size = serialize(flags, uptime_us, counters, records,
                                       stats_count, payload, sizeof(payload));
    if (size == 0U) return false;

    return endpoint_->send_logical(btp::MessageType::Control, kStatusObjectId,
                                   payload, size, timestamp_us);
}
