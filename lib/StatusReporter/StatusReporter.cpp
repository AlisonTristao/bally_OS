#include <StatusReporter.h>

#include <BtpTransport.h>
#include <TelemetryPublisher.h>
#include <btp/messages.hpp>

// The module's wire constants must equal the library's -- callers size buffers
// off the StatusReporter names, btp::messages does the serialization.
static_assert(StatusReporter::kBaseSize == btp::kStatusV1Size,
              "kBaseSize must match btp::kStatusV1Size");
static_assert(StatusReporter::kTopicStatusRecordSize == btp::kTopicStatusRecordSize,
              "kTopicStatusRecordSize must match btp::kTopicStatusRecordSize");

std::size_t StatusReporter::serialize(std::uint16_t flags,
                                      std::uint64_t uptime_us,
                                      const Counters& counters,
                                      const TopicRecord* topics,
                                      std::size_t topic_count,
                                      std::uint8_t* output,
                                      std::size_t output_capacity,
                                      bool version1) noexcept {
    if (output == nullptr) return 0U;

    btp::StatusV1 base{};
    // model.md section 7: every reserved flags bit MUST be zero on the wire, so
    // anything but DEGRADED the caller passes is dropped here.
    base.flags = static_cast<std::uint16_t>(flags & kFlagDegraded);
    base.uptime_us = uptime_us;
    base.frames_rx = counters.frames_rx;
    base.frames_tx = counters.frames_tx;
    base.frames_dropped = counters.frames_dropped;
    base.crc_errors = counters.crc_errors;
    base.decode_errors = counters.decode_errors;
    base.reassembly_completed = counters.reassembly_completed;
    base.reassembly_timeouts = counters.reassembly_timeouts;
    base.reassembly_rejected = counters.reassembly_rejected;
    base.command_duplicates = counters.command_duplicates;
    base.telemetry_dropped = counters.telemetry_dropped;

    std::size_t written = 0U;
    if (version1) {
        return (btp::encode_status_v1(base, output, output_capacity, &written) == btp::MessageError::Ok)
                   ? written
                   : 0U;
    }

    // The section 5.1 rules btp::messages leaves to the caller: skip a record
    // with a zero source_id / topic_id, and drop a repeated (source_id,
    // topic_id) pair -- it must be unique within one message.
    btp::TopicStatusRecord kept_records[kMaxTopicRecords];
    std::size_t kept = 0U;
    if (topics != nullptr) {
        const std::size_t limit = (topic_count < kMaxTopicRecords) ? topic_count : kMaxTopicRecords;
        for (std::size_t i = 0U; i < limit; ++i) {
            const TopicRecord& record = topics[i];
            if (record.source_id == 0U || record.topic_id == 0U) continue;
            bool duplicate = false;
            for (std::size_t j = 0U; j < kept; ++j) {
                if (kept_records[j].source_id == record.source_id &&
                    kept_records[j].topic_id == record.topic_id) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;
            kept_records[kept].source_id = record.source_id;
            kept_records[kept].topic_id = record.topic_id;
            kept_records[kept].subscriber_count = record.subscriber_count;
            kept_records[kept].effective_rate_millihz = record.effective_rate_millihz;
            kept_records[kept].bytes_total = record.bytes_total;
            kept_records[kept].samples_dropped_total = record.samples_dropped_total;
            ++kept;
        }
    }

    return (btp::encode_status_v2(base, kept_records, kept, output, output_capacity, &written) ==
            btp::MessageError::Ok)
               ? written
               : 0U;
}

void StatusReporter::configure(BtpEndpoint& endpoint,
                               TelemetryPublisher& publisher,
                               BtpSealFn seal, void* seal_context) noexcept {
    endpoint_ = &endpoint;
    publisher_ = &publisher;
    seal_ = seal;
    seal_context_ = seal_context;
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
                                   payload, size, timestamp_us, seal_,
                                   seal_context_);
}
