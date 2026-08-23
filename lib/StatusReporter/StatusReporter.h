#ifndef STATUS_REPORTER_H
#define STATUS_REPORTER_H

#include <cstddef>
#include <cstdint>

#include <btp/codec.hpp>

// Needed for BtpSealFn (a file-scope type alias, not a member of
// BtpEndpoint -- see the comment on it), which configure() below names.
#include <BtpTransport.h>

class TelemetryPublisher;

// Builds and publishes CONTROL/STATUS
// (BTP/docs/commands.md sections 5 and 5.1).
//
// Deliberately pure C++ (no Arduino/FreeRTOS/esp_timer), same shape as
// ManifestResponder and SubscriptionResponder, so the serializer links into
// env:native and is checked byte by byte by the unit tests. The caller owns
// the clock and the link counters; this class only lays them out on the wire
// and pulls the per-topic block straight out of TelemetryPublisher, which is
// the single source of truth for subscriptions, granted rates, bytes and
// drops (topico 17 PASSOS 8/9).
class StatusReporter {
public:
    static constexpr std::uint16_t kStatusObjectId = 0x0009U;
    static constexpr std::uint16_t kStatusVersion1 = 1U;
    static constexpr std::uint16_t kStatusVersion2 = 2U;
    static constexpr std::uint16_t kFlagDegraded = 0x0001U;

    // Section 5: the fixed part is exactly 92 octets and keeps the same
    // layout and offsets in version 2.
    static constexpr std::size_t kBaseSize = 92U;
    // Section 5.1: uint16_le count at offset 92, then fixed-size records with
    // no record_size of their own.
    static constexpr std::size_t kTopicStatusCountSize = 2U;
    // 28 octets: source_id@0(u32) topic_id@4(u16) subscriber_count@6(u16)
    // effective_rate_millihz@8(u32) bytes_total@12(u64)
    // samples_dropped_total@20(u64), exactly the normative offset table of
    // commands.md section 5.1. (An earlier revision said "24 octetos" by
    // an arithmetic slip; the spec has since been corrected to 28, which is
    // also what t_dongle_develop's SerialSession.h emits, so both emitters
    // agree on the wire.) Single place the stride is defined.
    static constexpr std::size_t kTopicStatusRecordSize = 28U;
    static constexpr std::size_t kMaxTopicRecords = 4U;
    static constexpr std::size_t kMaxPayloadSize =
        kBaseSize + kTopicStatusCountSize + (kTopicStatusRecordSize * kMaxTopicRecords);

    // Link/protocol counters of commands.md section 5. Monotonic since
    // boot; the caller saturates them, this class only serializes.
    struct Counters {
        std::uint64_t frames_rx = 0U;
        std::uint64_t frames_tx = 0U;
        std::uint64_t frames_dropped = 0U;
        std::uint64_t crc_errors = 0U;
        std::uint64_t decode_errors = 0U;
        std::uint64_t reassembly_completed = 0U;
        std::uint64_t reassembly_timeouts = 0U;
        std::uint64_t reassembly_rejected = 0U;
        std::uint64_t command_duplicates = 0U;
        std::uint64_t telemetry_dropped = 0U;
    };

    // One 28-octet topic_status record of commands.md section 5.1.
    struct TopicRecord {
        std::uint32_t source_id = 0U;
        std::uint16_t topic_id = 0U;
        std::uint16_t subscriber_count = 0U;
        std::uint32_t effective_rate_millihz = 0U;
        std::uint64_t bytes_total = 0U;
        std::uint64_t samples_dropped_total = 0U;
    };

    // Writes a status_version=2 payload into `output`. Returns the number of
    // octets written, or 0 when the buffer is too small or a record is
    // unusable. Records with a zero source_id or topic_id are skipped (both
    // are "non-zero" in commands.md section 5.1) and so is a duplicated
    // (source_id, topic_id) pair, which MUST be unique within one message.
    // A caller that wants the plain section 5 message passes topic_count = 0
    // together with version1 = true.
    static std::size_t serialize(std::uint16_t flags,
                                 std::uint64_t uptime_us,
                                 const Counters& counters,
                                 const TopicRecord* topics,
                                 std::size_t topic_count,
                                 std::uint8_t* output,
                                 std::size_t output_capacity,
                                 bool version1 = false) noexcept;

    // `seal`/`seal_context` (default nullptr) are forwarded verbatim to
    // BtpEndpoint::send_logical -- see BtpSealFn. STATUS is channel C
    // (dongle<->robot, heartbeat -- bally_channels.h), so its real caller
    // passes RadioSeal::seal; this class stays free of any AEAD dependency
    // itself, the same reason BtpEndpoint takes a function pointer instead
    // of calling btp::aead directly.
    void configure(BtpEndpoint& endpoint, TelemetryPublisher& publisher,
                   BtpSealFn seal = nullptr,
                   void* seal_context = nullptr) noexcept;

    // Snapshots TelemetryPublisher's per-topic block, stamps this robot's own
    // source_id on every record (commands.md section 5.1: "a source
    // describing only itself uses its own source_id in every record") and
    // sends one CONTROL/STATUS.
    // Never blocks: the frame goes through the normal BtpEndpoint ->
    // TxScheduler path, where STATUS already has its own priority queue below
    // COMMAND_RESULT and above telemetry.
    bool publish(std::uint16_t flags,
                 std::uint64_t uptime_us,
                 const Counters& counters,
                 std::uint64_t timestamp_us) noexcept;

private:
    BtpEndpoint* endpoint_ = nullptr;
    TelemetryPublisher* publisher_ = nullptr;
    BtpSealFn seal_ = nullptr;
    void* seal_context_ = nullptr;
};

#endif  // STATUS_REPORTER_H
