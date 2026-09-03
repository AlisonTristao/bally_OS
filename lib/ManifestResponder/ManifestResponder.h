#ifndef MANIFEST_RESPONDER_H
#define MANIFEST_RESPONDER_H

#include <cstddef>
#include <cstdint>

#include <btp/catalog.hpp>
#include <btp/codec.hpp>

// Needed for BtpSealFn (a file-scope type alias, not a member of
// BtpEndpoint), which configure() below names -- same reason
// StatusReporter.h and SubscriptionResponder.h already include this.
#include <BtpTransport.h>

class BtpEndpoint;

// Answers CONTROL/MANIFEST_REQUEST (BTP/docs/commands.md
// section 3) by describing this robot's static schemas, taken
// verbatim from TelemetryPublisher::schemas() -- the same table the robot
// already uses to pack samples, so the manifest can never drift from what is
// actually published on the wire.
//
// Deliberately pure C++ (no Arduino/FreeRTOS), same shape as BtpTransport, so
// it links into env:native if a future topico wants to test it there. This
// robot has exactly one static, compile-time catalog (no runtime schema
// changes), so config_revision is a compile-time constant rather than a
// persisted counter. It is bumped whenever a firmware build changes the
// catalog and remains fixed for that build.
//
// Every response is single-shot: MANIFEST_REQUEST addressed to this boot (or
// to the wildcard target 0, which for a leaf node with no gateway role below
// it just means "describe yourself") always describes exactly one source
// (this robot), so catalog_index=0/catalog_count=1/CATALOG_COMPLETE is set on
// every response, per commands.md section 3.2's "for a targeted request".
// One source_info entry (BTP/docs/commands.md section 3.12): three textual
// fields. `key` is a stable machine identifier ("fw_version"), `label` a
// human name for display ("Firmware", may be empty), `value` the datum as
// text ("2", "16777216", "ESP32-S3"). All three are copied onto the wire when
// the response is built, so a caller may point them at any storage that stays
// valid for the duration of handle_request() -- esp_app_get_description()
// fields, RobotSettings buffers, static scratch.
struct SourceInfoEntry {
    const char* key;
    const char* label;
    const char* value;
};

class ManifestResponder {
public:
    static constexpr std::uint16_t kManifestRequestObjectId = 0x0003U;
    static constexpr std::uint16_t kManifestDataObjectId = 0x0004U;
    // Format 2 adds the source_info block after source_name (commands.md
    // section 3.12). A consumer that only implements format 1 rejects this
    // response, so the fleet moves together -- there is no format negotiation
    // in MANIFEST_REQUEST.
    static constexpr std::uint16_t kManifestFormatVersion = 2U;
    static constexpr std::uint32_t kConfigRevision = 2U;
    static constexpr std::uint8_t kSourceRoleRobot = 0x01U;
    // Upper bound on source_info entries this responder will emit; commands.md
    // section 6 caps the wire at 32. A caller passing more is truncated here
    // rather than overrunning the payload buffer.
    static constexpr std::size_t kMaxSourceInfoEntries = 16U;
    // Octets build_manifest_data keeps in reserve for the topic records after
    // the source_info block. The two current schemas total ~200; the slack
    // covers a third small topic. A device with more info than fits in
    // (payload budget - this) has its trailing info entries dropped, never
    // the schema. Raise both this and kMaxManifestPayloadSize together if a
    // future schema genuinely needs more.
    static constexpr std::size_t kRecordsReserveBytes = 256U;
    // The compact schema catalog (including fieldless system.monitor) plus
    // prefix/source information fits this bound. Sized to
    // BtpEndpoint::send_logical's own
    // kMaxLogicalPayloadSize ceiling (600) -- a manifest larger than that is
    // refused there anyway, so build_manifest_data failing at this bound is
    // the earlier, cleaner failure. send_logical still fragments whatever
    // fits across ESP-NOW frames (kEspNowMaxPayloadSize=210) automatically.
    static constexpr std::size_t kMaxManifestPayloadSize = 600U;

    // uuid is copied (16 bytes, opaque, stable identity for this boot -- see
    // BallyRobot.cpp's configureProtocolIdentity(), derived from the MAC).
    //
    // `seal`/`seal_context` (default nullptr) are forwarded verbatim to
    // BtpEndpoint::send_logical -- see BtpSealFn. MANIFEST_REQUEST/DATA is
    // channel C always (bally_channels.h: only the dongle's own aggregation
    // cache legitimately asks a robot for its manifest), so its real caller
    // passes RadioSeal::seal, the same single key StatusReporter's STATUS
    // already seals with -- no per-channel choice to make, unlike
    // SubscriptionResponder's SUBSCRIBE_RESULT/UNSUBSCRIBE_RESULT.
    // `source_info`/`source_info_count` (default none) describe this robot to
    // a human operator -- name, firmware version, chip, running partition.
    // The array is borrowed, not copied: it and every string it points at
    // must outlive this responder (the composition root keeps it static). Its
    // string *contents* are re-read on every response, so a value backed by a
    // RobotSettings buffer reflects a live "settings -set" without a
    // config_revision bump (commands.md section 3.12: source_info is not
    // gated by config_revision).
    void configure(BtpEndpoint& endpoint, const std::uint8_t uuid[16],
                   BtpSealFn seal = nullptr, void* seal_context = nullptr,
                   const SourceInfoEntry* source_info = nullptr,
                   std::size_t source_info_count = 0U) noexcept;

    // True once configure() has loaded every one of TelemetryPublisher::
    // schemas()'s field-bearing topics into catalog() without a
    // btp::MessageError -- a duplicate topic_id, a field whose `order` skips
    // a position, or more topics/fields than kMaxCatalogTopics/
    // kMaxCatalogFields hold. A compile-time schema can only fail this from a
    // programming error in TelemetryPublisher::kSchemas, so the caller's only
    // job is to log it (see BallyRobot.cpp's configureProtocolIdentity()) --
    // MANIFEST_DATA itself does not depend on this catalog (see catalog()
    // below) and keeps answering correctly either way.
    bool catalog_ok() const noexcept { return catalog_valid_; }

    // The schema, also as a validated btp::Catalog -- built once by
    // configure() from the exact same TelemetryPublisher::schemas() table
    // handle_request() already reads from directly. NOT what MANIFEST_DATA is
    // built from: btp::Catalog::add_topic() keeps a field's type/scale/
    // offset/name for btp::SampleReader/Writer, but not its unit or
    // description, both of which the wire manifest must still carry -- see
    // BTP/src/catalog.cpp. This exists so a topic-bearing table validated at
    // boot is ready for whatever next reaches for it (btp::SubscriptionTable
    // validates a SUBSCRIBE against a served catalog the same way).
    const btp::Catalog& catalog() const noexcept { return catalog_; }

    // Parses a CONTROL/MANIFEST_REQUEST payload (already validated as
    // unfragmented or reassembled by the caller) and sends the matching
    // MANIFEST_DATA response over `endpoint`. Returns false when the payload
    // was too short to parse at all (caller should count it as a drop, same
    // as any other malformed CONTROL frame); a well-formed request always
    // gets an answer (possibly NOT_FOUND/STALE_TARGET_BOOT), which still
    // returns true.
    bool handle_request(const btp::Header& request_header,
                        btp::ByteView payload,
                        std::uint64_t timestamp_us) noexcept;

private:
    // Sized for this robot's own two field-bearing topics (protocol.test: 2
    // fields, robot.state: 1 -- kSystemMonitorTopicId has none and is never
    // added, see buildCatalog()), with headroom for a third small topic
    // before either bound needs raising. btp::StaticCatalog bundles the
    // topic/field/name pools this needs.
    static constexpr std::size_t kMaxCatalogTopics = 4U;
    static constexpr std::size_t kMaxCatalogFields = 8U;
    static constexpr std::size_t kMaxCatalogStringBytes = 256U;

    // Converts TelemetryPublisher::schemas() into catalog_ via
    // btp::Catalog::add_topic(), one call per field-bearing topic. Called
    // once by configure(); sets catalog_valid_ false (and stops) at the
    // first btp::MessageError, so a boot log can say which topic broke
    // instead of only "something did".
    void buildCatalog() noexcept;

    BtpEndpoint* endpoint_ = nullptr;
    std::uint8_t uuid_[16] = {};
    BtpSealFn seal_ = nullptr;
    void* seal_context_ = nullptr;
    const SourceInfoEntry* source_info_ = nullptr;
    std::size_t source_info_count_ = 0U;
    btp::StaticCatalog<kMaxCatalogTopics, kMaxCatalogFields,
                       kMaxCatalogStringBytes> catalog_;
    bool catalog_valid_ = false;
};

#endif  // MANIFEST_RESPONDER_H
