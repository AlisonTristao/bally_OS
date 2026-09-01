#include <ManifestResponder.h>

#include <BtpTransport.h>
#include <TelemetryPublisher.h>
#include <btp/messages.hpp>

#include <cstring>

namespace {

// Common result/error codes, BTP/docs/commands.md section 1 ("Result and
// error codes") -- mirrors CommandProcessor::Status/ErrorCode (same source of
// truth, kept as local constants here rather than pulling in a dependency on
// CommandProcessor for two enum values).
constexpr std::uint8_t kStatusSuccess = 0x00U;
constexpr std::uint8_t kStatusRejected = 0x01U;
constexpr std::uint16_t kErrorNone = 0x0000U;
constexpr std::uint16_t kErrorStaleTargetBoot = 0x0009U;
constexpr std::uint16_t kErrorNotFound = 0x000BU;

// MANIFEST_DATA flags (commands.md section 3.2).
constexpr std::uint8_t kFlagNotModified = 0x01U;
constexpr std::uint8_t kFlagCatalogComplete = 0x02U;
constexpr std::uint8_t kSourceFlagOnline = 0x01U;

std::uint8_t wire_type_code(TelemetryPublisher::WireType type) noexcept {
    switch (type) {
        case TelemetryPublisher::WireType::Uint8: return 0x01U;
        case TelemetryPublisher::WireType::Uint32: return 0x03U;
        case TelemetryPublisher::WireType::Float32: return 0x09U;
    }
    return 0U;
}

btp::ByteView view_of(const char* text) noexcept {
    return {reinterpret_cast<const std::uint8_t*>(text),
            (text == nullptr) ? 0U : std::strlen(text)};
}

// Writes the full MANIFEST_DATA payload via btp::ManifestWriter. For a
// non-SUCCESS status (REJECTED / NOT_FOUND), the response describes no source:
// identity / role / revision / counts are all zero and `name` (if non-null)
// carries a short human diagnostic instead of the source name -- commands.md
// section 3.2. Returns 0 on any capacity failure.
std::size_t build_manifest_data(const btp::Header& request_header, std::uint8_t status, std::uint8_t flags,
                                std::uint16_t error_code, const std::uint8_t uuid[16],
                                std::uint32_t described_source_id, std::uint32_t described_boot_id,
                                std::uint32_t config_revision, const char* name,
                                const SourceInfoEntry* source_info, std::size_t source_info_count,
                                std::uint8_t* output, std::size_t capacity) noexcept {
    const bool error = (status != kStatusSuccess);
    const bool notModified = (flags & kFlagNotModified) != 0U;

    std::size_t topicCount = 0U;
    const TelemetryPublisher::TopicSchema* schemas = nullptr;
    if (!error && !notModified) {
        schemas = TelemetryPublisher::schemas(&topicCount);
    }

    static const std::uint8_t kZeroUuid[16] = {0};
    btp::ManifestHeader header{};
    header.request = {request_header.source_id, request_header.boot_id, request_header.sequence};
    header.status = status;
    header.flags = flags;
    header.error_code = error_code;
    header.manifest_format_version = ManifestResponder::kManifestFormatVersion;  // always 2
    header.config_revision = error ? 0U : config_revision;
    std::memcpy(header.source_uuid, error ? kZeroUuid : uuid, 16U);
    header.described_source_id = error ? 0U : described_source_id;
    header.described_boot_id = error ? 0U : described_boot_id;
    header.source_role = error ? 0U : ManifestResponder::kSourceRoleRobot;
    header.source_flags = error ? 0U : kSourceFlagOnline;
    header.catalog_index = 0U;
    header.catalog_count = 1U;  // content zeroed on error, but still one entry
    header.topic_count = static_cast<std::uint16_t>(topicCount);
    header.action_count = 0U;
    header.source_name = view_of(error ? name : "bally_software");

    btp::ManifestWriter writer(output, capacity);
    if (writer.begin(header) != btp::MessageError::Ok) return 0U;

    // source_info block (commands.md 3.12): informational, rides SUCCESS and
    // NOT_MODIFIED alike, empty on an error. Whole entries only, and only while
    // they leave kRecordsReserveBytes for the topic records after them -- the
    // schema a consumer needs to decode telemetry wins over a human-facing row.
    // writer.size() is the running position; an unset value is skipped.
    const std::size_t infoClamped =
        (error || source_info == nullptr)
            ? 0U
            : (source_info_count > ManifestResponder::kMaxSourceInfoEntries
                   ? ManifestResponder::kMaxSourceInfoEntries
                   : source_info_count);
    const std::size_t infoBudgetEnd = (capacity > ManifestResponder::kRecordsReserveBytes)
                                          ? (capacity - ManifestResponder::kRecordsReserveBytes)
                                          : writer.size();
    for (std::size_t i = 0U; i < infoClamped; ++i) {
        const char* key = source_info[i].key;
        const char* label = source_info[i].label;
        const char* value = source_info[i].value;
        if (value == nullptr || value[0] == '\0') continue;
        const std::size_t entrySize = 6U + std::strlen(key != nullptr ? key : "") +
                                      std::strlen(label != nullptr ? label : "") + std::strlen(value);
        if (writer.size() + entrySize > infoBudgetEnd) break;
        const btp::SourceInfoEntry entry{view_of(key), view_of(label), view_of(value)};
        if (writer.add_source_info(entry) != btp::MessageError::Ok) return 0U;
    }

    for (std::size_t t = 0U; t < topicCount; ++t) {
        const TelemetryPublisher::TopicSchema& topic = schemas[t];
        btp::TopicRecord record{};
        record.topic_id = topic.topic_id;
        record.schema_version = topic.schema_version;
        record.encoding = static_cast<std::uint8_t>(topic.encoding);
        record.flags = btp::kTopicSubscribable;
        record.field_count = static_cast<std::uint16_t>(topic.field_count);
        record.max_rate_millihz = topic.max_rate_millihz;
        record.name = view_of(topic.name);
        record.description = view_of("");
        if (writer.begin_topic(record) != btp::MessageError::Ok) return 0U;

        for (std::size_t f = 0U; f < topic.field_count; ++f) {
            const TelemetryPublisher::FieldSchema& field = topic.fields[f];
            btp::FieldRecord fr{};
            fr.field_id = field.field_id;
            fr.order = field.order;
            fr.type = wire_type_code(field.type);
            fr.flags = field.nullable ? btp::kFieldNullable : static_cast<std::uint8_t>(0U);
            fr.element_count = field.element_count;
            fr.max_element_count = 0U;  // fixed-size
            fr.scale = static_cast<double>(field.scale);
            fr.offset = static_cast<double>(field.offset);
            fr.enum_count = 0U;  // no enum fields yet
            fr.name = view_of(field.name);
            fr.unit = view_of(field.unit);
            fr.description = view_of("");
            if (writer.add_field(fr) != btp::MessageError::Ok) return 0U;
        }
        if (writer.end_topic() != btp::MessageError::Ok) return 0U;
    }
    // No action records yet (action_count = 0; topico 18's territory).

    std::size_t written = 0U;
    if (writer.finish(&written) != btp::MessageError::Ok) return 0U;
    return written;
}

}  // namespace

void ManifestResponder::configure(BtpEndpoint& endpoint, const std::uint8_t uuid[16], BtpSealFn seal,
                                  void* seal_context, const SourceInfoEntry* source_info,
                                  std::size_t source_info_count) noexcept {
    endpoint_ = &endpoint;
    if (uuid != nullptr) std::memcpy(uuid_, uuid, 16U);
    seal_ = seal;
    seal_context_ = seal_context;
    source_info_ = source_info;
    source_info_count_ = (source_info == nullptr) ? 0U : source_info_count;
}

bool ManifestResponder::handle_request(const btp::Header& request_header, btp::ByteView payload,
                                       std::uint64_t timestamp_us) noexcept {
    if (endpoint_ == nullptr) return false;

    // The 12-octet MANIFEST_REQUEST layout (commands.md section 3.1) is
    // btp::decode_manifest_request.
    btp::ManifestRequest request{};
    if (btp::decode_manifest_request(payload.data, payload.size, &request) != btp::MessageError::Ok) {
        return false;
    }
    const std::uint32_t targetSourceId = request.target_source_id;
    const std::uint32_t targetBootId = request.target_boot_id;
    const std::uint32_t knownRevision = request.known_config_revision;

    const std::uint32_t localSourceId = endpoint_->source_id();
    const std::uint32_t localBootId = endpoint_->boot_id();

    std::uint8_t responsePayload[kMaxManifestPayloadSize];
    std::size_t responseSize = 0U;

    if (targetSourceId != 0U && targetSourceId != localSourceId) {
        // Not this robot's identity -- a leaf node only knows how to
        // describe itself (see class comment).
        responseSize = build_manifest_data(request_header, kStatusRejected, kFlagCatalogComplete, kErrorNotFound,
                                           uuid_, 0U, 0U, 0U, "unknown source", nullptr, 0U, responsePayload,
                                           sizeof(responsePayload));
    } else if (targetBootId != 0U && targetBootId != localBootId) {
        responseSize = build_manifest_data(request_header, kStatusRejected, kFlagCatalogComplete,
                                           kErrorStaleTargetBoot, uuid_, 0U, 0U, 0U, "boot mismatch", nullptr, 0U,
                                           responsePayload, sizeof(responsePayload));
    } else if (knownRevision != 0U && knownRevision == kConfigRevision) {
        responseSize = build_manifest_data(request_header, kStatusSuccess,
                                           static_cast<std::uint8_t>(kFlagNotModified | kFlagCatalogComplete),
                                           kErrorNone, uuid_, localSourceId, localBootId, kConfigRevision, nullptr,
                                           source_info_, source_info_count_, responsePayload,
                                           sizeof(responsePayload));
    } else {
        responseSize = build_manifest_data(request_header, kStatusSuccess, kFlagCatalogComplete, kErrorNone, uuid_,
                                           localSourceId, localBootId, kConfigRevision, nullptr, source_info_,
                                           source_info_count_, responsePayload, sizeof(responsePayload));
    }

    if (responseSize == 0U) return false;

    return endpoint_->send_logical(btp::MessageType::Control, kManifestDataObjectId, responsePayload, responseSize,
                                   timestamp_us, seal_, seal_context_);
}
