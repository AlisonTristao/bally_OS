#include <ManifestResponder.h>

#include <BtpTransport.h>
#include <TelemetryPublisher.h>

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

// Small append-only cursor over a fixed output buffer. Every append method
// returns false (and leaves the buffer untouched beyond what already
// succeeded) on overflow, so a caller can bail out cleanly rather than
// overrun kMaxManifestPayloadSize.
class Writer {
public:
    Writer(std::uint8_t* out, std::size_t capacity) noexcept : out_(out), capacity_(capacity) {}

    bool u8(std::uint8_t value) noexcept { return raw(&value, 1U); }

    bool u16(std::uint16_t value) noexcept {
        const std::uint8_t bytes[2] = {static_cast<std::uint8_t>(value),
                                       static_cast<std::uint8_t>(value >> 8U)};
        return raw(bytes, 2U);
    }

    bool u32(std::uint32_t value) noexcept {
        const std::uint8_t bytes[4] = {
            static_cast<std::uint8_t>(value), static_cast<std::uint8_t>(value >> 8U),
            static_cast<std::uint8_t>(value >> 16U), static_cast<std::uint8_t>(value >> 24U)};
        return raw(bytes, 4U);
    }

    bool f64(double value) noexcept {
        std::uint8_t bytes[8];
        std::memcpy(bytes, &value, sizeof(bytes));  // host is little-endian (ESP32/x86)
        return raw(bytes, 8U);
    }

    bool bytes16(const std::uint8_t* data, std::size_t size) noexcept { return raw(data, size); }

    bool utf8(const char* text) noexcept {
        const std::size_t len = (text == nullptr) ? 0U : std::strlen(text);
        if (len > 0xFFFFU) return false;
        if (!u16(static_cast<std::uint16_t>(len))) return false;
        return len == 0U || raw(reinterpret_cast<const std::uint8_t*>(text), len);
    }

    std::size_t size() const noexcept { return pos_; }

    bool reserveU32(std::size_t* offset_out) noexcept {
        *offset_out = pos_;
        return u32(0U);
    }

    void patchU32(std::size_t offset, std::uint32_t value) noexcept {
        out_[offset] = static_cast<std::uint8_t>(value);
        out_[offset + 1U] = static_cast<std::uint8_t>(value >> 8U);
        out_[offset + 2U] = static_cast<std::uint8_t>(value >> 16U);
        out_[offset + 3U] = static_cast<std::uint8_t>(value >> 24U);
    }

private:
    bool raw(const std::uint8_t* data, std::size_t n) noexcept {
        if (pos_ + n > capacity_) return false;
        std::memcpy(out_ + pos_, data, n);
        pos_ += n;
        return true;
    }

    std::uint8_t* out_;
    std::size_t capacity_;
    std::size_t pos_ = 0U;
};

std::uint32_t read_u32_le(const std::uint8_t* data) noexcept {
    return static_cast<std::uint32_t>(data[0]) | (static_cast<std::uint32_t>(data[1]) << 8U) |
           (static_cast<std::uint32_t>(data[2]) << 16U) | (static_cast<std::uint32_t>(data[3]) << 24U);
}

std::uint8_t wire_type_code(TelemetryPublisher::WireType type) noexcept {
    switch (type) {
        case TelemetryPublisher::WireType::Uint8: return 0x01U;
        case TelemetryPublisher::WireType::Uint32: return 0x03U;
        case TelemetryPublisher::WireType::Float32: return 0x09U;
    }
    return 0U;
}

bool write_field_record(Writer& writer, const TelemetryPublisher::FieldSchema& field) noexcept {
    std::size_t size_offset = 0U;
    if (!writer.reserveU32(&size_offset)) return false;
    const std::size_t content_start = writer.size();

    const bool ok = writer.u16(field.field_id) && writer.u16(field.order) &&
                    writer.u8(wire_type_code(field.type)) &&
                    writer.u8(field.nullable ? 0x01U : 0x00U) &&  // flags: bit0 NULLABLE
                    writer.u16(field.element_count) &&
                    writer.u16(0U) &&  // max_element_count: fixed-size, always zero
                    writer.f64(static_cast<double>(field.scale)) &&
                    writer.f64(static_cast<double>(field.offset)) &&
                    writer.u16(0U) &&  // enum_count: no enum fields yet
                    writer.utf8(field.name) && writer.utf8(field.unit) && writer.utf8("");
    if (!ok) return false;

    writer.patchU32(size_offset, static_cast<std::uint32_t>(writer.size() - content_start));
    return true;
}

bool write_topic_record(Writer& writer, const TelemetryPublisher::TopicSchema& topic) noexcept {
    std::size_t size_offset = 0U;
    if (!writer.reserveU32(&size_offset)) return false;
    const std::size_t content_start = writer.size();

    const bool ok = writer.u16(topic.topic_id) && writer.u16(topic.schema_version) &&
                    writer.u8(static_cast<std::uint8_t>(topic.encoding)) &&
                    writer.u8(0x01U) &&  // flags: bit0 SUBSCRIBABLE
                    writer.u16(static_cast<std::uint16_t>(topic.field_count)) &&
                    writer.u32(topic.max_rate_millihz) && writer.utf8(topic.name) && writer.utf8("");
    if (!ok) return false;

    for (std::size_t i = 0U; i < topic.field_count; ++i) {
        if (!write_field_record(writer, topic.fields[i])) return false;
    }

    writer.patchU32(size_offset, static_cast<std::uint32_t>(writer.size() - content_start));
    return true;
}

// Writes the full MANIFEST_DATA payload. For an error status (REJECTED),
// identity/role/flags/revision/topic&action counts are all zero and `name`
// (if non-null) carries a short human diagnostic instead of the source name
// -- commands.md section 3.2, the error case.
std::size_t build_manifest_data(const btp::Header& request_header, std::uint8_t status, std::uint8_t flags,
                                std::uint16_t error_code, const std::uint8_t uuid[16],
                                std::uint32_t described_source_id, std::uint32_t described_boot_id,
                                std::uint32_t config_revision, const char* name, std::uint8_t* output,
                                std::size_t capacity) noexcept {
    Writer writer(output, capacity);
    const bool error = (status != kStatusSuccess);

    bool ok = writer.u32(request_header.source_id) && writer.u32(request_header.boot_id) &&
             writer.u32(request_header.sequence) && writer.u8(status) && writer.u8(flags) &&
             writer.u16(error_code) && writer.u16(ManifestResponder::kManifestFormatVersion) &&
             writer.u16(0U) /*reserved*/ && writer.u32(error ? 0U : config_revision);
    if (!ok) return 0U;

    static const std::uint8_t kZeroUuid[16] = {0};
    ok = writer.bytes16(error ? kZeroUuid : uuid, 16U) &&
        writer.u32(error ? 0U : described_source_id) && writer.u32(error ? 0U : described_boot_id) &&
        writer.u8(error ? 0U : ManifestResponder::kSourceRoleRobot) &&
        writer.u8(error ? 0U : kSourceFlagOnline) && writer.u16(0U) /*catalog_index*/ &&
        writer.u16(1U) /*catalog_count*/;
    if (!ok) return 0U;

    const bool notModified = (flags & kFlagNotModified) != 0U;

    std::size_t topicCount = 0U;
    const TelemetryPublisher::TopicSchema* schemas = nullptr;
    if (!error && !notModified) {
        schemas = TelemetryPublisher::schemas(&topicCount);
    }

    if (!writer.u16(static_cast<std::uint16_t>(topicCount)) || !writer.u16(0U) /*action_count*/) {
        return 0U;
    }
    if (!writer.utf8(error ? name : "bally_software")) {
        return 0U;
    }

    for (std::size_t i = 0U; i < topicCount; ++i) {
        if (!write_topic_record(writer, schemas[i])) return 0U;
    }
    // No action records yet (action_count=0 above; topico 18's territory).

    return writer.size();
}

}  // namespace

void ManifestResponder::configure(BtpEndpoint& endpoint, const std::uint8_t uuid[16], BtpSealFn seal,
                                  void* seal_context) noexcept {
    endpoint_ = &endpoint;
    if (uuid != nullptr) std::memcpy(uuid_, uuid, 16U);
    seal_ = seal;
    seal_context_ = seal_context;
}

bool ManifestResponder::handle_request(const btp::Header& request_header, btp::ByteView payload,
                                       std::uint64_t timestamp_us) noexcept {
    if (endpoint_ == nullptr || payload.data == nullptr || payload.size < 12U) return false;

    const std::uint32_t targetSourceId = read_u32_le(payload.data);
    const std::uint32_t targetBootId = read_u32_le(payload.data + 4U);
    const std::uint32_t knownRevision = read_u32_le(payload.data + 8U);

    const std::uint32_t localSourceId = endpoint_->source_id();
    const std::uint32_t localBootId = endpoint_->boot_id();

    std::uint8_t responsePayload[kMaxManifestPayloadSize];
    std::size_t responseSize = 0U;

    if (targetSourceId != 0U && targetSourceId != localSourceId) {
        // Not this robot's identity -- a leaf node only knows how to
        // describe itself (see class comment).
        responseSize = build_manifest_data(request_header, kStatusRejected, kFlagCatalogComplete, kErrorNotFound,
                                           uuid_, 0U, 0U, 0U, "unknown source", responsePayload,
                                           sizeof(responsePayload));
    } else if (targetBootId != 0U && targetBootId != localBootId) {
        responseSize = build_manifest_data(request_header, kStatusRejected, kFlagCatalogComplete,
                                           kErrorStaleTargetBoot, uuid_, 0U, 0U, 0U, "boot mismatch",
                                           responsePayload, sizeof(responsePayload));
    } else if (knownRevision != 0U && knownRevision == kConfigRevision) {
        responseSize = build_manifest_data(request_header, kStatusSuccess,
                                           static_cast<std::uint8_t>(kFlagNotModified | kFlagCatalogComplete),
                                           kErrorNone, uuid_, localSourceId, localBootId, kConfigRevision, nullptr,
                                           responsePayload, sizeof(responsePayload));
    } else {
        responseSize = build_manifest_data(request_header, kStatusSuccess, kFlagCatalogComplete, kErrorNone, uuid_,
                                           localSourceId, localBootId, kConfigRevision, nullptr, responsePayload,
                                           sizeof(responsePayload));
    }

    if (responseSize == 0U) return false;

    return endpoint_->send_logical(btp::MessageType::Control, kManifestDataObjectId, responsePayload, responseSize,
                                   timestamp_us, seal_, seal_context_);
}
