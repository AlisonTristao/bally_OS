#include <unity.h>

#include <BtpTransport.h>
#include <ManifestResponder.h>
#include <TelemetryPublisher.h>
#include <btp/codec.hpp>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// ManifestResponder is deliberately pure C++ (see its class comment: "same
// shape as BtpTransport, so it links into env:native"), and its own header
// promises that a MANIFEST_DATA response "can never drift from what is
// actually published on the wire" because it is built straight from
// TelemetryPublisher::schemas(). This suite is what actually holds that
// promise to account: it decodes the wire bytes ManifestResponder produces
// and checks every field against the schema table itself, rather than
// against a second, hand-copied expectation that could drift right along
// with a bug.

namespace {

// Mirrors the anonymous-namespace constants in ManifestResponder.cpp
// (BTP/docs/commands.md section 1 "Result and error codes" / section 3.2).
// Not reused from CommandProcessor::Status/ErrorCode because NotFound
// (0x000B) has no CommandProcessor equivalent -- MANIFEST_REQUEST is the
// only place this firmware answers "I don't describe that source".
constexpr std::uint8_t kStatusSuccess = 0x00U;
constexpr std::uint8_t kStatusRejected = 0x01U;
constexpr std::uint16_t kErrorNone = 0x0000U;
constexpr std::uint16_t kErrorStaleTargetBoot = 0x0009U;
constexpr std::uint16_t kErrorNotFound = 0x000BU;
constexpr std::uint8_t kFlagNotModified = 0x01U;
constexpr std::uint8_t kFlagCatalogComplete = 0x02U;
constexpr std::uint8_t kSourceFlagOnline = 0x01U;

std::uint8_t sent_frames[4][btp::kEspNowMaxFrameSize]{};
std::size_t sent_sizes[4]{};
std::size_t sent_count = 0U;

bool capture_send(const std::uint8_t* data, std::size_t size) {
    if (sent_count >= 4U || size > btp::kEspNowMaxFrameSize) return false;
    std::memcpy(sent_frames[sent_count], data, size);
    sent_sizes[sent_count] = size;
    ++sent_count;
    return true;
}

std::uint32_t read_u32_le(const std::uint8_t* data) {
    return static_cast<std::uint32_t>(data[0]) |
          (static_cast<std::uint32_t>(data[1]) << 8U) |
          (static_cast<std::uint32_t>(data[2]) << 16U) |
          (static_cast<std::uint32_t>(data[3]) << 24U);
}

// Unity's double-precision asserts (TEST_ASSERT_EQUAL_DOUBLE) need
// UNITY_INCLUDE_DOUBLE, which this build doesn't define. scale/offset never
// go through arithmetic here -- Writer::f64() just widens the schema's float
// to double and memcpy's it onto the wire -- so an exact bit-pattern compare
// is both sufficient and avoids depending on that build flag.
std::uint64_t double_bits(double value) {
    std::uint64_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

void write_u32(std::uint8_t* out, std::uint32_t value) {
    out[0] = static_cast<std::uint8_t>(value);
    out[1] = static_cast<std::uint8_t>(value >> 8U);
    out[2] = static_cast<std::uint8_t>(value >> 16U);
    out[3] = static_cast<std::uint8_t>(value >> 24U);
}

// Field type codes MANIFEST_DATA puts on the wire (BTP/docs/commands.md
// field-type table), same numbers ManifestResponder.cpp's local
// wire_type_code() uses. Kept here too so a topic/field record can be
// checked byte-for-byte, not just "some type was written".
std::uint8_t expected_wire_type_code(TelemetryPublisher::WireType type) {
    switch (type) {
        case TelemetryPublisher::WireType::Uint8: return 0x01U;
        case TelemetryPublisher::WireType::Uint32: return 0x03U;
        case TelemetryPublisher::WireType::Float32: return 0x09U;
    }
    return 0U;
}

// A request payload is exactly the 12-octet
// target_source_id/target_boot_id/known_config_revision triple (commands.md
// section 3.1); handle_request rejects anything shorter.
std::vector<std::uint8_t> manifest_request_payload(std::uint32_t target_source_id,
                                                    std::uint32_t target_boot_id,
                                                    std::uint32_t known_revision) {
    std::vector<std::uint8_t> payload(12U);
    write_u32(payload.data(), target_source_id);
    write_u32(payload.data() + 4U, target_boot_id);
    write_u32(payload.data() + 8U, known_revision);
    return payload;
}

btp::Header manifest_request_header(std::uint32_t requester_source_id,
                                    std::uint32_t requester_boot_id,
                                    std::uint32_t sequence) {
    return {
        .type = btp::MessageType::Control,
        .flags = 0U,
        .source_id = requester_source_id,
        .boot_id = requester_boot_id,
        .sequence = sequence,
        .timestamp_us = 42U,
        .object_id = ManifestResponder::kManifestRequestObjectId,
        .fragment_index = 0U,
        .fragment_count = 1U,
    };
}

// Sends the request, decodes every captured fragment (there may be more than
// one -- MANIFEST_DATA can exceed one ESP-NOW frame once topic/field records
// are included) and concatenates their payloads back into one logical
// MANIFEST_DATA buffer, the same reassembly a real peer would do. Fragments
// are captured in send order, which BtpEndpoint::send_logical always emits
// as index 0..count-1 (see BtpTransport.cpp), so no reordering is needed.
std::vector<std::uint8_t> send_and_reassemble(BtpEndpoint& endpoint,
                                              ManifestResponder& responder,
                                              const btp::Header& request_header,
                                              const std::vector<std::uint8_t>& request_payload) {
    sent_count = 0U;
    const btp::ByteView view{request_payload.data(), request_payload.size()};
    TEST_ASSERT_TRUE(responder.handle_request(request_header, view, 100000U));
    TEST_ASSERT_GREATER_THAN_UINT32(0U, sent_count);

    std::vector<std::uint8_t> logical;
    std::uint8_t expected_count = 0U;
    for (std::size_t i = 0U; i < sent_count; ++i) {
        btp::DecodedFrame decoded{};
        TEST_ASSERT_EQUAL_UINT8(
            static_cast<std::uint8_t>(btp::Error::Ok),
            static_cast<std::uint8_t>(btp::decode(
                sent_frames[i], sent_sizes[i], btp::TransportProfile::EspNow,
                &decoded)));
        TEST_ASSERT_EQUAL(static_cast<int>(btp::MessageType::Control),
                          static_cast<int>(decoded.header.type));
        TEST_ASSERT_EQUAL_HEX16(ManifestResponder::kManifestDataObjectId,
                                decoded.header.object_id);
        // The response is this robot speaking, not an echo of the
        // requester's identity.
        TEST_ASSERT_EQUAL_HEX32(endpoint.source_id(), decoded.header.source_id);
        TEST_ASSERT_EQUAL_HEX32(endpoint.boot_id(), decoded.header.boot_id);
        TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(i),
                                decoded.header.fragment_index);
        if (i == 0U) {
            expected_count = decoded.header.fragment_count;
        } else {
            TEST_ASSERT_EQUAL_UINT8(expected_count, decoded.header.fragment_count);
        }
        logical.insert(logical.end(), decoded.payload.data,
                       decoded.payload.data + decoded.payload.size);
    }
    TEST_ASSERT_EQUAL_UINT32(expected_count, sent_count);
    return logical;
}

// Bounds-checked cursor over the reassembled MANIFEST_DATA payload, the
// mirror image of ManifestResponder.cpp's own Writer. Every accessor fails
// the current test (via TEST_ASSERT) rather than returning an error code, so
// a framing bug points straight at the field that broke instead of trailing
// off into "size mismatch" at the very end.
class Reader {
public:
    explicit Reader(const std::vector<std::uint8_t>& data) : data_(data) {}

    std::uint8_t u8() {
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(data_.size(), pos_ + 1U);
        return data_[pos_++];
    }
    std::uint16_t u16() {
        const std::uint8_t lo = u8();
        const std::uint8_t hi = u8();
        return static_cast<std::uint16_t>(lo) | (static_cast<std::uint16_t>(hi) << 8U);
    }
    std::uint32_t u32() {
        const std::uint16_t lo = u16();
        const std::uint16_t hi = u16();
        return static_cast<std::uint32_t>(lo) | (static_cast<std::uint32_t>(hi) << 16U);
    }
    double f64() {
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(data_.size(), pos_ + 8U);
        double value = 0.0;
        std::memcpy(&value, data_.data() + pos_, sizeof(value));
        pos_ += 8U;
        return value;
    }
    void bytes(std::uint8_t* out, std::size_t n) {
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(data_.size(), pos_ + n);
        std::memcpy(out, data_.data() + pos_, n);
        pos_ += n;
    }
    std::string utf8() {
        const std::uint16_t len = u16();
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(data_.size(), pos_ + len);
        std::string value(reinterpret_cast<const char*>(data_.data() + pos_), len);
        pos_ += len;
        return value;
    }
    std::size_t pos() const { return pos_; }

private:
    const std::vector<std::uint8_t>& data_;
    std::size_t pos_ = 0U;
};

// One robot identity + one requester identity, reused by every test so only
// the request payload (and occasionally the header) needs to vary.
constexpr std::uint32_t kLocalSourceId = 0xAABBCCDDU;
constexpr std::uint32_t kLocalBootId = 0x99887766U;
constexpr std::uint32_t kRequesterSourceId = 0x11112222U;
constexpr std::uint32_t kRequesterBootId = 0x33334444U;
constexpr std::uint32_t kRequestSequence = 77U;

const std::uint8_t kUuid[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

void make_endpoint_and_responder(BtpEndpoint* endpoint, ManifestResponder* responder) {
    TEST_ASSERT_TRUE(endpoint->configure(kLocalSourceId, kLocalBootId));
    endpoint->set_send_callback(capture_send);
    responder->configure(*endpoint, kUuid);
}

// ---------------------------------------------------------------------------
// A wildcard request (target_source_id=0, target_boot_id=0) with an unknown
// revision (0) gets the full catalog back, and every topic/field record in
// it must match TelemetryPublisher::schemas() exactly -- that agreement is
// the entire reason ManifestResponder exists instead of a second, hand-kept
// copy of the schema table.
// ---------------------------------------------------------------------------
void test_full_manifest_response_matches_telemetry_schemas() {
    BtpEndpoint endpoint;
    ManifestResponder responder;
    make_endpoint_and_responder(&endpoint, &responder);

    const auto request_payload = manifest_request_payload(0U, 0U, 0U);
    const auto header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                kRequestSequence);
    const auto logical = send_and_reassemble(endpoint, responder, header, request_payload);

    Reader reader(logical);
    TEST_ASSERT_EQUAL_HEX32(kRequesterSourceId, reader.u32());
    TEST_ASSERT_EQUAL_HEX32(kRequesterBootId, reader.u32());
    TEST_ASSERT_EQUAL_UINT32(kRequestSequence, reader.u32());
    TEST_ASSERT_EQUAL_UINT8(kStatusSuccess, reader.u8());
    TEST_ASSERT_EQUAL_UINT8(kFlagCatalogComplete, reader.u8());
    TEST_ASSERT_EQUAL_HEX16(kErrorNone, reader.u16());
    TEST_ASSERT_EQUAL_UINT16(ManifestResponder::kManifestFormatVersion, reader.u16());
    TEST_ASSERT_EQUAL_UINT16(0U, reader.u16());  // reserved
    TEST_ASSERT_EQUAL_UINT32(ManifestResponder::kConfigRevision, reader.u32());

    std::uint8_t uuid[16]{};
    reader.bytes(uuid, sizeof(uuid));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(kUuid, uuid, sizeof(uuid));

    TEST_ASSERT_EQUAL_HEX32(kLocalSourceId, reader.u32());  // described_source_id
    TEST_ASSERT_EQUAL_HEX32(kLocalBootId, reader.u32());    // described_boot_id
    TEST_ASSERT_EQUAL_UINT8(ManifestResponder::kSourceRoleRobot, reader.u8());
    TEST_ASSERT_EQUAL_UINT8(kSourceFlagOnline, reader.u8());
    TEST_ASSERT_EQUAL_UINT16(0U, reader.u16());  // catalog_index
    TEST_ASSERT_EQUAL_UINT16(1U, reader.u16());  // catalog_count

    std::size_t expected_topic_count = 0U;
    const TelemetryPublisher::TopicSchema* schemas =
        TelemetryPublisher::schemas(&expected_topic_count);
    TEST_ASSERT_GREATER_THAN_UINT32(0U, expected_topic_count);

    const std::uint16_t topic_count = reader.u16();
    TEST_ASSERT_EQUAL_UINT32(expected_topic_count, topic_count);
    TEST_ASSERT_EQUAL_UINT16(0U, reader.u16());  // action_count
    TEST_ASSERT_EQUAL_STRING("bally_software", reader.utf8().c_str());

    for (std::size_t t = 0U; t < expected_topic_count; ++t) {
        const TelemetryPublisher::TopicSchema& topic = schemas[t];

        const std::uint32_t topic_record_size = reader.u32();
        const std::size_t topic_content_start = reader.pos();

        TEST_ASSERT_EQUAL_HEX16(topic.topic_id, reader.u16());
        TEST_ASSERT_EQUAL_UINT16(topic.schema_version, reader.u16());
        TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(topic.encoding), reader.u8());
        TEST_ASSERT_EQUAL_UINT8(0x01U, reader.u8());  // flags: SUBSCRIBABLE
        TEST_ASSERT_EQUAL_UINT32(topic.field_count, reader.u16());
        TEST_ASSERT_EQUAL_UINT32(topic.max_rate_millihz, reader.u32());
        TEST_ASSERT_EQUAL_STRING(topic.name, reader.utf8().c_str());
        TEST_ASSERT_EQUAL_STRING("", reader.utf8().c_str());

        for (std::size_t f = 0U; f < topic.field_count; ++f) {
            const TelemetryPublisher::FieldSchema& field = topic.fields[f];

            const std::uint32_t field_record_size = reader.u32();
            const std::size_t field_content_start = reader.pos();

            TEST_ASSERT_EQUAL_UINT16(field.field_id, reader.u16());
            TEST_ASSERT_EQUAL_UINT16(field.order, reader.u16());
            TEST_ASSERT_EQUAL_UINT8(expected_wire_type_code(field.type), reader.u8());
            TEST_ASSERT_EQUAL_UINT8(field.nullable ? 0x01U : 0x00U, reader.u8());
            TEST_ASSERT_EQUAL_UINT16(field.element_count, reader.u16());
            TEST_ASSERT_EQUAL_UINT16(0U, reader.u16());  // max_element_count
            TEST_ASSERT_EQUAL_UINT64(double_bits(static_cast<double>(field.scale)),
                                     double_bits(reader.f64()));
            TEST_ASSERT_EQUAL_UINT64(double_bits(static_cast<double>(field.offset)),
                                     double_bits(reader.f64()));
            TEST_ASSERT_EQUAL_UINT16(0U, reader.u16());  // enum_count
            TEST_ASSERT_EQUAL_STRING(field.name, reader.utf8().c_str());
            TEST_ASSERT_EQUAL_STRING(field.unit, reader.utf8().c_str());
            TEST_ASSERT_EQUAL_STRING("", reader.utf8().c_str());

            TEST_ASSERT_EQUAL_UINT32(field_record_size, reader.pos() - field_content_start);
        }

        TEST_ASSERT_EQUAL_UINT32(topic_record_size, reader.pos() - topic_content_start);
    }

    // Every octet was accounted for -- no trailing garbage, no under-read.
    TEST_ASSERT_EQUAL_UINT32(logical.size(), reader.pos());
}

// ---------------------------------------------------------------------------
// A request explicitly targeted at this robot's own (source_id, boot_id)
// takes the same success path as the wildcard -- targeting yourself by name
// is not a different case from targeting nobody in particular.
// ---------------------------------------------------------------------------
void test_targeted_request_matching_this_robot_succeeds() {
    BtpEndpoint endpoint;
    ManifestResponder responder;
    make_endpoint_and_responder(&endpoint, &responder);

    const auto request_payload =
        manifest_request_payload(kLocalSourceId, kLocalBootId, 0U);
    const auto header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                kRequestSequence);
    const auto logical = send_and_reassemble(endpoint, responder, header, request_payload);

    Reader reader(logical);
    reader.u32();
    reader.u32();
    reader.u32();
    TEST_ASSERT_EQUAL_UINT8(kStatusSuccess, reader.u8());
}

// ---------------------------------------------------------------------------
// A request for a revision the caller already has (NOT_MODIFIED) still
// succeeds, but sheds every topic record -- the caller already knows them.
// ---------------------------------------------------------------------------
void test_known_revision_returns_not_modified_with_no_topics() {
    BtpEndpoint endpoint;
    ManifestResponder responder;
    make_endpoint_and_responder(&endpoint, &responder);

    const auto request_payload =
        manifest_request_payload(0U, 0U, ManifestResponder::kConfigRevision);
    const auto header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                kRequestSequence);
    const auto logical = send_and_reassemble(endpoint, responder, header, request_payload);

    Reader reader(logical);
    reader.u32();
    reader.u32();
    reader.u32();
    TEST_ASSERT_EQUAL_UINT8(kStatusSuccess, reader.u8());
    TEST_ASSERT_EQUAL_UINT8(kFlagNotModified | kFlagCatalogComplete, reader.u8());
    TEST_ASSERT_EQUAL_HEX16(kErrorNone, reader.u16());
    reader.u16();  // format_version
    reader.u16();  // reserved
    TEST_ASSERT_EQUAL_UINT32(ManifestResponder::kConfigRevision, reader.u32());
    std::uint8_t uuid[16]{};
    reader.bytes(uuid, sizeof(uuid));
    reader.u32();  // described_source_id
    reader.u32();  // described_boot_id
    reader.u8();   // source_role
    reader.u8();   // source_flags
    reader.u16();  // catalog_index
    reader.u16();  // catalog_count
    TEST_ASSERT_EQUAL_UINT16(0U, reader.u16());  // topic_count
    TEST_ASSERT_EQUAL_UINT16(0U, reader.u16());  // action_count
    TEST_ASSERT_EQUAL_STRING("bally_software", reader.utf8().c_str());
    TEST_ASSERT_EQUAL_UINT32(logical.size(), reader.pos());
}

// ---------------------------------------------------------------------------
// This robot only ever describes itself (class comment: "a leaf node only
// knows how to describe itself"), so a request naming a different source_id
// must be answered, not ignored -- and answered with an all-zeroed identity
// so a misrouted request can never be mistaken for a real description.
// ---------------------------------------------------------------------------
void test_request_for_different_source_is_rejected_not_found() {
    BtpEndpoint endpoint;
    ManifestResponder responder;
    make_endpoint_and_responder(&endpoint, &responder);

    const auto request_payload = manifest_request_payload(0xDEADBEEFU, 0U, 0U);
    const auto header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                kRequestSequence);
    const auto logical = send_and_reassemble(endpoint, responder, header, request_payload);

    Reader reader(logical);
    reader.u32();
    reader.u32();
    reader.u32();
    TEST_ASSERT_EQUAL_UINT8(kStatusRejected, reader.u8());
    TEST_ASSERT_EQUAL_UINT8(kFlagCatalogComplete, reader.u8());
    TEST_ASSERT_EQUAL_HEX16(kErrorNotFound, reader.u16());
    reader.u16();  // format_version
    reader.u16();  // reserved
    TEST_ASSERT_EQUAL_UINT32(0U, reader.u32());  // config_revision zeroed on error

    static const std::uint8_t kZeroUuid[16] = {0};
    std::uint8_t uuid[16]{};
    reader.bytes(uuid, sizeof(uuid));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(kZeroUuid, uuid, sizeof(uuid));

    TEST_ASSERT_EQUAL_UINT32(0U, reader.u32());  // described_source_id
    TEST_ASSERT_EQUAL_UINT32(0U, reader.u32());  // described_boot_id
    TEST_ASSERT_EQUAL_UINT8(0U, reader.u8());    // source_role
    TEST_ASSERT_EQUAL_UINT8(0U, reader.u8());    // source_flags
    TEST_ASSERT_EQUAL_UINT16(0U, reader.u16());  // catalog_index
    TEST_ASSERT_EQUAL_UINT16(1U, reader.u16());  // catalog_count: still 1, only the content is zeroed
    TEST_ASSERT_EQUAL_UINT16(0U, reader.u16());  // topic_count
    TEST_ASSERT_EQUAL_UINT16(0U, reader.u16());  // action_count
    TEST_ASSERT_EQUAL_STRING("unknown source", reader.utf8().c_str());
    TEST_ASSERT_EQUAL_UINT32(logical.size(), reader.pos());
}

// ---------------------------------------------------------------------------
// Targeting this robot's source_id but a boot_id it never had (or no longer
// has) means the caller's last-known session is gone -- STALE_TARGET_BOOT,
// not NOT_FOUND, so the caller knows to re-discover rather than retry.
// ---------------------------------------------------------------------------
void test_request_with_stale_boot_id_is_rejected() {
    BtpEndpoint endpoint;
    ManifestResponder responder;
    make_endpoint_and_responder(&endpoint, &responder);

    const auto request_payload =
        manifest_request_payload(0U, kLocalBootId + 1U, 0U);
    const auto header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                kRequestSequence);
    const auto logical = send_and_reassemble(endpoint, responder, header, request_payload);

    Reader reader(logical);
    reader.u32();
    reader.u32();
    reader.u32();
    TEST_ASSERT_EQUAL_UINT8(kStatusRejected, reader.u8());
    TEST_ASSERT_EQUAL_UINT8(kFlagCatalogComplete, reader.u8());
    TEST_ASSERT_EQUAL_HEX16(kErrorStaleTargetBoot, reader.u16());
    reader.u16();  // format_version
    reader.u16();  // reserved
    reader.u32();  // config_revision
    std::uint8_t uuid[16]{};
    reader.bytes(uuid, sizeof(uuid));
    reader.u32();
    reader.u32();
    reader.u8();
    reader.u8();
    reader.u16();
    reader.u16();
    reader.u16();
    reader.u16();
    TEST_ASSERT_EQUAL_STRING("boot mismatch", reader.utf8().c_str());
}

// ---------------------------------------------------------------------------
// A payload too short to even hold target_source_id/target_boot_id/
// known_revision (12 octets) is malformed, not a request the responder can
// answer with NOT_FOUND -- the caller (radio rx path) must count it as a
// drop, and nothing goes out over the endpoint.
// ---------------------------------------------------------------------------
void test_short_payload_is_rejected_without_sending() {
    BtpEndpoint endpoint;
    ManifestResponder responder;
    make_endpoint_and_responder(&endpoint, &responder);
    sent_count = 0U;

    const std::uint8_t short_payload[11] = {0};
    const btp::Header header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                        kRequestSequence);
    TEST_ASSERT_FALSE(responder.handle_request(
        header, {short_payload, sizeof(short_payload)}, 1U));
    TEST_ASSERT_EQUAL_UINT32(0U, sent_count);

    TEST_ASSERT_FALSE(responder.handle_request(header, {nullptr, 0U}, 1U));
    TEST_ASSERT_EQUAL_UINT32(0U, sent_count);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_full_manifest_response_matches_telemetry_schemas);
    RUN_TEST(test_targeted_request_matching_this_robot_succeeds);
    RUN_TEST(test_known_revision_returns_not_modified_with_no_topics);
    RUN_TEST(test_request_for_different_source_is_rejected_not_found);
    RUN_TEST(test_request_with_stale_boot_id_is_rejected);
    RUN_TEST(test_short_payload_is_rejected_without_sending);
    return UNITY_END();
}
