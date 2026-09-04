#include <unity.h>

#include <ManifestCatalog.h>
#include <TelemetryPublisher.h>
#include <btp/codec.hpp>
#include <btp/node.hpp>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// ManifestResponder is gone (BTP 2.39.0 / library 2.35.0's Catalog::
// write_source_info() + body-only topics let btp::Node answer CONTROL/
// MANIFEST_REQUEST entirely by itself now -- see BallyRobot.h's node_
// comment). This suite tests the replacement seam: ManifestCatalog::
// populate() (deliberately pure C++, "same shape ManifestResponder always
// was, so it links into env:native" -- see its own header) feeding a real
// btp::StaticNode, then MANIFEST_REQUEST -> node.receive() -> the wire bytes
// checked field-for-field against TelemetryPublisher::schemas(), the same
// promise the old suite held ManifestResponder to.
//
// Two behaviors changed on purpose migrating this onto btp::Node itself
// (not a ManifestCatalog choice) -- each has its own test below documenting
// the divergence from the old hand-rolled bally_OS behavior:
//   - a MANIFEST_REQUEST targeted at a different source_id is now silently
//     ignored (no reply at all) instead of answered REJECTED/NOT_FOUND.
//   - a REJECTED reply (STALE_TARGET_BOOT) now still carries this robot's
//     REAL uuid/described_source_id/described_boot_id/source_role/
//     config_revision, instead of an all-zeroed identity.

namespace {

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

// This suite's own btp::NodeConfig -- mirrors utils/BallyRobot/BallyRobot.h's
// real RobotLink, minus what this suite never exercises (open()/terminal()):
// send() through capture_send() above, seal() wired per test via a plain
// function pointer (has_seal() false, the default, means cleartext).
class TestLink : public btp::NodeConfig {
public:
    using SealFn = bool (*)(const btp::Header&, std::uint16_t,
                            const std::uint8_t*, std::uint8_t*);

    bool send(const std::uint8_t* frame, std::size_t frame_size) override {
        return capture_send(frame, frame_size);
    }

    bool has_seal() const noexcept override { return seal_fn != nullptr; }
    bool seal(const btp::Header& header, std::uint16_t payload_size,
              const std::uint8_t* plaintext, std::uint8_t* out) override {
        return seal_fn(header, payload_size, plaintext, out);
    }

    SealFn seal_fn = nullptr;
};

// Capacities mirror BallyRobot.h's real node_ (ManifestCatalog::
// kMaxCatalogFields / kMaxSourceInfoEntries are literally shared with it;
// Topics/StringBytes/Slots are repeated by value -- see that file's own
// node_ comment for the reasoning).
constexpr std::size_t kSlots = 4U;
constexpr std::size_t kSlotBytes = 600U;
constexpr std::size_t kScratchBytes = 640U;
constexpr std::size_t kCatalogTopics = 4U;
constexpr std::size_t kCatalogStringBytes = 256U;
constexpr std::uint64_t kReassemblyTimeoutMs = 4000U;

using TestNode = btp::StaticNode<
    kSlots, kSlotBytes,
    /*SealBytes=*/kScratchBytes, /*ScratchBytes=*/kScratchBytes,
    /*CatalogTopics=*/kCatalogTopics,
    /*CatalogFields=*/ManifestCatalog::kMaxCatalogFields,
    /*CatalogStringBytes=*/kCatalogStringBytes,
    /*MaxSubscriptions=*/1U, /*MaxCommands=*/1U, /*CommandBytes=*/16U,
    /*CatalogSourceInfo=*/ManifestCatalog::kMaxSourceInfoEntries>;

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

// Field type code MANIFEST_DATA puts on the wire (telemetry.md section 13):
// FieldSchema::type is btp::WireType now, whose octet value is the wire code
// itself. Kept as a named helper so a field record can be checked
// byte-for-byte, not just "some type was written".
std::uint8_t expected_wire_type_code(btp::WireType type) {
    return static_cast<std::uint8_t>(type);
}

// A request payload is exactly the 12-octet
// target_source_id/target_boot_id/known_config_revision triple (commands.md
// section 3.1); a shorter one fails to decode inside serve_manifest().
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
        .object_id = btp::object_id::kManifestRequest,
        .fragment_index = 0U,
        .fragment_count = 1U,
    };
}

// Feeds the request straight in as an already-decoded frame (BTP 2.35.0's
// Node::receive(const DecodedFrame&, ...) -- this suite owns the framing the
// same way a caller reassembling its own fragments does, so there is no
// encode/decode round trip on the REQUEST side to reason about, only on the
// RESPONSE side below). Returns the NodeRx outcome alongside the reassembled
// reply, since two tests below care about outcome-with-no-reply.
struct RequestResult {
    btp::NodeRx outcome;
    std::vector<std::uint8_t> logical;  // empty when nothing was sent
};

RequestResult send_request_and_reassemble(TestNode& node,
                                          const btp::Header& request_header,
                                          const std::vector<std::uint8_t>& request_payload) {
    sent_count = 0U;
    const btp::DecodedFrame decoded{
        request_header,
        btp::ByteView{request_payload.data(), request_payload.size()},
        /*crc32=*/0U,
    };
    btp::ReceivedMessage out{};
    const btp::NodeRx outcome = node.receive(decoded, /*now_ms=*/1000ULL, &out);

    RequestResult result{outcome, {}};
    if (sent_count == 0U) return result;

    std::uint8_t expected_count = 0U;
    for (std::size_t i = 0U; i < sent_count; ++i) {
        btp::DecodedFrame reply{};
        TEST_ASSERT_EQUAL_UINT8(
            static_cast<std::uint8_t>(btp::Error::Ok),
            static_cast<std::uint8_t>(btp::decode(
                sent_frames[i], sent_sizes[i], btp::kEspNowTransport, &reply)));
        TEST_ASSERT_EQUAL(static_cast<int>(btp::MessageType::Control),
                          static_cast<int>(reply.header.type));
        TEST_ASSERT_EQUAL_HEX16(btp::object_id::kManifestData, reply.header.object_id);
        TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(i),
                                reply.header.fragment_index);
        if (i == 0U) {
            expected_count = reply.header.fragment_count;
        } else {
            TEST_ASSERT_EQUAL_UINT8(expected_count, reply.header.fragment_count);
        }
        result.logical.insert(result.logical.end(), reply.payload.data,
                              reply.payload.data + reply.payload.size);
    }
    TEST_ASSERT_EQUAL_UINT32(expected_count, sent_count);
    return result;
}

// Bounds-checked cursor over the reassembled MANIFEST_DATA payload. Every
// accessor fails the current test (via TEST_ASSERT) rather than returning an
// error code, so a framing bug points straight at the field that broke
// instead of trailing off into "size mismatch" at the very end.
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

// Sets a link's identity/transport -- MUST run before the TestNode
// constructor, not after: btp::Receiver's constructor reads cfg.transport
// EAGERLY (unlike source_id/boot_id, which endpoint_.configure() only reads
// live at begin()-time), so a TestNode built from a not-yet-configured link
// would receiver_.valid() == false forever, same reason the real robot's
// bindProtocolTransport() sets protocol_link_.source_id/boot_id/transport
// before node_.emplace(), never after.
void configure_link(TestLink& link) {
    link.source_id = kLocalSourceId;
    link.boot_id = kLocalBootId;
    link.transport = btp::kEspNowTransport;
}

// Populates an already-begin()-able node's served catalogue straight from
// TelemetryPublisher::schemas() (ManifestCatalog::populate(), the exact call
// bindProtocolTransport()/populateProtocolCatalog() make on the real robot),
// with `source_info`/`source_info_count` (default none) forwarded verbatim,
// then begin()s and serve_catalog()s it. TEST_ASSERT's on any setup failure
// rather than returning one -- a fixture that cannot even construct has
// nothing left to test.
void make_node(TestNode& node,
              const ManifestCatalog::SourceInfoEntry* source_info = nullptr,
              std::size_t source_info_count = 0U) {
    std::size_t schema_count = 0U;
    const TelemetryPublisher::TopicSchema* schemas =
        TelemetryPublisher::schemas(&schema_count);
    TEST_ASSERT_TRUE(ManifestCatalog::populate(node.catalog(), schemas, schema_count,
                                               source_info, source_info_count));

    TEST_ASSERT_TRUE(node.begin());
    node.serve_catalog(ManifestCatalog::kSourceRoleRobot, kUuid, "bally_software");
}

// ---------------------------------------------------------------------------
// A wildcard request (target_source_id=0, target_boot_id=0) with an unknown
// revision (0) gets the full catalog back, and every topic/field record in
// it must match TelemetryPublisher::schemas() exactly -- including the
// fieldless kSystemMonitorTopicId (BTP 2.39.0's body-only topics), which the
// old ManifestResponder::catalog() could never hold but always put on the
// wire anyway (a divergence that would have made this exact assertion catch
// the topic vanishing, had it existed under the old architecture).
// ---------------------------------------------------------------------------
void test_full_manifest_response_matches_telemetry_schemas() {
    TestLink link;
    configure_link(link);
    TestNode node(link, kReassemblyTimeoutMs);
    make_node(node);

    const auto request_payload = manifest_request_payload(0U, 0U, 0U);
    const auto header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                kRequestSequence);
    const auto result = send_request_and_reassemble(node, header, request_payload);
    TEST_ASSERT_EQUAL(static_cast<int>(btp::NodeRx::RequestServed),
                      static_cast<int>(result.outcome));
    const auto& logical = result.logical;
    TEST_ASSERT_FALSE(logical.empty());

    Reader reader(logical);
    TEST_ASSERT_EQUAL_HEX32(kRequesterSourceId, reader.u32());
    TEST_ASSERT_EQUAL_HEX32(kRequesterBootId, reader.u32());
    TEST_ASSERT_EQUAL_UINT32(kRequestSequence, reader.u32());
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(btp::ResultStatus::Success), reader.u8());
    TEST_ASSERT_EQUAL_UINT8(btp::kManifestCatalogComplete, reader.u8());
    TEST_ASSERT_EQUAL_HEX16(static_cast<std::uint16_t>(btp::ResultError::None), reader.u16());
    TEST_ASSERT_EQUAL_UINT16(1U, reader.u16());  // format_version: no source_info configured
    TEST_ASSERT_EQUAL_UINT16(0U, reader.u16());  // reserved
    TEST_ASSERT_EQUAL_UINT32(ManifestCatalog::kConfigRevision, reader.u32());

    std::uint8_t uuid[16]{};
    reader.bytes(uuid, sizeof(uuid));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(kUuid, uuid, sizeof(uuid));

    TEST_ASSERT_EQUAL_HEX32(kLocalSourceId, reader.u32());  // described_source_id
    TEST_ASSERT_EQUAL_HEX32(kLocalBootId, reader.u32());    // described_boot_id
    TEST_ASSERT_EQUAL_UINT8(ManifestCatalog::kSourceRoleRobot, reader.u8());
    TEST_ASSERT_EQUAL_UINT8(btp::kSourceOnline, reader.u8());
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

    // source_info block: this fixture configures no entries -> format 1, no
    // block at all (unlike the old always-format-2 ManifestResponder).

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
// ManifestCatalog::populate() loads TelemetryPublisher::schemas() into
// node.catalog() -- MANIFEST_DATA is now read straight from that catalogue
// (Node::serve_catalog()/emit_manifest(), unlike the old ManifestResponder
// where the catalog was a parallel, unread validation copy), so this checks
// the catalogue field for field against the schema table, INCLUDING the
// fieldless kSystemMonitorTopicId -- BTP 2.39.0 is what makes that entry
// exist in the catalogue at all (the old btp::Catalog rejected field_count
// == 0 outright).
// ---------------------------------------------------------------------------
void test_catalog_matches_telemetry_schemas() {
    TestLink link;
    configure_link(link);
    TestNode node(link, kReassemblyTimeoutMs);
    make_node(node);

    std::size_t schema_count = 0U;
    const TelemetryPublisher::TopicSchema* schemas =
        TelemetryPublisher::schemas(&schema_count);
    TEST_ASSERT_GREATER_THAN_UINT32(0U, schema_count);

    for (std::size_t t = 0U; t < schema_count; ++t) {
        const TelemetryPublisher::TopicSchema& topic = schemas[t];
        const btp::CatalogTopic* cat_topic = node.catalog().topic(topic.topic_id);
        TEST_ASSERT_NOT_NULL(cat_topic);
        TEST_ASSERT_EQUAL_UINT16(topic.schema_version, cat_topic->schema_version);
        TEST_ASSERT_EQUAL_UINT32(topic.max_rate_millihz, cat_topic->max_rate_millihz);
        TEST_ASSERT_EQUAL_UINT32(topic.field_count, cat_topic->field_count);
        TEST_ASSERT_EQUAL_STRING(topic.name, cat_topic->name);

        for (std::size_t f = 0U; f < topic.field_count; ++f) {
            const TelemetryPublisher::FieldSchema& field = topic.fields[f];
            const btp::FieldSpec& spec = cat_topic->fields[f];
            TEST_ASSERT_EQUAL_UINT16(field.field_id, spec.field_id);
            TEST_ASSERT_EQUAL_UINT16(f, spec.order);
            TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(field.type), spec.type);
            TEST_ASSERT_EQUAL_UINT16(field.element_count, spec.element_count);
            TEST_ASSERT_EQUAL_STRING(field.name, node.catalog().field_name(*cat_topic, f));
            TEST_ASSERT_EQUAL_STRING(field.unit, node.catalog().field_unit(*cat_topic, f));
        }
    }
    TEST_ASSERT_EQUAL_UINT32(schema_count, node.catalog().topic_count());
}

// ---------------------------------------------------------------------------
// A request explicitly targeted at this robot's own (source_id, boot_id)
// takes the same success path as the wildcard -- targeting yourself by name
// is not a different case from targeting nobody in particular.
// ---------------------------------------------------------------------------
void test_targeted_request_matching_this_robot_succeeds() {
    TestLink link;
    configure_link(link);
    TestNode node(link, kReassemblyTimeoutMs);
    make_node(node);

    const auto request_payload =
        manifest_request_payload(kLocalSourceId, kLocalBootId, 0U);
    const auto header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                kRequestSequence);
    const auto result = send_request_and_reassemble(node, header, request_payload);
    TEST_ASSERT_FALSE(result.logical.empty());

    Reader reader(result.logical);
    reader.u32();
    reader.u32();
    reader.u32();
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(btp::ResultStatus::Success), reader.u8());
}

// ---------------------------------------------------------------------------
// A request for a revision the caller already has (NOT_MODIFIED) still
// succeeds, but sheds every topic record -- the caller already knows them.
//
// BEHAVIOR CHANGE from the old hand-rolled ManifestResponder: flags carries
// ONLY kManifestNotModified now, not kManifestNotModified |
// kManifestCatalogComplete -- btp::Node::serve_manifest() passes
// kManifestNotModified alone for this reply (library behavior, not a
// ManifestCatalog choice). CATALOG_COMPLETE describes whether catalog_index/
// catalog_count span the WHOLE catalogue; NOT_MODIFIED already tells the
// caller nothing changed since its last (complete) copy, so the omission
// does not lose information for a caller that already implements both.
// ---------------------------------------------------------------------------
void test_known_revision_returns_not_modified_with_no_topics() {
    TestLink link;
    configure_link(link);
    TestNode node(link, kReassemblyTimeoutMs);
    make_node(node);

    const auto request_payload =
        manifest_request_payload(0U, 0U, ManifestCatalog::kConfigRevision);
    const auto header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                kRequestSequence);
    const auto result = send_request_and_reassemble(node, header, request_payload);
    const auto& logical = result.logical;
    TEST_ASSERT_FALSE(logical.empty());

    Reader reader(logical);
    reader.u32();
    reader.u32();
    reader.u32();
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(btp::ResultStatus::Success), reader.u8());
    TEST_ASSERT_EQUAL_UINT8(btp::kManifestNotModified, reader.u8());
    TEST_ASSERT_EQUAL_HEX16(static_cast<std::uint16_t>(btp::ResultError::None), reader.u16());
    TEST_ASSERT_EQUAL_UINT16(1U, reader.u16());  // format_version: no source_info configured
    reader.u16();  // reserved
    TEST_ASSERT_EQUAL_UINT32(ManifestCatalog::kConfigRevision, reader.u32());
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
// BEHAVIOR CHANGE from the old hand-rolled ManifestResponder: a request
// naming a different source_id used to be answered REJECTED/NOT_FOUND with
// an all-zeroed identity ("a leaf node only knows how to describe itself...
// answered with an all-zeroed identity so a misrouted request can never be
// mistaken for a real description"). btp::Node::serve_manifest() instead
// ignores it outright -- ANY reply (even a REJECTED one) would itself be
// this robot describing itself in response to a request that was not
// addressed to it, which the library's own design treats as not this
// robot's business to answer. Accepted as-is (bally_OS's dongle is this
// robot's only ESP-NOW peer and never sends a targeted request for anyone
// else): see PLANO_ESPIDF_DONGLE / the btp-node-consumer-migration memory
// for the decision.
// ---------------------------------------------------------------------------
void test_request_for_different_source_is_ignored() {
    TestLink link;
    configure_link(link);
    TestNode node(link, kReassemblyTimeoutMs);
    make_node(node);

    const auto request_payload = manifest_request_payload(0xDEADBEEFU, 0U, 0U);
    const auto header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                kRequestSequence);
    const auto result = send_request_and_reassemble(node, header, request_payload);
    TEST_ASSERT_EQUAL(static_cast<int>(btp::NodeRx::RequestServed),
                      static_cast<int>(result.outcome));
    TEST_ASSERT_TRUE(result.logical.empty());
}

// ---------------------------------------------------------------------------
// Targeting this robot's source_id but a boot_id it never had (or no longer
// has) means the caller's last-known session is gone -- STALE_TARGET_BOOT,
// not NOT_FOUND, so the caller knows to re-discover rather than retry. This
// path IS still answered (unlike the different-source_id case above): a
// stale boot_id still targets THIS robot.
//
// BEHAVIOR CHANGE from the old hand-rolled ManifestResponder: the identity
// fields (uuid/described_source_id/described_boot_id/source_role/
// config_revision) are the REAL ones here, not zeroed -- btp::Node's
// emit_manifest() always describes who is actually answering, even on a
// REJECTED reply; only topic_count/source_info are withheld.
// ---------------------------------------------------------------------------
void test_request_with_stale_boot_id_is_rejected() {
    TestLink link;
    configure_link(link);
    TestNode node(link, kReassemblyTimeoutMs);
    make_node(node);

    const auto request_payload =
        manifest_request_payload(0U, kLocalBootId + 1U, 0U);
    const auto header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                kRequestSequence);
    const auto result = send_request_and_reassemble(node, header, request_payload);
    const auto& logical = result.logical;
    TEST_ASSERT_FALSE(logical.empty());

    Reader reader(logical);
    reader.u32();
    reader.u32();
    reader.u32();
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(btp::ResultStatus::Rejected), reader.u8());
    TEST_ASSERT_EQUAL_UINT8(0U, reader.u8());  // flags: neither NOT_MODIFIED nor CATALOG_COMPLETE
    TEST_ASSERT_EQUAL_HEX16(static_cast<std::uint16_t>(btp::ResultError::StaleTargetBoot), reader.u16());
    TEST_ASSERT_EQUAL_UINT16(1U, reader.u16());  // format_version: REJECTED always stays format 1
    reader.u16();  // reserved
    TEST_ASSERT_EQUAL_UINT32(ManifestCatalog::kConfigRevision, reader.u32());  // real, not zeroed

    std::uint8_t uuid[16]{};
    reader.bytes(uuid, sizeof(uuid));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(kUuid, uuid, sizeof(uuid));  // real, not zeroed

    TEST_ASSERT_EQUAL_HEX32(kLocalSourceId, reader.u32());  // described_source_id: real
    TEST_ASSERT_EQUAL_HEX32(kLocalBootId, reader.u32());    // described_boot_id: real
    TEST_ASSERT_EQUAL_UINT8(ManifestCatalog::kSourceRoleRobot, reader.u8());  // source_role: real
    TEST_ASSERT_EQUAL_UINT8(btp::kSourceOnline, reader.u8());
    TEST_ASSERT_EQUAL_UINT16(0U, reader.u16());  // catalog_index
    TEST_ASSERT_EQUAL_UINT16(1U, reader.u16());  // catalog_count
    TEST_ASSERT_EQUAL_UINT16(0U, reader.u16());  // topic_count: withheld
    TEST_ASSERT_EQUAL_UINT16(0U, reader.u16());  // action_count
    TEST_ASSERT_EQUAL_STRING("bally_software", reader.utf8().c_str());  // real name, not "boot mismatch"
    TEST_ASSERT_EQUAL_UINT32(logical.size(), reader.pos());
}

// ---------------------------------------------------------------------------
// A malformed/too-short payload (shorter than the 12-octet target_source_id/
// target_boot_id/known_revision triple) never reaches decode_manifest_
// request() successfully, so serve_manifest() returns without sending --
// same "malformed, not answerable" outcome the old handle_request() had,
// just now indistinguishable at the NodeRx level from the different-source_id
// ignore case above (both are RequestServed with zero frames sent; see this
// suite's own top comment).
// ---------------------------------------------------------------------------
void test_short_payload_is_ignored_without_sending() {
    TestLink link;
    configure_link(link);
    TestNode node(link, kReassemblyTimeoutMs);
    make_node(node);

    const std::vector<std::uint8_t> short_payload(11U, 0U);
    const auto header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                kRequestSequence);
    const auto result = send_request_and_reassemble(node, header, short_payload);
    TEST_ASSERT_EQUAL(static_cast<int>(btp::NodeRx::RequestServed),
                      static_cast<int>(result.outcome));
    TEST_ASSERT_TRUE(result.logical.empty());
}

// ---------------------------------------------------------------------------
// Topico 31.3: MANIFEST_DATA rides channel C like every other CONTROL reply
// this robot originates toward the dongle, so it must seal with whichever
// function the link is given, exactly like StatusReporter's STATUS already
// does. Without this, bally_dongle's own reply to its own manifest-priming
// request never authenticates under key L (see bally_channels.h's
// dongle_consumes comment on the topico 31.3 gap this closes).
// ---------------------------------------------------------------------------
bool fake_seal(const btp::Header&, std::uint16_t payload_size,
              const std::uint8_t* plaintext, std::uint8_t* out) {
    std::memcpy(out, plaintext, payload_size);
    std::memset(out + payload_size, 0x11, btp::kEndpointAeadTagSize);
    return true;
}

void test_reply_is_sealed_when_link_has_a_seal_function() {
    TestLink link;
    configure_link(link);
    link.seal_fn = &fake_seal;
    TestNode node(link, kReassemblyTimeoutMs);
    make_node(node);

    // NOT_MODIFIED (known revision) keeps this to one fragment, so checking
    // the AEAD tag at the end of the sealed logical payload is simple to
    // reason about.
    const auto request_payload =
        manifest_request_payload(0U, 0U, ManifestCatalog::kConfigRevision);
    const auto header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                kRequestSequence);
    sent_count = 0U;
    const btp::DecodedFrame decoded{
        header, btp::ByteView{request_payload.data(), request_payload.size()}, 0U};
    btp::ReceivedMessage out{};
    node.receive(decoded, 1000ULL, &out);
    TEST_ASSERT_EQUAL_UINT32(1U, sent_count);

    btp::DecodedFrame reply{};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp::Error::Ok),
        static_cast<std::uint8_t>(btp::decode(
            sent_frames[0], sent_sizes[0], btp::kEspNowTransport, &reply)));
    TEST_ASSERT_TRUE((reply.header.flags & btp::kFlagEncrypted) != 0U);
    // 0x11 is fake_seal's tag marker -- proves the reply actually routed
    // through the configured seal function instead of going out in the clear.
    TEST_ASSERT_EQUAL_HEX8(0x11U, reply.payload.data[reply.payload.size - 1U]);
}

// ---------------------------------------------------------------------------
// A configured source_info block (commands.md 3.12) round-trips: format
// version bumps to 2, then info_count, then key/label/value per entry, in
// order; an entry with an empty value is dropped by ManifestCatalog::
// populate() (btp::Catalog::add_source_info() -- see its own comment). It
// rides a full response AND a NOT_MODIFIED one, since source_info is not
// gated by config_revision.
// ---------------------------------------------------------------------------
void skip_prefix_and_name_to_source_info(Reader& reader, std::uint16_t* topic_count_out) {
    for (int i = 0; i < 3; ++i) reader.u32();  // request reference
    reader.u8();                               // status
    reader.u8();                               // flags
    reader.u16();                              // error_code
    TEST_ASSERT_EQUAL_UINT16(2U, reader.u16());  // manifest_format_version: source_info present
    reader.u16();                              // reserved
    reader.u32();                              // config_revision
    std::uint8_t uuid[16]{};
    reader.bytes(uuid, sizeof(uuid));
    reader.u32();                              // described_source_id
    reader.u32();                              // described_boot_id
    reader.u8();                               // source_role
    reader.u8();                               // source_flags
    reader.u16();                              // catalog_index
    reader.u16();                              // catalog_count
    *topic_count_out = reader.u16();           // topic_count
    reader.u16();                              // action_count
    TEST_ASSERT_EQUAL_STRING("bally_software", reader.utf8().c_str());
}

void assert_two_entry_info_block(Reader& reader) {
    TEST_ASSERT_EQUAL_UINT16(2U, reader.u16());  // info_count: the empty entry was dropped
    TEST_ASSERT_EQUAL_STRING("fw_version", reader.utf8().c_str());
    TEST_ASSERT_EQUAL_STRING("Firmware", reader.utf8().c_str());
    TEST_ASSERT_EQUAL_STRING("1dd9fc5", reader.utf8().c_str());
    TEST_ASSERT_EQUAL_STRING("chip", reader.utf8().c_str());
    TEST_ASSERT_EQUAL_STRING("Chip", reader.utf8().c_str());
    TEST_ASSERT_EQUAL_STRING("ESP32-S3", reader.utf8().c_str());
}

void test_source_info_block_round_trips_and_skips_empty_values() {
    TestLink link;
    configure_link(link);
    TestNode node(link, kReassemblyTimeoutMs);

    const ManifestCatalog::SourceInfoEntry info[] = {
        {"fw_version", "Firmware", "1dd9fc5"},
        {"name", "Name", ""},  // empty value -> not emitted
        {"chip", "Chip", "ESP32-S3"},
    };
    make_node(node, info, sizeof(info) / sizeof(info[0]));

    std::size_t expected_topic_count = 0U;
    TelemetryPublisher::schemas(&expected_topic_count);

    const auto header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                kRequestSequence);

    // Full response: source_info sits between source_name and the topic records.
    {
        const auto result = send_request_and_reassemble(
            node, header, manifest_request_payload(0U, 0U, 0U));
        Reader reader(result.logical);
        std::uint16_t topic_count = 0U;
        skip_prefix_and_name_to_source_info(reader, &topic_count);
        TEST_ASSERT_EQUAL_UINT32(expected_topic_count, topic_count);
        assert_two_entry_info_block(reader);
        TEST_ASSERT_LESS_THAN_UINT32(result.logical.size(), reader.pos());  // topic records still follow
    }

    // NOT_MODIFIED response: no topic records, but the same source_info block.
    {
        const auto result = send_request_and_reassemble(
            node, header,
            manifest_request_payload(0U, 0U, ManifestCatalog::kConfigRevision));
        Reader reader(result.logical);
        std::uint16_t topic_count = 0U;
        skip_prefix_and_name_to_source_info(reader, &topic_count);
        TEST_ASSERT_EQUAL_UINT16(0U, topic_count);
        assert_two_entry_info_block(reader);
        TEST_ASSERT_EQUAL_UINT32(result.logical.size(), reader.pos());  // nothing after
    }
}

// ---------------------------------------------------------------------------
// A device with more source_info than fits (a long name + description on top
// of the build/chip fields) does not fail the whole response: btp::Catalog::
// write_source_info() stops before crowding out the topic records the
// consumer actually needs to decode telemetry. info_count reflects what was
// actually written, and the records after it still parse.
// ---------------------------------------------------------------------------
void test_source_info_truncates_before_crowding_out_the_topic_records() {
    TestLink link;
    configure_link(link);
    TestNode node(link, kReassemblyTimeoutMs);

    // kMaxSourceInfoEntries entries of ~50 octets each, well past the scratch
    // budget (kScratchBytes) once the topic records are added too --
    // ManifestCatalog::populate() itself is expected to report this as a
    // partial failure (some source_info entries do not fit the catalog's
    // own string pool), so this deliberately does NOT go through make_node()
    // (which TEST_ASSERT_TRUE's on populate() succeeding): begin()/
    // serve_catalog() still run, since the topics themselves loaded fine.
    static const char* kLong = "0123456789012345678901234567890123456789";  // 40
    ManifestCatalog::SourceInfoEntry info[ManifestCatalog::kMaxSourceInfoEntries];
    for (auto& entry : info) {
        entry = ManifestCatalog::SourceInfoEntry{"k", "label", kLong};
    }
    std::size_t schema_count = 0U;
    const TelemetryPublisher::TopicSchema* schemas =
        TelemetryPublisher::schemas(&schema_count);
    ManifestCatalog::populate(node.catalog(), schemas, schema_count, info,
                              sizeof(info) / sizeof(info[0]));
    TEST_ASSERT_TRUE(node.begin());
    node.serve_catalog(ManifestCatalog::kSourceRoleRobot, kUuid, "bally_software");

    const auto header = manifest_request_header(kRequesterSourceId, kRequesterBootId,
                                                kRequestSequence);
    const auto result = send_request_and_reassemble(
        node, header, manifest_request_payload(0U, 0U, 0U));
    const auto& logical = result.logical;
    TEST_ASSERT_FALSE(logical.empty());
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(kScratchBytes, logical.size());

    Reader reader(logical);
    std::uint16_t topic_count = 0U;
    skip_prefix_and_name_to_source_info(reader, &topic_count);
    TEST_ASSERT_EQUAL_UINT32(schema_count, topic_count);

    const std::uint16_t info_count = reader.u16();
    TEST_ASSERT_GREATER_THAN_UINT16(0U, info_count);   // some fit
    TEST_ASSERT_LESS_THAN_UINT16(ManifestCatalog::kMaxSourceInfoEntries, info_count);  // not all
    for (std::uint16_t i = 0U; i < info_count; ++i) {
        reader.utf8();  // key
        reader.utf8();  // label
        TEST_ASSERT_EQUAL_STRING(kLong, reader.utf8().c_str());
    }

    // The topic records after the (truncated) block still frame cleanly: each
    // record_size prefix lands within bounds and consumption is exact.
    for (std::size_t t = 0U; t < schema_count; ++t) {
        const std::uint32_t record_size = reader.u32();
        for (std::uint32_t b = 0U; b < record_size; ++b) {
            reader.u8();
        }
    }
    TEST_ASSERT_EQUAL_UINT32(logical.size(), reader.pos());
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_full_manifest_response_matches_telemetry_schemas);
    RUN_TEST(test_catalog_matches_telemetry_schemas);
    RUN_TEST(test_targeted_request_matching_this_robot_succeeds);
    RUN_TEST(test_known_revision_returns_not_modified_with_no_topics);
    RUN_TEST(test_request_for_different_source_is_ignored);
    RUN_TEST(test_request_with_stale_boot_id_is_rejected);
    RUN_TEST(test_short_payload_is_ignored_without_sending);
    RUN_TEST(test_source_info_block_round_trips_and_skips_empty_values);
    RUN_TEST(test_source_info_truncates_before_crowding_out_the_topic_records);
    RUN_TEST(test_reply_is_sealed_when_link_has_a_seal_function);
    return UNITY_END();
}
