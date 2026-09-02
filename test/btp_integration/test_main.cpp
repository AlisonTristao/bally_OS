#include <unity.h>

#include <BtpTransport.h>
#include <CommandProcessor.h>
#include <StatusReporter.h>
#include <SubscriptionResponder.h>
#include <TelemetryPublisher.h>
#include <TxScheduler.h>
#include <bally_channels.h>
#include <btp/codec.hpp>
#include <btp/fragmentation.hpp>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

std::vector<std::uint8_t> read_vector(const char* relative_path) {
    const std::string candidates[] = {
        std::string("../BTP/test-vectors/v1/") + relative_path,
        std::string("../../BTP/test-vectors/v1/") + relative_path,
    };
    for (const auto& path : candidates) {
        std::ifstream input(path, std::ios::binary);
        if (input) {
            return {std::istreambuf_iterator<char>(input),
                    std::istreambuf_iterator<char>()};
        }
    }
    return {};
}

// Deep enough for the largest logical message this firmware sends: the
// system.monitor UTF-8 document sealed and split into ESP-NOW fragments.
constexpr std::size_t kMaxCapturedFrames = 16U;
std::uint8_t sent_frames[kMaxCapturedFrames][btp::kEspNowMaxFrameSize]{};
std::size_t sent_sizes[kMaxCapturedFrames]{};
std::size_t sent_count = 0U;

bool capture_send(const std::uint8_t* data, std::size_t size) {
    if (sent_count >= kMaxCapturedFrames || size > btp::kEspNowMaxFrameSize)
        return false;
    std::memcpy(sent_frames[sent_count], data, size);
    sent_sizes[sent_count] = size;
    ++sent_count;
    return true;
}

float float_from_bits(std::uint32_t bits) {
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

void write_u16(std::uint8_t* output, std::uint16_t value) {
    output[0] = static_cast<std::uint8_t>(value);
    output[1] = static_cast<std::uint8_t>(value >> 8U);
}

void write_u32(std::uint8_t* output, std::uint32_t value) {
    output[0] = static_cast<std::uint8_t>(value);
    output[1] = static_cast<std::uint8_t>(value >> 8U);
    output[2] = static_cast<std::uint8_t>(value >> 16U);
    output[3] = static_cast<std::uint8_t>(value >> 24U);
}

std::uint16_t read_u16(const std::uint8_t* data);
btp::DecodedFrame decode_only_frame();

std::vector<std::uint8_t> shell_request_payload(std::uint32_t target_source,
                                                std::uint32_t target_boot,
                                                const char* command) {
    const std::size_t command_size = std::strlen(command);
    std::vector<std::uint8_t> payload(btp_command::kRequestPrefixSize +
                                      command_size);
    write_u32(payload.data(), target_source);
    write_u32(payload.data() + 4U, target_boot);
    write_u16(payload.data() + 8U, btp_command::kShellActionId);
    write_u16(payload.data() + 10U, btp_command::kShellActionVersion);
    write_u16(payload.data() + 12U, 0U);
    write_u16(payload.data() + 14U, 0U);
    write_u32(payload.data() + 16U, static_cast<std::uint32_t>(command_size));
    std::memcpy(payload.data() + btp_command::kRequestPrefixSize, command,
                command_size);
    return payload;
}

btp::Header command_header(std::uint32_t sequence) {
    return {
        .type = btp::MessageType::Command,
        .flags = 0U,
        .source_id = 0x0C30AA5CU,
        .boot_id = 0x10203040U,
        .sequence = sequence,
        .timestamp_us = 1000U,
        .object_id = btp_command::kCommandRequestObjectId,
        .fragment_index = 0U,
        .fragment_count = 1U,
    };
}

bool capture_radio(void*, const std::uint8_t* data, std::size_t size) {
    return capture_send(data, size);
}

// Fake seal functions distinguishable by their tag byte, standing in for
// RadioSeal::seal (key L) and RadioSeal::seal_e (key E): neither actually
// runs AEAD, they only prove WHICH function CommandProcessor::send_result
// picked for a given ResultView::channel. Real cryptographic sealing is
// RadioSeal's own concern and out of reach for env:native (see RadioSeal.h's
// class comment on why it is never linked here).
bool fake_seal_link(void*, const btp::Header&, std::uint16_t payload_size,
                    const std::uint8_t* plaintext, std::uint8_t* out) {
    std::memcpy(out, plaintext, payload_size);
    std::memset(out + payload_size, 0x11, BtpEndpoint::kAeadTagSize);
    return true;
}

bool fake_seal_endpoint(void*, const btp::Header&, std::uint16_t payload_size,
                        const std::uint8_t* plaintext, std::uint8_t* out) {
    std::memcpy(out, plaintext, payload_size);
    std::memset(out + payload_size, 0x22, BtpEndpoint::kAeadTagSize);
    return true;
}

// Stands in for "the key this channel needs is not loaded" -- RadioSeal::seal /
// seal_e both return false in that state, and every send path must then drop
// the frame rather than transmit it in the clear.
bool always_failing_seal(void*, const btp::Header&, std::uint16_t,
                         const std::uint8_t*, std::uint8_t*) {
    return false;
}

void test_canonical_command_request_is_fully_parsed() {
    const auto bytes = read_vector("valid/command_request.bin");
    TEST_ASSERT_FALSE(bytes.empty());

    btp::DecodedFrame decoded{};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp::Error::Ok),
        static_cast<std::uint8_t>(btp::decode(
            bytes.data(), bytes.size(), btp::TransportProfile::Serial,
            &decoded)));

    btp_command::RequestView request{};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp_command::ParseError::Ok),
        static_cast<std::uint8_t>(btp_command::parse_request(
            decoded.header, decoded.payload, 0x11223344U, 0x55667788U,
            &request)));
    TEST_ASSERT_EQUAL_HEX16(0x0201U, request.action_id);
    TEST_ASSERT_EQUAL_UINT16(1U, request.action_version);
    TEST_ASSERT_EQUAL_UINT32(6U, request.parameters.size);
}

void test_canonical_crc_failure_is_rejected() {
    const auto bytes = read_vector("invalid/crc.bin");
    TEST_ASSERT_FALSE(bytes.empty());

    btp::DecodedFrame decoded{};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp::Error::CrcMismatch),
        static_cast<std::uint8_t>(btp::decode(
            bytes.data(), bytes.size(), btp::TransportProfile::EspNow,
            &decoded)));
}

void test_telemetry_can_never_become_a_command() {
    const auto bytes = read_vector("valid/telemetry_packed_le.bin");
    TEST_ASSERT_FALSE(bytes.empty());

    btp::DecodedFrame decoded{};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp::Error::Ok),
        static_cast<std::uint8_t>(btp::decode(
            bytes.data(), bytes.size(), btp::TransportProfile::EspNow,
            &decoded)));

    btp_command::RequestView request{};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp_command::ParseError::WrongType),
        static_cast<std::uint8_t>(btp_command::parse_request(
            decoded.header, decoded.payload, decoded.header.source_id,
            decoded.header.boot_id, &request)));
}

void test_only_supported_single_shell_command_is_copied() {
    const std::uint8_t command[] = "logger psram_usage";
    const btp_command::RequestView request{
        .target_source_id = 1U,
        .target_boot_id = 2U,
        .action_id = btp_command::kShellActionId,
        .action_version = btp_command::kShellActionVersion,
        .parameters = {command, sizeof(command) - 1U},
    };
    char output[btp_command::kMaxShellCommandSize + 1U]{};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp_command::ParseError::Ok),
        static_cast<std::uint8_t>(btp_command::copy_shell_command(
            request, output, sizeof(output))));
    TEST_ASSERT_EQUAL_STRING("logger psram_usage", output);

    const std::uint8_t batch[] = "logger psram_usage\nrobot btn 1";
    auto invalid = request;
    invalid.parameters = {batch, sizeof(batch) - 1U};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp_command::ParseError::InvalidShellText),
        static_cast<std::uint8_t>(btp_command::copy_shell_command(
            invalid, output, sizeof(output))));
}

void test_endpoint_fragments_with_one_shared_sequence_and_exact_sizes() {
    sent_count = 0U;
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(0x11223344U, 0xA1B2C3D4U));
    endpoint.set_send_callback(capture_send);

    std::uint8_t payload[btp::kEspNowMaxPayloadSize + 1U]{};
    for (std::size_t index = 0U; index < sizeof(payload); ++index) {
        payload[index] = static_cast<std::uint8_t>(index);
    }
    TEST_ASSERT_TRUE(endpoint.send_logical(
        btp::MessageType::Log, 2U, payload, sizeof(payload), 1000000U));
    TEST_ASSERT_EQUAL_UINT32(2U, sent_count);
    TEST_ASSERT_EQUAL_UINT32(btp::kEspNowMaxFrameSize, sent_sizes[0]);
    TEST_ASSERT_EQUAL_UINT32(btp::kV1MinimumFrameSize + 1U, sent_sizes[1]);

    btp::DecodedFrame first{};
    btp::DecodedFrame second{};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp::Error::Ok),
        static_cast<std::uint8_t>(btp::decode(
            sent_frames[0], sent_sizes[0], btp::TransportProfile::EspNow,
            &first)));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp::Error::Ok),
        static_cast<std::uint8_t>(btp::decode(
            sent_frames[1], sent_sizes[1], btp::TransportProfile::EspNow,
            &second)));
    TEST_ASSERT_EQUAL_UINT32(first.header.sequence, second.header.sequence);
    TEST_ASSERT_EQUAL_UINT8(0U, first.header.fragment_index);
    TEST_ASSERT_EQUAL_UINT8(1U, second.header.fragment_index);
    TEST_ASSERT_EQUAL_UINT8(2U, first.header.fragment_count);
}

// Renamed from test_authorization_binds_mac_to_claimed_source: topico 28
// removed that binding, so asserting it would assert a rule the firmware no
// longer has. What is left is a radio filter, and the last assertion is the
// point -- a frame whose source_id is somebody else's is now ACCEPTED at this
// stage, which is what makes the hub work; authentication is the AEAD tag
// opened after reassembly in ROBOT::handleReceiveStatic, not this memcmp.
void test_peer_mac_filter_accepts_relayed_source_ids() {
    const std::uint8_t expected[6] = {0xDC, 0xDA, 0x0C, 0x30, 0xAA, 0x5C};
    const std::uint8_t attacker[6] = {0xDC, 0xDA, 0x0C, 0x30, 0xAA, 0x5D};
    TEST_ASSERT_EQUAL_HEX32(0x0C30AA5CU,
                            btp_command::source_id_from_mac(expected));
    TEST_ASSERT_TRUE(btp_command::authorized_source(expected, expected));
    TEST_ASSERT_FALSE(btp_command::authorized_source(expected, attacker));
    TEST_ASSERT_FALSE(btp_command::authorized_source(nullptr, expected));
    TEST_ASSERT_FALSE(btp_command::authorized_source(expected, nullptr));
}

void test_protocol_test_matches_canonical_vector_and_origin_timestamp() {
    const auto canonical = read_vector("valid/protocol_test.bin");
    TEST_ASSERT_FALSE(canonical.empty());

    sent_count = 0U;
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(0x11223344U, 0xA1B2C3D4U));
    endpoint.set_send_callback(capture_send);

    TelemetryPublisher publisher;
    publisher.configure(endpoint);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::Queued),
        static_cast<std::uint8_t>(publisher.publish_protocol_test(
            0x01020304U, float_from_bits(0x3F0D0A00U), 1000000U)));
    TEST_ASSERT_EQUAL_UINT32(1U, publisher.queued_count());

    TEST_ASSERT_EQUAL_UINT32(1U, publisher.flush(1U));
    TEST_ASSERT_EQUAL_UINT32(1U, sent_count);
    TEST_ASSERT_EQUAL_UINT32(canonical.size(), sent_sizes[0]);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(canonical.data(), sent_frames[0],
                                  canonical.size());

    btp::DecodedFrame decoded{};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp::Error::Ok),
        static_cast<std::uint8_t>(btp::decode(
            sent_frames[0], sent_sizes[0], btp::TransportProfile::EspNow,
            &decoded)));
    TEST_ASSERT_EQUAL_UINT64(1000000U, decoded.header.timestamp_us);
    TEST_ASSERT_EQUAL_HEX8(0x00U, decoded.payload.data[6]);
    TEST_ASSERT_EQUAL_HEX8(0x0AU, decoded.payload.data[7]);
    TEST_ASSERT_EQUAL_HEX8(0x0DU, decoded.payload.data[8]);
}

void test_publisher_queue_is_bounded_and_drop_newest_is_counted() {
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(1U, 2U));

    TelemetryPublisher publisher;
    publisher.configure(endpoint);
    for (std::size_t index = 0U;
         index < TelemetryPublisher::kQueueCapacity; ++index) {
        TEST_ASSERT_EQUAL_UINT8(
            static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::Queued),
            static_cast<std::uint8_t>(publisher.publish_robot_state(
                static_cast<std::uint8_t>(index), index)));
    }

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::QueueFull),
        static_cast<std::uint8_t>(publisher.publish_robot_state(99U, 99U)));
    TEST_ASSERT_EQUAL_UINT32(TelemetryPublisher::kQueueCapacity,
                             publisher.queued_count());
    const auto stats = publisher.stats();
    TEST_ASSERT_EQUAL_UINT32(TelemetryPublisher::kQueueCapacity, stats.queued);
    TEST_ASSERT_EQUAL_UINT32(1U, stats.dropped_full);
}

void test_publisher_registers_static_schemas_and_rejects_nan() {
    std::size_t schema_count = 0U;
    const auto* schemas = TelemetryPublisher::schemas(&schema_count);
    TEST_ASSERT_NOT_NULL(schemas);
    TEST_ASSERT_EQUAL_UINT32(3U, schema_count);
    TEST_ASSERT_EQUAL_STRING("protocol.test", schemas[0].name);
    TEST_ASSERT_EQUAL_HEX16(TelemetryPublisher::kProtocolTestTopicId,
                            schemas[0].topic_id);
    TEST_ASSERT_EQUAL_STRING("robot.state", schemas[1].name);
    TEST_ASSERT_EQUAL_STRING("system.monitor", schemas[2].name);
    TEST_ASSERT_EQUAL_HEX16(TelemetryPublisher::kSystemMonitorTopicId,
                            schemas[2].topic_id);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::Encoding::Utf8),
        static_cast<std::uint8_t>(schemas[2].encoding));
    TEST_ASSERT_EQUAL_UINT32(0U, schemas[2].field_count);
    TEST_ASSERT_EQUAL_UINT32(1000U, schemas[2].max_rate_millihz);
    TEST_ASSERT_EQUAL_UINT32(333U, schemas[2].default_rate_millihz);

    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(1U, 2U));
    TelemetryPublisher publisher;
    publisher.configure(endpoint);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::InvalidValue),
        static_cast<std::uint8_t>(publisher.publish_protocol_test(
            1U, float_from_bits(0x7FC00000U), 10U)));
    TEST_ASSERT_EQUAL_UINT32(1U, publisher.stats().dropped_invalid);
}

void test_system_monitor_publishes_complete_utf8_document() {
    sent_count = 0U;
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(0x11223344U, 0xA1B2C3D4U));
    endpoint.set_send_callback(capture_send);

    TelemetryPublisher publisher;
    publisher.configure(endpoint);
    const char report[] = "TASK            C  CPU%  STKk\nEKF_task        P  12.3   1.8\n";
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::Queued),
        static_cast<std::uint8_t>(publisher.publish_system_monitor(
            report, std::strlen(report), 3000000U)));
    TEST_ASSERT_EQUAL_UINT32(1U, publisher.flush(1U));

    const btp::DecodedFrame decoded = decode_only_frame();
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(btp::MessageType::Telemetry),
                            static_cast<std::uint8_t>(decoded.header.type));
    TEST_ASSERT_EQUAL_HEX16(TelemetryPublisher::kSystemMonitorTopicId,
                            decoded.header.object_id);
    TEST_ASSERT_EQUAL_UINT64(3000000U, decoded.header.timestamp_us);
    TEST_ASSERT_EQUAL_UINT16(TelemetryPublisher::kSchemaVersion,
                             read_u16(decoded.payload.data));
    TEST_ASSERT_EQUAL_UINT32(std::strlen(report) + 2U, decoded.payload.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(
        reinterpret_cast<const std::uint8_t*>(report), decoded.payload.data + 2U,
        std::strlen(report));

    std::string too_large(TelemetryPublisher::kMaxSystemMonitorTextSize + 1U, 'x');
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::InvalidValue),
        static_cast<std::uint8_t>(publisher.publish_system_monitor(
            too_large.data(), too_large.size(), 4000000U)));
}

void test_large_sealed_monitor_fragments_and_reassembles() {
    sent_count = 0U;
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(0x11223344U, 0xA1B2C3D4U));
    endpoint.set_send_callback(capture_send);

    TelemetryPublisher publisher;
    publisher.configure(endpoint, fake_seal_endpoint, nullptr);

    std::string report(TelemetryPublisher::kMaxSystemMonitorTextSize, 'x');
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::Queued),
        static_cast<std::uint8_t>(publisher.publish_system_monitor(
            report.data(), report.size(), 7000000U)));

    // Replace-in-place staging: a second document before flush() takes the
    // first is dropped, not queued behind it.
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::QueueFull),
        static_cast<std::uint8_t>(publisher.publish_system_monitor(
            report.data(), report.size(), 7003000U)));

    std::uint8_t expected_fragments = 0U;
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp::Error::Ok),
        static_cast<std::uint8_t>(btp::fragment_count(
            TelemetryPublisher::kMaxSystemMonitorPayloadSize +
                BtpEndpoint::kAeadTagSize,
            btp::TransportProfile::EspNow, &expected_fragments)));
    TEST_ASSERT_TRUE(expected_fragments > 1U);
    TEST_ASSERT_TRUE(expected_fragments <= kMaxCapturedFrames);

    // One logical sample, however many wire fragments it needs.
    TEST_ASSERT_EQUAL_UINT32(1U, publisher.flush(4U));
    TEST_ASSERT_EQUAL_UINT32(expected_fragments, sent_count);

    btp::DecodedFrame decoded[kMaxCapturedFrames]{};
    std::size_t reassembled = 0U;
    for (std::size_t i = 0U; i < sent_count; ++i) {
        TEST_ASSERT_EQUAL_UINT8(
            static_cast<std::uint8_t>(btp::Error::Ok),
            static_cast<std::uint8_t>(btp::decode(
                sent_frames[i], sent_sizes[i], btp::TransportProfile::EspNow,
                &decoded[i])));
        TEST_ASSERT_EQUAL_HEX16(TelemetryPublisher::kSystemMonitorTopicId,
                                decoded[i].header.object_id);
        // Every fragment carries the one sequence the whole document was
        // sealed under, and the collection timestamp is preserved.
        TEST_ASSERT_EQUAL_UINT32(decoded[0].header.sequence,
                                 decoded[i].header.sequence);
        TEST_ASSERT_EQUAL_UINT64(7000000U, decoded[i].header.timestamp_us);
        TEST_ASSERT_EQUAL_UINT8(expected_fragments,
                                decoded[i].header.fragment_count);
        TEST_ASSERT_EQUAL_UINT8(i, decoded[i].header.fragment_index);
        TEST_ASSERT_TRUE((decoded[i].header.flags & btp::kFlagEncrypted) != 0U);
        TEST_ASSERT_TRUE((decoded[i].header.flags & btp::kFlagFragmented) != 0U);
        reassembled += decoded[i].payload.size;
    }
    TEST_ASSERT_EQUAL_UINT32(TelemetryPublisher::kMaxSystemMonitorPayloadSize +
                                 BtpEndpoint::kAeadTagSize,
                             reassembled);
}

// The staged system.monitor document and the numeric queue drain in the same
// flush() without either starving the other. Wire order between topics is not
// asserted -- a replace-in-place UTF-8 topic has no ordering requirement.
void test_monitor_and_numeric_samples_both_flush() {
    sent_count = 0U;
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(0x11223344U, 0xA1B2C3D4U));
    endpoint.set_send_callback(capture_send);

    TelemetryPublisher publisher;
    publisher.configure(endpoint);

    const char report[] = "================= Monitor Stats =================\n"
                          "core PRO    :   12.30 %\n";
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::Queued),
        static_cast<std::uint8_t>(publisher.publish_protocol_test(1U, 1.0F, 1U)));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::Queued),
        static_cast<std::uint8_t>(publisher.publish_system_monitor(
            report, std::strlen(report), 2U)));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::Queued),
        static_cast<std::uint8_t>(publisher.publish_robot_state(3U, 3U)));
    TEST_ASSERT_EQUAL_UINT32(3U, publisher.queued_count());

    TEST_ASSERT_EQUAL_UINT32(3U, publisher.flush(8U));
    TEST_ASSERT_EQUAL_UINT32(0U, publisher.queued_count());
    TEST_ASSERT_EQUAL_UINT32(3U, sent_count);

    bool saw_monitor = false;
    for (std::size_t i = 0U; i < sent_count; ++i) {
        btp::DecodedFrame frame{};
        TEST_ASSERT_EQUAL_UINT8(
            static_cast<std::uint8_t>(btp::Error::Ok),
            static_cast<std::uint8_t>(btp::decode(
                sent_frames[i], sent_sizes[i], btp::TransportProfile::EspNow,
                &frame)));
        if (frame.header.object_id == TelemetryPublisher::kSystemMonitorTopicId) {
            saw_monitor = true;
            TEST_ASSERT_EQUAL_UINT32(std::strlen(report) + 2U, frame.payload.size);
            TEST_ASSERT_EQUAL_UINT8_ARRAY(
                reinterpret_cast<const std::uint8_t*>(report),
                frame.payload.data + 2U, std::strlen(report));
        }
    }
    TEST_ASSERT_TRUE(saw_monitor);
}

void test_duplicate_command_executes_once_and_replays_exact_result() {
    sent_count = 0U;
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(0x11223344U, 0xA1B2C3D4U));
    endpoint.set_send_callback(capture_send);
    CommandProcessor processor;
    processor.configure(endpoint);

    const auto payload = shell_request_payload(endpoint.source_id(),
                                               endpoint.boot_id(),
                                               "logger -psram_usage");
    const btp::Header header = command_header(77U);
    const btp::ByteView bytes{payload.data(), payload.size()};
    const auto first = processor.intake(header, bytes, 2000U);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(CommandProcessor::IntakeKind::Ready),
        static_cast<std::uint8_t>(first.kind));

    const auto while_running = processor.intake(header, bytes, 2100U);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(
            CommandProcessor::IntakeKind::DuplicateInProgress),
        static_cast<std::uint8_t>(while_running.kind));

    CommandProcessor::ResultView completed{};
    TEST_ASSERT_TRUE(processor.complete(first.work.cache_slot, 0U, 3000U,
                                        &completed));
    const auto retry = processor.intake(header, bytes, 4000U);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(CommandProcessor::IntakeKind::ResultReady),
        static_cast<std::uint8_t>(retry.kind));
    TEST_ASSERT_EQUAL_UINT32(completed.sequence, retry.result.sequence);
    TEST_ASSERT_EQUAL_UINT64(completed.timestamp_us,
                             retry.result.timestamp_us);
    TEST_ASSERT_EQUAL_UINT32(completed.payload_size,
                             retry.result.payload_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(completed.payload, retry.result.payload,
                                  completed.payload_size);
    TEST_ASSERT_EQUAL_UINT32(1U, processor.stats().executed);

    TEST_ASSERT_TRUE(processor.send_result(retry.result));
    btp::DecodedFrame decoded{};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp::Error::Ok),
        static_cast<std::uint8_t>(btp::decode(
            sent_frames[0], sent_sizes[0], btp::TransportProfile::EspNow,
            &decoded)));
    TEST_ASSERT_EQUAL_HEX16(CommandProcessor::kCommandResultObjectId,
                            decoded.header.object_id);
    TEST_ASSERT_EQUAL_HEX32(header.source_id,
                            static_cast<std::uint32_t>(decoded.payload.data[0]) |
                            (static_cast<std::uint32_t>(decoded.payload.data[1]) << 8U) |
                            (static_cast<std::uint32_t>(decoded.payload.data[2]) << 16U) |
                            (static_cast<std::uint32_t>(decoded.payload.data[3]) << 24U));
    TEST_ASSERT_EQUAL_HEX8(0U, decoded.payload.data[16U]);
}

void test_channel_b_reply_is_sealed_with_endpoint_key_not_link_key() {
    sent_count = 0U;
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(0x11223344U, 0xA1B2C3D4U));
    endpoint.set_send_callback(capture_send);
    CommandProcessor processor;
    processor.configure(endpoint, fake_seal_link, nullptr, fake_seal_endpoint,
                        nullptr);

    const auto payload = shell_request_payload(endpoint.source_id(),
                                               endpoint.boot_id(),
                                               "sys -manifest");
    const btp::Header header = command_header(201U);
    const btp::ByteView bytes{payload.data(), payload.size()};
    const auto accepted =
        processor.intake(header, bytes, 5000U, bally::Channel::B_Endpoint);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(CommandProcessor::IntakeKind::Ready),
        static_cast<std::uint8_t>(accepted.kind));

    CommandProcessor::ResultView completed{};
    TEST_ASSERT_TRUE(processor.complete(accepted.work.cache_slot, 0U, 5100U,
                                        &completed));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(bally::Channel::B_Endpoint),
        static_cast<std::uint8_t>(completed.channel));

    TEST_ASSERT_TRUE(processor.send_result(completed));
    TEST_ASSERT_EQUAL_UINT32(1U, sent_count);

    btp::DecodedFrame decoded{};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp::Error::Ok),
        static_cast<std::uint8_t>(btp::decode(
            sent_frames[0], sent_sizes[0], btp::TransportProfile::EspNow,
            &decoded)));
    TEST_ASSERT_TRUE((decoded.header.flags & btp::kFlagEncrypted) != 0U);
    // The whole tag is 0x22 (fake_seal_endpoint's marker); 0x11 here would
    // mean the reply went out sealed with the CHANNEL-C key instead -- a
    // key whoever sent this channel-B request does not hold.
    TEST_ASSERT_EQUAL_HEX8(0x22U,
                          decoded.payload.data[decoded.payload.size - 1U]);
}

void test_channel_b_reply_without_endpoint_key_configured_is_dropped() {
    sent_count = 0U;
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(0x11223344U, 0xA1B2C3D4U));
    endpoint.set_send_callback(capture_send);
    CommandProcessor processor;
    // Channel C is keyed, channel B is not -- the gap this firmware was in
    // before topico 31, kept here as a regression test for the fail-closed
    // rule: a reply must never fall back to the other channel's key, or to
    // cleartext, just because ITS key is missing while some key exists.
    processor.configure(endpoint, fake_seal_link, nullptr);

    const auto payload = shell_request_payload(endpoint.source_id(),
                                               endpoint.boot_id(),
                                               "sys -manifest");
    const btp::Header header = command_header(202U);
    const btp::ByteView bytes{payload.data(), payload.size()};
    const auto accepted =
        processor.intake(header, bytes, 6000U, bally::Channel::B_Endpoint);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(CommandProcessor::IntakeKind::Ready),
        static_cast<std::uint8_t>(accepted.kind));

    CommandProcessor::ResultView completed{};
    TEST_ASSERT_TRUE(processor.complete(accepted.work.cache_slot, 0U, 6100U,
                                        &completed));

    const std::uint32_t dropped_before = processor.stats().dropped;
    TEST_ASSERT_FALSE(processor.send_result(completed));
    TEST_ASSERT_EQUAL_UINT32(dropped_before + 1U, processor.stats().dropped);
    TEST_ASSERT_EQUAL_UINT32(0U, sent_count);
}

void test_conflicting_duplicate_is_rejected_without_execution() {
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(1U, 2U));
    CommandProcessor processor;
    processor.configure(endpoint);
    const btp::Header header = command_header(88U);
    const auto original = shell_request_payload(1U, 2U, "help -modules");
    const auto changed = shell_request_payload(1U, 2U, "help -types");

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(CommandProcessor::IntakeKind::Ready),
        static_cast<std::uint8_t>(processor.intake(
            header, {original.data(), original.size()}, 1U).kind));
    const auto conflict = processor.intake(
        header, {changed.data(), changed.size()}, 2U);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(CommandProcessor::IntakeKind::ResultReady),
        static_cast<std::uint8_t>(conflict.kind));
    TEST_ASSERT_EQUAL_HEX8(
        static_cast<std::uint8_t>(CommandProcessor::Status::Rejected),
        conflict.result.payload[16U]);
    TEST_ASSERT_EQUAL_HEX16(
        static_cast<std::uint16_t>(CommandProcessor::ErrorCode::RequestConflict),
        static_cast<std::uint16_t>(conflict.result.payload[18U]) |
            static_cast<std::uint16_t>(conflict.result.payload[19U] << 8U));
    TEST_ASSERT_EQUAL_UINT32(0U, processor.stats().executed);
}

void test_saturated_execution_queue_returns_cached_busy_result() {
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(1U, 2U));
    CommandProcessor processor;
    processor.configure(endpoint);
    const btp::Header header = command_header(89U);
    const auto payload = shell_request_payload(1U, 2U, "help -modules");
    const btp::ByteView bytes{payload.data(), payload.size()};
    const auto work = processor.intake(header, bytes, 10U);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(CommandProcessor::IntakeKind::Ready),
        static_cast<std::uint8_t>(work.kind));

    CommandProcessor::ResultView busy{};
    TEST_ASSERT_TRUE(processor.reject_busy(work.work.cache_slot, 20U, &busy));
    TEST_ASSERT_EQUAL_HEX8(
        static_cast<std::uint8_t>(CommandProcessor::Status::Busy),
        busy.payload[16U]);
    TEST_ASSERT_EQUAL_HEX16(
        static_cast<std::uint16_t>(
            CommandProcessor::ErrorCode::CapacityExhausted),
        static_cast<std::uint16_t>(busy.payload[18U]) |
            static_cast<std::uint16_t>(busy.payload[19U] << 8U));

    const auto retry = processor.intake(header, bytes, 30U);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(CommandProcessor::IntakeKind::ResultReady),
        static_cast<std::uint8_t>(retry.kind));
    TEST_ASSERT_EQUAL_UINT32(busy.sequence, retry.result.sequence);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(busy.payload, retry.result.payload,
                                  busy.payload_size);
    TEST_ASSERT_EQUAL_UINT32(0U, processor.stats().executed);
}

// btp::DedupCache is a ring: once every slot holds a completed command, the
// next fresh identity evicts the OLDEST completed one so the executor keeps
// working -- and the evicted identity can never re-run
// (docs/commands.md 2.6 via btp::DedupCache's per-requester high-water mark).
void test_dedup_ring_evicts_oldest_and_never_re_executes_it() {
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(1U, 2U));
    CommandProcessor processor;
    processor.configure(endpoint);
    const auto request = shell_request_payload(1U, 2U, "help -modules");
    const btp::ByteView bytes{request.data(), request.size()};

    // Fill every dedup slot with a distinct, completed command (seq 1..16).
    for (std::uint32_t seq = 1U; seq <= CommandProcessor::kCacheCapacity; ++seq) {
        const auto in = processor.intake(command_header(seq), bytes, seq * 10U);
        TEST_ASSERT_EQUAL_UINT8(
            static_cast<std::uint8_t>(CommandProcessor::IntakeKind::Ready),
            static_cast<std::uint8_t>(in.kind));
        CommandProcessor::ResultView done{};
        TEST_ASSERT_TRUE(
            processor.complete(in.work.cache_slot, 0U, seq * 10U + 1U, &done));
    }
    TEST_ASSERT_EQUAL_UINT32(CommandProcessor::kCacheCapacity,
                             processor.stats().executed);

    // A brand-new identity is still served -- seq 1 is evicted to make room.
    const std::uint32_t fresh_seq = CommandProcessor::kCacheCapacity + 1U;
    const auto fresh = processor.intake(command_header(fresh_seq), bytes, 5000U);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(CommandProcessor::IntakeKind::Ready),
        static_cast<std::uint8_t>(fresh.kind));
    CommandProcessor::ResultView fresh_done{};
    TEST_ASSERT_TRUE(
        processor.complete(fresh.work.cache_slot, 0U, 5001U, &fresh_done));
    TEST_ASSERT_EQUAL_UINT32(fresh_seq, processor.stats().executed);

    // seq 2 is still cached: a retransmission replays, never re-executes.
    const auto still_cached = processor.intake(command_header(2U), bytes, 6000U);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(CommandProcessor::IntakeKind::ResultReady),
        static_cast<std::uint8_t>(still_cached.kind));

    // seq 1 was evicted: BUSY / CAPACITY_EXHAUSTED, and the shell never runs it.
    const auto evicted = processor.intake(command_header(1U), bytes, 7000U);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(CommandProcessor::IntakeKind::ResultReady),
        static_cast<std::uint8_t>(evicted.kind));
    TEST_ASSERT_EQUAL_HEX8(
        static_cast<std::uint8_t>(CommandProcessor::Status::Busy),
        evicted.result.payload[16U]);
    TEST_ASSERT_EQUAL_HEX16(
        static_cast<std::uint16_t>(
            CommandProcessor::ErrorCode::CapacityExhausted),
        static_cast<std::uint16_t>(evicted.result.payload[18U]) |
            static_cast<std::uint16_t>(evicted.result.payload[19U] << 8U));
    TEST_ASSERT_EQUAL_UINT32(fresh_seq, processor.stats().executed);
}

void test_full_telemetry_queue_cannot_block_command_result() {
    sent_count = 0U;
    TxScheduler scheduler;
    scheduler.configure(capture_radio, nullptr);
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(1U, 2U));
    endpoint.set_send_callback(TxScheduler::enqueue_callback, &scheduler);

    const std::uint8_t sample[] = {1U, 0U, 9U};
    for (std::uint32_t sequence = 1U; sequence <= 16U; ++sequence) {
        TEST_ASSERT_TRUE(endpoint.send_fragment(
            btp::MessageType::Telemetry, 1U, sequence, sequence, sample,
            sizeof(sample), 0U, 1U));
    }
    TEST_ASSERT_FALSE(endpoint.send_fragment(
        btp::MessageType::Telemetry, 1U, 17U, 17U, sample, sizeof(sample),
        0U, 1U));

    const std::uint8_t result_payload[26]{};
    TEST_ASSERT_TRUE(endpoint.send_fragment(
        btp::MessageType::Command,
        CommandProcessor::kCommandResultObjectId, 100U, 100U,
        result_payload, sizeof(result_payload), 0U, 1U));
    TEST_ASSERT_TRUE(scheduler.pump(0U));
    TEST_ASSERT_EQUAL_UINT32(1U, sent_count);

    btp::DecodedFrame sent{};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp::Error::Ok),
        static_cast<std::uint8_t>(btp::decode(
            sent_frames[0], sent_sizes[0], btp::TransportProfile::EspNow,
            &sent)));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp::MessageType::Command),
        static_cast<std::uint8_t>(sent.header.type));
    TEST_ASSERT_EQUAL_HEX16(CommandProcessor::kCommandResultObjectId,
                            sent.header.object_id);
    TEST_ASSERT_EQUAL_UINT32(1U, scheduler.stats().dropped);
}

void test_scheduler_counts_delivery_timeout_and_link_delivery() {
    sent_count = 0U;
    TxScheduler scheduler;
    scheduler.configure(capture_radio, nullptr, 10U);
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(1U, 2U));
    endpoint.set_send_callback(TxScheduler::enqueue_callback, &scheduler);
    const std::uint8_t payload[] = {0U};

    TEST_ASSERT_TRUE(endpoint.send_fragment(btp::MessageType::Log, 4U, 1U,
                                            1U, payload, sizeof(payload),
                                            0U, 1U));
    TEST_ASSERT_TRUE(scheduler.pump(100U));
    TEST_ASSERT_FALSE(scheduler.pump(111U));
    TEST_ASSERT_EQUAL_UINT32(1U, scheduler.stats().timeouts);

    TEST_ASSERT_TRUE(endpoint.send_fragment(btp::MessageType::Log, 3U, 2U,
                                            2U, payload, sizeof(payload),
                                            0U, 1U));
    TEST_ASSERT_TRUE(scheduler.pump(120U));
    scheduler.on_delivery(true);
    TEST_ASSERT_FALSE(scheduler.pump(121U));
    TEST_ASSERT_EQUAL_UINT32(1U, scheduler.stats().delivered);
}

// ---------------------------------------------------------------------------
// Topico 17: assinaturas e controle de taxa.
// ---------------------------------------------------------------------------

std::uint16_t read_u16(const std::uint8_t* data) {
    return static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[0]) |
                                      (static_cast<std::uint16_t>(data[1]) << 8U));
}

std::uint32_t read_u32(const std::uint8_t* data) {
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8U) |
           (static_cast<std::uint32_t>(data[2]) << 16U) |
           (static_cast<std::uint32_t>(data[3]) << 24U);
}

std::uint64_t read_u64(const std::uint8_t* data) {
    std::uint64_t value = 0U;
    for (std::size_t i = 0U; i < 8U; ++i) {
        value |= static_cast<std::uint64_t>(data[i]) << (8U * i);
    }
    return value;
}

constexpr std::uint32_t kLocalSource = 0x11223344U;
constexpr std::uint32_t kLocalBoot = 0xA1B2C3D4U;

btp::Header control_header(std::uint16_t object_id, std::uint32_t source_id,
                           std::uint32_t boot_id, std::uint32_t sequence) {
    return {
        .type = btp::MessageType::Control,
        .flags = 0U,
        .source_id = source_id,
        .boot_id = boot_id,
        .sequence = sequence,
        .timestamp_us = 1000U,
        .object_id = object_id,
        .fragment_index = 0U,
        .fragment_count = 1U,
    };
}

std::vector<std::uint8_t> subscribe_payload(std::uint16_t topic_id,
                                            std::uint32_t rate_millihz,
                                            std::uint32_t lease_ms) {
    std::vector<std::uint8_t> payload(20U, 0U);
    write_u32(payload.data(), kLocalSource);
    write_u32(payload.data() + 4U, kLocalBoot);
    write_u16(payload.data() + 8U, topic_id);
    write_u16(payload.data() + 10U, 0U);  // flags: zero in v1
    write_u32(payload.data() + 12U, rate_millihz);
    write_u32(payload.data() + 16U, lease_ms);
    return payload;
}

// Decodes the single frame captured by capture_send and returns its logical
// payload, which for every *_RESULT here fits in one unfragmented frame.
btp::DecodedFrame decode_only_frame() {
    btp::DecodedFrame decoded{};
    TEST_ASSERT_EQUAL_UINT32(1U, sent_count);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(btp::Error::Ok),
        static_cast<std::uint8_t>(btp::decode(sent_frames[0], sent_sizes[0],
                                              btp::TransportProfile::EspNow,
                                              &decoded)));
    return decoded;
}

// TELEMETRY rides channel B (bally_channels.h): configure() with an endpoint
// seal makes flush() route every sample through it, exactly like STATUS
// already seals on channel C. The dongle relays these and never reads them,
// so the real caller passes RadioSeal::seal_e. Without a seal (the default)
// the sample goes out in the clear, which is what every other test here
// relies on.
void test_telemetry_sample_is_sealed_when_configured_with_a_seal_function() {
    sent_count = 0U;
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(0x11223344U, 0xA1B2C3D4U));
    endpoint.set_send_callback(capture_send);

    TelemetryPublisher publisher;
    publisher.configure(endpoint, fake_seal_endpoint, nullptr);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::Queued),
        static_cast<std::uint8_t>(publisher.publish_protocol_test(
            0x01020304U, float_from_bits(0x3F0D0A00U), 1000000U)));

    TEST_ASSERT_EQUAL_UINT32(1U, publisher.flush(1U));
    btp::DecodedFrame decoded = decode_only_frame();
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(btp::MessageType::Telemetry),
                            static_cast<std::uint8_t>(decoded.header.type));
    TEST_ASSERT_TRUE((decoded.header.flags & btp::kFlagEncrypted) != 0U);
    // 0x22 is fake_seal_endpoint's tag marker (key E); 0x11 would mean this
    // went out under the channel-C key, which no TraceView holds.
    TEST_ASSERT_EQUAL_HEX8(0x22U,
                           decoded.payload.data[decoded.payload.size - 1U]);
}

void test_telemetry_sample_with_no_key_configured_is_dropped_not_sent_clear() {
    sent_count = 0U;
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(0x11223344U, 0xA1B2C3D4U));
    endpoint.set_send_callback(capture_send);

    // A seal that always fails stands in for "key E not loaded".
    TelemetryPublisher publisher;
    publisher.configure(endpoint, always_failing_seal, nullptr);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::Queued),
        static_cast<std::uint8_t>(publisher.publish_protocol_test(
            0x01020304U, 0.0f, 1000000U)));

    TEST_ASSERT_EQUAL_UINT32(1U, publisher.flush(1U));  // consumed from the queue
    TEST_ASSERT_EQUAL_UINT32(0U, sent_count);           // but nothing on the wire
    TEST_ASSERT_EQUAL_UINT32(1U, publisher.stats().send_failed);
}

// Acceptance criterion: "pedido acima do maximo e limitado e informado ao
// cliente" -- clamped, answered SUCCESS, never rejected. The mirror case
// (below the schema's floor) is rejected, because section 4 forbids granting
// a rate above the requested one.
void test_subscribe_above_max_is_clamped_and_below_min_is_rejected() {
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(kLocalSource, kLocalBoot));
    endpoint.set_send_callback(capture_send);
    TelemetryPublisher publisher;
    publisher.configure(endpoint);
    SubscriptionResponder responder;
    responder.configure(endpoint, publisher);

    const auto request = subscribe_payload(
        TelemetryPublisher::kProtocolTestTopicId, 200000U, 5000U);
    const btp::Header header = control_header(
        SubscriptionResponder::kSubscribeObjectId, 0x0C30AA5CU, 0x10203040U, 7U);

    sent_count = 0U;
    TEST_ASSERT_TRUE(responder.handle_subscribe(
        header, {request.data(), request.size()}, 1000U));
    btp::DecodedFrame decoded = decode_only_frame();
    TEST_ASSERT_EQUAL_HEX16(SubscriptionResponder::kSubscribeResultObjectId,
                            decoded.header.object_id);
    TEST_ASSERT_EQUAL_UINT32(28U, decoded.payload.size);
    // Reference to the request: (source, boot, reply_to_sequence).
    TEST_ASSERT_EQUAL_HEX32(0x0C30AA5CU, read_u32(decoded.payload.data));
    TEST_ASSERT_EQUAL_HEX32(0x10203040U, read_u32(decoded.payload.data + 4U));
    TEST_ASSERT_EQUAL_UINT32(7U, read_u32(decoded.payload.data + 8U));
    TEST_ASSERT_EQUAL_UINT8(0x00U, decoded.payload.data[12]);   // SUCCESS
    TEST_ASSERT_EQUAL_HEX16(0x0000U, read_u16(decoded.payload.data + 14U));
    TEST_ASSERT_NOT_EQUAL(0U, read_u32(decoded.payload.data + 16U));
    // 200 Hz requested, schema max is 50 Hz: clamped, and the client is told.
    TEST_ASSERT_EQUAL_UINT32(50000U, read_u32(decoded.payload.data + 20U));
    TEST_ASSERT_EQUAL_UINT32(5000U, read_u32(decoded.payload.data + 24U));
    // The topic now publishes at the granted rate, not the requested one.
    TEST_ASSERT_EQUAL_UINT64(
        20000U,
        publisher.topic_period_us(TelemetryPublisher::kProtocolTestTopicId));

    // Below the schema floor (0.1 Hz for protocol.test): rejected with
    // INVALID_ARGUMENT instead of being silently raised.
    const auto slow = subscribe_payload(
        TelemetryPublisher::kProtocolTestTopicId, 50U, 5000U);
    sent_count = 0U;
    TEST_ASSERT_TRUE(responder.handle_subscribe(
        control_header(SubscriptionResponder::kSubscribeObjectId, 0x0C30AA5CU,
                       0x10203040U, 8U),
        {slow.data(), slow.size()}, 2000U));
    decoded = decode_only_frame();
    TEST_ASSERT_EQUAL_UINT8(0x01U, decoded.payload.data[12]);  // REJECTED
    TEST_ASSERT_EQUAL_HEX16(0x0003U, read_u16(decoded.payload.data + 14U));
    TEST_ASSERT_EQUAL_UINT32(0U, read_u32(decoded.payload.data + 16U));
    TEST_ASSERT_EQUAL_UINT32(0U, read_u32(decoded.payload.data + 20U));
    TEST_ASSERT_EQUAL_UINT32(0U, read_u32(decoded.payload.data + 24U));

    // A non-periodic topic (max_rate_millihz == 0) is capped by the schema's
    // nominal default instead, so the answer is never an arbitrary echo.
    const auto state = subscribe_payload(
        TelemetryPublisher::kRobotStateTopicId, 999000U, 5000U);
    sent_count = 0U;
    TEST_ASSERT_TRUE(responder.handle_subscribe(
        control_header(SubscriptionResponder::kSubscribeObjectId, 0x0C30AA5CU,
                       0x10203040U, 9U),
        {state.data(), state.size()}, 3000U));
    decoded = decode_only_frame();
    TEST_ASSERT_EQUAL_UINT8(0x00U, decoded.payload.data[12]);
    TEST_ASSERT_EQUAL_UINT32(10000U, read_u32(decoded.payload.data + 20U));
}

// Mirrors test_channel_b_reply_is_sealed_with_endpoint_key_not_link_key
// (CommandProcessor) for SUBSCRIBE_RESULT: topico 31.2 widened
// SubscriptionResponder to the same seal_link/seal_endpoint-by-channel rule.
void test_subscribe_channel_b_reply_is_sealed_with_endpoint_key_not_link_key() {
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(kLocalSource, kLocalBoot));
    endpoint.set_send_callback(capture_send);
    TelemetryPublisher publisher;
    publisher.configure(endpoint);
    SubscriptionResponder responder;
    responder.configure(endpoint, publisher, fake_seal_link, nullptr,
                        fake_seal_endpoint, nullptr);

    const auto request = subscribe_payload(
        TelemetryPublisher::kProtocolTestTopicId, 10000U, 5000U);
    const btp::Header header = control_header(
        SubscriptionResponder::kSubscribeObjectId, 0x0C30AA5CU, 0x10203040U, 40U);

    sent_count = 0U;
    TEST_ASSERT_TRUE(responder.handle_subscribe(
        header, {request.data(), request.size()}, 1000U, bally::Channel::B_Endpoint));

    btp::DecodedFrame decoded = decode_only_frame();
    TEST_ASSERT_TRUE((decoded.header.flags & btp::kFlagEncrypted) != 0U);
    // 0x22 is fake_seal_endpoint's marker; 0x11 would mean this went out
    // sealed with the CHANNEL-C key instead, which a TraceView hub-channel
    // requester does not hold.
    TEST_ASSERT_EQUAL_HEX8(0x22U,
                          decoded.payload.data[decoded.payload.size - 1U]);
}

void test_subscribe_channel_b_reply_without_endpoint_key_configured_is_dropped() {
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(kLocalSource, kLocalBoot));
    endpoint.set_send_callback(capture_send);
    TelemetryPublisher publisher;
    publisher.configure(endpoint);
    SubscriptionResponder responder;
    // Channel C is keyed, channel B is not: the reply must be dropped, never
    // fall back to the link key or to cleartext just because SOME key exists.
    responder.configure(endpoint, publisher, fake_seal_link, nullptr);

    const auto request = subscribe_payload(
        TelemetryPublisher::kProtocolTestTopicId, 10000U, 5000U);
    const btp::Header header = control_header(
        SubscriptionResponder::kSubscribeObjectId, 0x0C30AA5CU, 0x10203040U, 41U);

    sent_count = 0U;
    TEST_ASSERT_FALSE(responder.handle_subscribe(
        header, {request.data(), request.size()}, 1000U, bally::Channel::B_Endpoint));
    TEST_ASSERT_EQUAL_UINT32(0U, sent_count);
}

// PASSO 5: several sessions on one topic aggregate; the slow one never
// throttles the fast one and each is told its own granted rate.
void test_multiple_subscribers_aggregate_on_one_topic() {
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(kLocalSource, kLocalBoot));
    TelemetryPublisher publisher;
    publisher.configure(endpoint);

    const auto slow = publisher.subscribe(
        TelemetryPublisher::kProtocolTestTopicId, 0xAAAAU, 1U, 1000U, 5000U, 0U);
    const auto fast = publisher.subscribe(
        TelemetryPublisher::kProtocolTestTopicId, 0xBBBBU, 1U, 25000U, 5000U, 0U);
    TEST_ASSERT_TRUE(slow.topic_known);
    TEST_ASSERT_TRUE(fast.topic_known);
    TEST_ASSERT_NOT_EQUAL(slow.subscription_id, fast.subscription_id);
    TEST_ASSERT_EQUAL_UINT32(1000U, slow.effective_rate_millihz);
    TEST_ASSERT_EQUAL_UINT32(25000U, fast.effective_rate_millihz);

    TEST_ASSERT_EQUAL_UINT16(
        2U, publisher.topic_subscriber_count(
                TelemetryPublisher::kProtocolTestTopicId));
    TEST_ASSERT_EQUAL_UINT32(
        25000U, publisher.topic_effective_rate_millihz(
                    TelemetryPublisher::kProtocolTestTopicId));
    TEST_ASSERT_EQUAL_UINT64(
        40000U,
        publisher.topic_period_us(TelemetryPublisher::kProtocolTestTopicId));

    // An exact repeat of the same request from the same session renews the
    // lease and returns the same subscription instead of creating another.
    const auto repeat = publisher.subscribe(
        TelemetryPublisher::kProtocolTestTopicId, 0xAAAAU, 1U, 1000U, 5000U, 500U);
    TEST_ASSERT_EQUAL_UINT32(slow.subscription_id, repeat.subscription_id);
    TEST_ASSERT_EQUAL_UINT32(2U, publisher.active_subscription_count());

    // Different bytes from the same session atomically replace it.
    const auto changed = publisher.subscribe(
        TelemetryPublisher::kProtocolTestTopicId, 0xAAAAU, 1U, 2000U, 5000U, 600U);
    TEST_ASSERT_NOT_EQUAL(slow.subscription_id, changed.subscription_id);
    TEST_ASSERT_EQUAL_UINT32(2U, publisher.active_subscription_count());
}

// Acceptance criterion: "fechar um grafico reduz trafego quando nenhum outro
// consumidor usa o topico" -- and only then.
void test_topic_keeps_publishing_until_the_last_consumer_leaves() {
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(kLocalSource, kLocalBoot));
    TelemetryPublisher publisher;
    publisher.configure(endpoint);

    const auto first = publisher.subscribe(
        TelemetryPublisher::kProtocolTestTopicId, 0xAAAAU, 1U, 50000U, 5000U, 0U);
    const auto second = publisher.subscribe(
        TelemetryPublisher::kProtocolTestTopicId, 0xBBBBU, 1U, 10000U, 5000U, 0U);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::UnsubscribeOutcome::Removed),
        static_cast<std::uint8_t>(publisher.unsubscribe(first.subscription_id)));
    // The fast consumer left; the topic keeps going for the slow one, now at
    // the slow one's rate.
    TEST_ASSERT_TRUE(
        publisher.topic_active(TelemetryPublisher::kProtocolTestTopicId));
    TEST_ASSERT_EQUAL_UINT32(
        10000U, publisher.topic_effective_rate_millihz(
                    TelemetryPublisher::kProtocolTestTopicId));

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::UnsubscribeOutcome::Removed),
        static_cast<std::uint8_t>(publisher.unsubscribe(second.subscription_id)));
    TEST_ASSERT_FALSE(
        publisher.topic_active(TelemetryPublisher::kProtocolTestTopicId));
    TEST_ASSERT_EQUAL_UINT64(
        0U, publisher.topic_period_us(TelemetryPublisher::kProtocolTestTopicId));

    // Removing an already-absent subscription stays idempotent.
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::UnsubscribeOutcome::NotFound),
        static_cast<std::uint8_t>(publisher.unsubscribe(second.subscription_id)));
}

// PASSO 6: lease expiry and peer reboot both end a session's subscriptions.
void test_lease_expiry_and_new_boot_id_end_a_session() {
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(kLocalSource, kLocalBoot));
    TelemetryPublisher publisher;
    publisher.configure(endpoint);

    // 2000 ms lease granted at t=1 s.
    const auto held = publisher.subscribe(
        TelemetryPublisher::kProtocolTestTopicId, 0xAAAAU, 1U, 50000U, 2000U,
        1000000U);
    TEST_ASSERT_EQUAL_UINT32(2000U, held.granted_lease_ms);

    publisher.expire_subscriptions(2999999U);
    TEST_ASSERT_TRUE(
        publisher.topic_active(TelemetryPublisher::kProtocolTestTopicId));
    publisher.expire_subscriptions(3000000U);
    TEST_ASSERT_FALSE(
        publisher.topic_active(TelemetryPublisher::kProtocolTestTopicId));
    TEST_ASSERT_EQUAL_UINT64(
        0U, publisher.topic_period_us(TelemetryPublisher::kProtocolTestTopicId));

    // A lease shorter than the local floor is raised, so an abandoned client
    // is never swept faster than the minimum.
    const auto short_lease = publisher.subscribe(
        TelemetryPublisher::kProtocolTestTopicId, 0xAAAAU, 2U, 50000U, 1U, 0U);
    TEST_ASSERT_EQUAL_UINT32(1000U, short_lease.granted_lease_ms);

    // Same peer, new boot_id: the old session's subscriptions are released
    // before the new one is granted.
    publisher.subscribe(TelemetryPublisher::kRobotStateTopicId, 0xAAAAU, 2U,
                        5000U, 5000U, 0U);
    TEST_ASSERT_EQUAL_UINT32(2U, publisher.active_subscription_count());
    publisher.subscribe(TelemetryPublisher::kProtocolTestTopicId, 0xAAAAU, 3U,
                        50000U, 5000U, 0U);
    TEST_ASSERT_EQUAL_UINT32(1U, publisher.active_subscription_count());
    TEST_ASSERT_EQUAL_UINT16(
        0U, publisher.topic_subscriber_count(
                TelemetryPublisher::kRobotStateTopicId));

    // Explicit teardown of the surviving session.
    TEST_ASSERT_EQUAL_UINT32(1U, publisher.drop_session(0xAAAAU, 3U));
    TEST_ASSERT_EQUAL_UINT32(0U, publisher.active_subscription_count());
}

// PASSO 8/9: bytes and drops are measured per topic and reach the wire as the
// 28-octet topic_status records of commands.md section 5.1.
void test_topic_status_is_measured_and_serialized() {
    sent_count = 0U;
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(kLocalSource, kLocalBoot));
    endpoint.set_send_callback(capture_send);
    TelemetryPublisher publisher;
    publisher.configure(endpoint);
    publisher.subscribe(TelemetryPublisher::kProtocolTestTopicId, 0xAAAAU, 1U,
                        50000U, 5000U, 0U);

    // One published sample: 10 octets of logical TELEMETRY payload, no
    // envelope, no CRC.
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::Queued),
        static_cast<std::uint8_t>(publisher.publish_protocol_test(
            1U, float_from_bits(0x3F0D0A00U), 1000U)));
    TEST_ASSERT_EQUAL_UINT32(1U, publisher.flush(1U));

    // Then fill the queue and force one drop on the same topic.
    for (std::size_t i = 0U; i < TelemetryPublisher::kQueueCapacity; ++i) {
        publisher.publish_protocol_test(static_cast<std::uint32_t>(i),
                                        float_from_bits(0x3F0D0A00U), 2000U);
    }
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<std::uint8_t>(TelemetryPublisher::PublishResult::QueueFull),
        static_cast<std::uint8_t>(publisher.publish_protocol_test(
            99U, float_from_bits(0x3F0D0A00U), 3000U)));

    TelemetryPublisher::TopicStats stats[4]{};
    TEST_ASSERT_EQUAL_UINT32(3U, publisher.topic_stats(stats, 4U));
    TEST_ASSERT_EQUAL_HEX16(TelemetryPublisher::kProtocolTestTopicId,
                            stats[0].topic_id);
    TEST_ASSERT_EQUAL_UINT16(1U, stats[0].subscriber_count);
    TEST_ASSERT_EQUAL_UINT32(50000U, stats[0].effective_rate_millihz);
    TEST_ASSERT_EQUAL_UINT64(TelemetryPublisher::kProtocolTestPayloadSize,
                             stats[0].bytes_sent_total);
    TEST_ASSERT_EQUAL_UINT64(1U, stats[0].samples_dropped_total);
    TEST_ASSERT_EQUAL_UINT16(0U, stats[1].subscriber_count);
    TEST_ASSERT_EQUAL_UINT32(0U, stats[1].effective_rate_millihz);

    StatusReporter::Counters counters{};
    counters.frames_rx = 0x0102030405060708ULL;
    counters.telemetry_dropped = 1U;
    StatusReporter::TopicRecord records[2]{};
    records[0].source_id = kLocalSource;
    records[0].topic_id = TelemetryPublisher::kProtocolTestTopicId;
    records[0].subscriber_count = 2U;
    records[0].effective_rate_millihz = 50000U;
    records[0].bytes_total = 0x1122334455667788ULL;
    records[0].samples_dropped_total = 7U;
    records[1].source_id = kLocalSource;
    records[1].topic_id = TelemetryPublisher::kRobotStateTopicId;

    std::uint8_t payload[StatusReporter::kMaxPayloadSize]{};
    const std::size_t size = StatusReporter::serialize(
        StatusReporter::kFlagDegraded, 0x00000000DEADBEEFULL, counters, records,
        2U, payload, sizeof(payload));
    TEST_ASSERT_EQUAL_UINT32(92U + 2U + (28U * 2U), size);
    TEST_ASSERT_EQUAL_UINT16(2U, read_u16(payload));  // status_version
    TEST_ASSERT_EQUAL_HEX16(0x0001U, read_u16(payload + 2U));  // DEGRADED
    TEST_ASSERT_EQUAL_UINT64(0x00000000DEADBEEFULL, read_u64(payload + 4U));
    TEST_ASSERT_EQUAL_UINT64(0x0102030405060708ULL, read_u64(payload + 12U));
    TEST_ASSERT_EQUAL_UINT64(1U, read_u64(payload + 84U));
    TEST_ASSERT_EQUAL_UINT16(2U, read_u16(payload + 92U));  // topic_status_count
    TEST_ASSERT_EQUAL_HEX32(kLocalSource, read_u32(payload + 94U));
    TEST_ASSERT_EQUAL_HEX16(TelemetryPublisher::kProtocolTestTopicId,
                            read_u16(payload + 98U));
    TEST_ASSERT_EQUAL_UINT16(2U, read_u16(payload + 100U));
    TEST_ASSERT_EQUAL_UINT32(50000U, read_u32(payload + 102U));
    TEST_ASSERT_EQUAL_UINT64(0x1122334455667788ULL, read_u64(payload + 106U));
    TEST_ASSERT_EQUAL_UINT64(7U, read_u64(payload + 114U));
    TEST_ASSERT_EQUAL_HEX16(TelemetryPublisher::kRobotStateTopicId,
                            read_u16(payload + 94U + 28U + 4U));

    // A record without a source_id or topic_id is not emitted, and the count
    // reflects what was actually written.
    records[1].topic_id = 0U;
    const std::size_t trimmed = StatusReporter::serialize(
        0U, 1U, counters, records, 2U, payload, sizeof(payload));
    TEST_ASSERT_EQUAL_UINT32(92U + 2U + 28U, trimmed);
    TEST_ASSERT_EQUAL_UINT16(1U, read_u16(payload + 92U));

    // status_version=1 MUST NOT carry the per-topic block at all.
    const std::size_t legacy = StatusReporter::serialize(
        0U, 1U, counters, records, 2U, payload, sizeof(payload), true);
    TEST_ASSERT_EQUAL_UINT32(92U, legacy);
    TEST_ASSERT_EQUAL_UINT16(1U, read_u16(payload));
}

// The reporter stamps this robot's own source_id on every record and sends
// one unfragmented CONTROL/STATUS.
void test_status_is_published_as_a_control_message() {
    sent_count = 0U;
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(kLocalSource, kLocalBoot));
    endpoint.set_send_callback(capture_send);
    TelemetryPublisher publisher;
    publisher.configure(endpoint);
    publisher.subscribe(TelemetryPublisher::kProtocolTestTopicId, 0xAAAAU, 1U,
                        50000U, 5000U, 0U);

    StatusReporter reporter;
    reporter.configure(endpoint, publisher);
    StatusReporter::Counters counters{};
    TEST_ASSERT_TRUE(reporter.publish(0U, 123456U, counters, 123456U));

    const btp::DecodedFrame decoded = decode_only_frame();
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(btp::MessageType::Control),
                            static_cast<std::uint8_t>(decoded.header.type));
    TEST_ASSERT_EQUAL_HEX16(StatusReporter::kStatusObjectId,
                            decoded.header.object_id);
    TEST_ASSERT_EQUAL_UINT8(1U, decoded.header.fragment_count);
    TEST_ASSERT_EQUAL_UINT32(92U + 2U + (28U * 3U), decoded.payload.size);
    TEST_ASSERT_EQUAL_UINT16(2U, read_u16(decoded.payload.data));
    TEST_ASSERT_EQUAL_UINT16(3U, read_u16(decoded.payload.data + 92U));
    TEST_ASSERT_EQUAL_HEX32(kLocalSource, read_u32(decoded.payload.data + 94U));
    TEST_ASSERT_EQUAL_UINT32(50000U, read_u32(decoded.payload.data + 102U));
}

// Rate control must not touch the sample itself: same schema_version, same
// payload bytes and the origin timestamp, whatever rate was granted.
void test_rate_control_changes_neither_timestamp_nor_schema() {
    sent_count = 0U;
    BtpEndpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(kLocalSource, kLocalBoot));
    endpoint.set_send_callback(capture_send);
    TelemetryPublisher publisher;
    publisher.configure(endpoint);
    publisher.subscribe(TelemetryPublisher::kProtocolTestTopicId, 0xAAAAU, 1U,
                        1000U, 5000U, 0U);

    publisher.publish_protocol_test(0x01020304U, float_from_bits(0x3F0D0A00U),
                                    987654321U);
    TEST_ASSERT_EQUAL_UINT32(1U, publisher.flush(1U));
    const btp::DecodedFrame decoded = decode_only_frame();
    TEST_ASSERT_EQUAL_UINT64(987654321U, decoded.header.timestamp_us);
    TEST_ASSERT_EQUAL_UINT16(TelemetryPublisher::kSchemaVersion,
                             read_u16(decoded.payload.data));
    TEST_ASSERT_EQUAL_UINT32(TelemetryPublisher::kProtocolTestPayloadSize,
                             decoded.payload.size);
    TEST_ASSERT_EQUAL_HEX32(0x01020304U, read_u32(decoded.payload.data + 2U));
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_canonical_command_request_is_fully_parsed);
    RUN_TEST(test_canonical_crc_failure_is_rejected);
    RUN_TEST(test_telemetry_can_never_become_a_command);
    RUN_TEST(test_only_supported_single_shell_command_is_copied);
    RUN_TEST(test_endpoint_fragments_with_one_shared_sequence_and_exact_sizes);
    RUN_TEST(test_peer_mac_filter_accepts_relayed_source_ids);
    RUN_TEST(test_protocol_test_matches_canonical_vector_and_origin_timestamp);
    RUN_TEST(test_telemetry_sample_is_sealed_when_configured_with_a_seal_function);
    RUN_TEST(test_telemetry_sample_with_no_key_configured_is_dropped_not_sent_clear);
    RUN_TEST(test_publisher_queue_is_bounded_and_drop_newest_is_counted);
    RUN_TEST(test_publisher_registers_static_schemas_and_rejects_nan);
    RUN_TEST(test_system_monitor_publishes_complete_utf8_document);
    RUN_TEST(test_large_sealed_monitor_fragments_and_reassembles);
    RUN_TEST(test_monitor_and_numeric_samples_both_flush);
    RUN_TEST(test_duplicate_command_executes_once_and_replays_exact_result);
    RUN_TEST(test_channel_b_reply_is_sealed_with_endpoint_key_not_link_key);
    RUN_TEST(test_channel_b_reply_without_endpoint_key_configured_is_dropped);
    RUN_TEST(test_conflicting_duplicate_is_rejected_without_execution);
    RUN_TEST(test_saturated_execution_queue_returns_cached_busy_result);
    RUN_TEST(test_dedup_ring_evicts_oldest_and_never_re_executes_it);
    RUN_TEST(test_full_telemetry_queue_cannot_block_command_result);
    RUN_TEST(test_scheduler_counts_delivery_timeout_and_link_delivery);
    RUN_TEST(test_subscribe_above_max_is_clamped_and_below_min_is_rejected);
    RUN_TEST(test_subscribe_channel_b_reply_is_sealed_with_endpoint_key_not_link_key);
    RUN_TEST(test_subscribe_channel_b_reply_without_endpoint_key_configured_is_dropped);
    RUN_TEST(test_multiple_subscribers_aggregate_on_one_topic);
    RUN_TEST(test_topic_keeps_publishing_until_the_last_consumer_leaves);
    RUN_TEST(test_lease_expiry_and_new_boot_id_end_a_session);
    RUN_TEST(test_topic_status_is_measured_and_serialized);
    RUN_TEST(test_status_is_published_as_a_control_message);
    RUN_TEST(test_rate_control_changes_neither_timestamp_nor_schema);
    return UNITY_END();
}
