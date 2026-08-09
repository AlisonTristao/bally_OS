#include <unity.h>

#include <BtpTransport.h>
#include <CommandProcessor.h>
#include <TelemetryPublisher.h>
#include <TxScheduler.h>
#include <btp/codec.hpp>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

std::vector<std::uint8_t> read_vector(const char* relative_path) {
    const std::string candidates[] = {
        std::string("../bally_protocol/test-vectors/v1/") + relative_path,
        std::string("../../bally_protocol/test-vectors/v1/") + relative_path,
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

std::uint8_t sent_frames[2][btp::kEspNowMaxFrameSize]{};
std::size_t sent_sizes[2]{};
std::size_t sent_count = 0U;

bool capture_send(const std::uint8_t* data, std::size_t size) {
    if (sent_count >= 2U || size > btp::kEspNowMaxFrameSize) return false;
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

void test_authorization_binds_mac_to_claimed_source() {
    const std::uint8_t expected[6] = {0xDC, 0xDA, 0x0C, 0x30, 0xAA, 0x5C};
    const std::uint8_t attacker[6] = {0xDC, 0xDA, 0x0C, 0x30, 0xAA, 0x5D};
    const std::uint32_t source = btp_command::source_id_from_mac(expected);
    TEST_ASSERT_EQUAL_HEX32(0x0C30AA5CU, source);
    TEST_ASSERT_TRUE(btp_command::authorized_source(expected, expected, source));
    TEST_ASSERT_FALSE(btp_command::authorized_source(expected, attacker, source));
    TEST_ASSERT_FALSE(btp_command::authorized_source(expected, expected,
                                                     source + 1U));
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
    TEST_ASSERT_EQUAL_UINT32(2U, schema_count);
    TEST_ASSERT_EQUAL_STRING("protocol.test", schemas[0].name);
    TEST_ASSERT_EQUAL_HEX16(TelemetryPublisher::kProtocolTestTopicId,
                            schemas[0].topic_id);
    TEST_ASSERT_EQUAL_STRING("robot.state", schemas[1].name);

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
    RUN_TEST(test_authorization_binds_mac_to_claimed_source);
    RUN_TEST(test_protocol_test_matches_canonical_vector_and_origin_timestamp);
    RUN_TEST(test_publisher_queue_is_bounded_and_drop_newest_is_counted);
    RUN_TEST(test_publisher_registers_static_schemas_and_rejects_nan);
    RUN_TEST(test_duplicate_command_executes_once_and_replays_exact_result);
    RUN_TEST(test_conflicting_duplicate_is_rejected_without_execution);
    RUN_TEST(test_saturated_execution_queue_returns_cached_busy_result);
    RUN_TEST(test_full_telemetry_queue_cannot_block_command_result);
    RUN_TEST(test_scheduler_counts_delivery_timeout_and_link_delivery);
    return UNITY_END();
}
