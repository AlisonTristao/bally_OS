#include <unity.h>

#include <BtpTransport.h>
#include <RxRouter.h>
#include <btp/codec.hpp>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

// RxRouter on the host: decode + CRC + reassembly, driven the same way the
// ESP-NOW receive callback drives it, with no ESP32 in the loop. These are
// the cases the hub model makes routine -- several senders fragmenting at
// once behind one radio peer -- plus the bounds that keep a four-slot pool
// from being a denial of service.

namespace {

// Same loader as test/btp_integration/test_main.cpp: canonical vectors live
// in a sibling checkout of the protocol repo, reached relative to the
// project root (or one level deeper, depending on where the runner starts).
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

// A slot has to hold the largest logical message the robot accepts today.
// RxRouter deliberately does not include BtpTransport.h to say so (it must
// not depend on the command envelope to reassemble bytes), so the agreement
// is asserted here, where including both costs nothing.
static_assert(RxRouter::kMaxPayloadSize >= btp_command::kMaxLogicalRequestSize,
              "a COMMAND_REQUEST must fit in one reassembly slot");

constexpr std::uint16_t kTestObjectId = btp_command::kCommandRequestObjectId;

struct Encoded {
    std::uint8_t bytes[btp::kEspNowMaxFrameSize];
    std::size_t size;
};

// One fragment on the wire. Uses the firmware's own encoder so the test
// cannot drift from what the robot actually emits and accepts.
Encoded encode_fragment(const BtpEndpoint& endpoint,
                        std::uint32_t sequence,
                        const std::uint8_t* logical_payload,
                        std::size_t logical_size,
                        std::uint8_t fragment_index,
                        std::uint8_t fragment_count) {
    const std::size_t limit = btp::kEspNowMaxPayloadSize;
    const std::size_t offset = static_cast<std::size_t>(fragment_index) * limit;
    const std::size_t remaining = logical_size - offset;
    const std::size_t size = remaining < limit ? remaining : limit;

    Encoded encoded{};
    const bool ok = endpoint.encode_fragment(
        btp::MessageType::Command, kTestObjectId, sequence, 1000U,
        logical_payload + offset, size, fragment_index, fragment_count,
        encoded.bytes, sizeof(encoded.bytes), &encoded.size);
    TEST_ASSERT_TRUE_MESSAGE(ok, "encode_fragment failed");
    return encoded;
}

// BtpEndpoint holds atomics, so it is neither copyable nor movable; it is
// configured in place rather than returned from a factory.
void configure(BtpEndpoint& endpoint, std::uint32_t source_id,
               std::uint32_t boot_id) {
    TEST_ASSERT_TRUE(endpoint.configure(source_id, boot_id));
}

// 300 octets is deliberately just over the 210-octet ESP-NOW ceiling: two
// fragments, the first exactly full and the second short, which is the shape
// BTP's positional cut always produces.
constexpr std::size_t kLogicalSize = 300U;

void fill_pattern(std::uint8_t* buffer, std::size_t size, std::uint8_t seed) {
    for (std::size_t index = 0U; index < size; ++index) {
        buffer[index] = static_cast<std::uint8_t>(seed + index * 7U);
    }
}

RxRouter::Outcome submit(RxRouter::Router& router,
                         const Encoded& frame,
                         std::uint64_t now_ms,
                         RxRouter::RoutedMessage* out) {
    return router.submit(frame.bytes, frame.size, now_ms, out);
}

// ---------------------------------------------------------------------------

// The unfragmented path must not consume a slot, and the way to prove that is
// to have none left: four reassemblies are already in flight when the whole
// message arrives.
void test_unfragmented_message_crosses_without_touching_a_slot() {
    BtpEndpoint endpoint;
    configure(endpoint, 0x0C30AA5CU, 0x10203040U);
    RxRouter::Router router;
    TEST_ASSERT_TRUE(router.valid());
    RxRouter::RoutedMessage routed{};

    std::uint8_t logical[kLogicalSize];
    fill_pattern(logical, sizeof(logical), 0x31U);
    for (std::uint32_t sequence = 1U; sequence <= RxRouter::kSlotCount;
         ++sequence) {
        const Encoded first = encode_fragment(
            endpoint, sequence, logical, sizeof(logical), 0U, 2U);
        TEST_ASSERT_EQUAL(
            static_cast<int>(RxRouter::Outcome::FragmentAccepted),
            static_cast<int>(submit(router, first, 10U, &routed)));
    }

    const std::uint8_t whole[] = {'p', 'i', 'n', 'g'};
    const Encoded single =
        encode_fragment(endpoint, 99U, whole, sizeof(whole), 0U, 1U);
    TEST_ASSERT_EQUAL(static_cast<int>(RxRouter::Outcome::Routed),
                      static_cast<int>(submit(router, single, 10U, &routed)));
    TEST_ASSERT_FALSE(routed.reassembled);
    TEST_ASSERT_EQUAL_UINT32(sizeof(whole), routed.payload_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(whole, routed.payload, sizeof(whole));

    const RxRouter::Stats stats = router.stats();
    TEST_ASSERT_EQUAL_UINT32(1U, stats.routed);
    TEST_ASSERT_EQUAL_UINT32(RxRouter::kSlotCount, stats.fragments_accepted);
    TEST_ASSERT_EQUAL_UINT32(0U, stats.dropped_reassembly);
}

void test_fragmented_message_reassembles_in_order() {
    BtpEndpoint endpoint;
    configure(endpoint, 0x0C30AA5CU, 0x10203040U);
    RxRouter::Router router;
    RxRouter::RoutedMessage routed{};

    std::uint8_t logical[kLogicalSize];
    fill_pattern(logical, sizeof(logical), 0x11U);
    const Encoded first =
        encode_fragment(endpoint, 7U, logical, sizeof(logical), 0U, 2U);
    const Encoded second =
        encode_fragment(endpoint, 7U, logical, sizeof(logical), 1U, 2U);

    TEST_ASSERT_EQUAL(static_cast<int>(RxRouter::Outcome::FragmentAccepted),
                      static_cast<int>(submit(router, first, 100U, &routed)));
    TEST_ASSERT_EQUAL(static_cast<int>(RxRouter::Outcome::Routed),
                      static_cast<int>(submit(router, second, 200U, &routed)));

    TEST_ASSERT_TRUE(routed.reassembled);
    TEST_ASSERT_EQUAL_UINT32(sizeof(logical), routed.payload_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(logical, routed.payload, sizeof(logical));

    // Completion normalizes the header, so the consumer cannot tell how the
    // message travelled -- which is the whole point of putting reassembly
    // ahead of routing.
    TEST_ASSERT_EQUAL_UINT16(0U, routed.header.flags & btp::kFlagFragmented);
    TEST_ASSERT_EQUAL_UINT8(0U, routed.header.fragment_index);
    TEST_ASSERT_EQUAL_UINT8(1U, routed.header.fragment_count);
    TEST_ASSERT_EQUAL_HEX32(0x0C30AA5CU, routed.header.source_id);
    TEST_ASSERT_EQUAL_UINT32(7U, routed.header.sequence);
    TEST_ASSERT_EQUAL_HEX16(kTestObjectId, routed.header.object_id);
}

void test_fragmented_message_reassembles_out_of_order() {
    BtpEndpoint endpoint;
    configure(endpoint, 0x0C30AA5CU, 0x10203040U);
    RxRouter::Router router;
    RxRouter::RoutedMessage routed{};

    std::uint8_t logical[kLogicalSize];
    fill_pattern(logical, sizeof(logical), 0x55U);
    const Encoded first =
        encode_fragment(endpoint, 8U, logical, sizeof(logical), 0U, 2U);
    const Encoded second =
        encode_fragment(endpoint, 8U, logical, sizeof(logical), 1U, 2U);

    // Last fragment first: the offset of fragment 0 is not known when it is
    // stored, so a reassembler that appended instead of inserting would
    // produce the two halves swapped.
    TEST_ASSERT_EQUAL(static_cast<int>(RxRouter::Outcome::FragmentAccepted),
                      static_cast<int>(submit(router, second, 100U, &routed)));
    TEST_ASSERT_EQUAL(static_cast<int>(RxRouter::Outcome::Routed),
                      static_cast<int>(submit(router, first, 150U, &routed)));

    TEST_ASSERT_TRUE(routed.reassembled);
    TEST_ASSERT_EQUAL_UINT32(sizeof(logical), routed.payload_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(logical, routed.payload, sizeof(logical));
}

// Canonical scenario "two_sources_interleaved_out_of_order" from BTP's
// test-vectors manifest. This is the case the hub makes ordinary: TraceView
// and the dongle both fragmenting toward this robot, arriving interleaved,
// each keyed by its own (source_id, boot_id, sequence).
void test_two_sources_interleaved_reassemble_independently() {
    const std::vector<std::uint8_t> a0 =
        read_vector("valid/fragment_source_a_0.bin");
    const std::vector<std::uint8_t> a1 =
        read_vector("valid/fragment_source_a_1.bin");
    const std::vector<std::uint8_t> b0 =
        read_vector("valid/fragment_source_b_0.bin");
    const std::vector<std::uint8_t> b1 =
        read_vector("valid/fragment_source_b_1.bin");
    TEST_ASSERT_FALSE(a0.empty());
    TEST_ASSERT_FALSE(a1.empty());
    TEST_ASSERT_FALSE(b0.empty());
    TEST_ASSERT_FALSE(b1.empty());

    RxRouter::Router router;
    RxRouter::RoutedMessage routed{};

    // arrival order: a_1, b_0, a_0, b_1
    TEST_ASSERT_EQUAL(
        static_cast<int>(RxRouter::Outcome::FragmentAccepted),
        static_cast<int>(router.submit(a1.data(), a1.size(), 1U, &routed)));
    TEST_ASSERT_EQUAL(
        static_cast<int>(RxRouter::Outcome::FragmentAccepted),
        static_cast<int>(router.submit(b0.data(), b0.size(), 2U, &routed)));

    TEST_ASSERT_EQUAL(
        static_cast<int>(RxRouter::Outcome::Routed),
        static_cast<int>(router.submit(a0.data(), a0.size(), 3U, &routed)));
    const std::uint8_t expected_a[] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15};
    TEST_ASSERT_EQUAL_HEX32(0xAAA00001U, routed.header.source_id);
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_a), routed.payload_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_a, routed.payload,
                                  sizeof(expected_a));

    TEST_ASSERT_EQUAL(
        static_cast<int>(RxRouter::Outcome::Routed),
        static_cast<int>(router.submit(b1.data(), b1.size(), 4U, &routed)));
    const std::uint8_t expected_b[] = {0xA0, 0xA1, 0xA2, 0xA3,
                                       0xA4, 0xA5, 0xA6, 0xA7};
    TEST_ASSERT_EQUAL_HEX32(0xBBB00002U, routed.header.source_id);
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected_b), routed.payload_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_b, routed.payload,
                                  sizeof(expected_b));

    const RxRouter::Stats stats = router.stats();
    TEST_ASSERT_EQUAL_UINT32(2U, stats.routed);
    TEST_ASSERT_EQUAL_UINT32(2U, stats.fragments_accepted);
    TEST_ASSERT_EQUAL_UINT32(0U, stats.dropped_reassembly);
}

// A retransmitted fragment is ordinary on a lossy radio. It must be absorbed,
// not counted as progress and not written twice into the slot.
void test_duplicate_fragment_is_absorbed_without_corruption() {
    BtpEndpoint endpoint;
    configure(endpoint, 0x0C30AA5CU, 0x10203040U);
    RxRouter::Router router;
    RxRouter::RoutedMessage routed{};

    std::uint8_t logical[kLogicalSize];
    fill_pattern(logical, sizeof(logical), 0x77U);
    const Encoded first =
        encode_fragment(endpoint, 9U, logical, sizeof(logical), 0U, 2U);
    const Encoded second =
        encode_fragment(endpoint, 9U, logical, sizeof(logical), 1U, 2U);

    TEST_ASSERT_EQUAL(static_cast<int>(RxRouter::Outcome::FragmentAccepted),
                      static_cast<int>(submit(router, first, 100U, &routed)));
    TEST_ASSERT_EQUAL(static_cast<int>(RxRouter::Outcome::DuplicateFragment),
                      static_cast<int>(submit(router, first, 110U, &routed)));
    TEST_ASSERT_EQUAL(static_cast<int>(RxRouter::Outcome::Routed),
                      static_cast<int>(submit(router, second, 120U, &routed)));

    TEST_ASSERT_EQUAL_UINT32(sizeof(logical), routed.payload_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(logical, routed.payload, sizeof(logical));

    const RxRouter::Stats stats = router.stats();
    TEST_ASSERT_EQUAL_UINT32(1U, stats.duplicate_fragments);
    TEST_ASSERT_EQUAL_UINT32(1U, stats.routed);
    TEST_ASSERT_EQUAL_UINT32(0U, stats.dropped_reassembly);
}

// The slot pool is only bounded if abandoned reassemblies actually leave it.
// Nothing else in the firmware sweeps them, so expire() is what publishStatus
// calls once a second.
void test_incomplete_reassembly_expires_after_four_seconds() {
    BtpEndpoint endpoint;
    configure(endpoint, 0x0C30AA5CU, 0x10203040U);
    RxRouter::Router router;
    RxRouter::RoutedMessage routed{};

    std::uint8_t logical[kLogicalSize];
    fill_pattern(logical, sizeof(logical), 0x22U);
    const Encoded first =
        encode_fragment(endpoint, 11U, logical, sizeof(logical), 0U, 2U);
    const Encoded second =
        encode_fragment(endpoint, 11U, logical, sizeof(logical), 1U, 2U);

    const std::uint64_t arrival = 1000U;
    TEST_ASSERT_EQUAL(
        static_cast<int>(RxRouter::Outcome::FragmentAccepted),
        static_cast<int>(submit(router, first, arrival, &routed)));

    // One millisecond short of the timeout the slot is still holding on.
    TEST_ASSERT_EQUAL_UINT32(
        0U, router.expireForTest(arrival + RxRouter::kReassemblyTimeoutMs - 1U));
    TEST_ASSERT_EQUAL_UINT32(
        1U, router.expireForTest(arrival + RxRouter::kReassemblyTimeoutMs));
    TEST_ASSERT_EQUAL_UINT32(
        0U, router.expireForTest(arrival + RxRouter::kReassemblyTimeoutMs));

    // The message is genuinely gone: its second half now opens a NEW
    // reassembly instead of completing the old one.
    TEST_ASSERT_EQUAL(
        static_cast<int>(RxRouter::Outcome::FragmentAccepted),
        static_cast<int>(submit(router, second, arrival + 5000U, &routed)));

    // And the freed slot is usable -- all four are available again once that
    // one is expired too.
    TEST_ASSERT_EQUAL_UINT32(1U, router.expireForTest(arrival + 20000U));
    for (std::uint32_t sequence = 20U; sequence < 20U + RxRouter::kSlotCount;
         ++sequence) {
        const Encoded fragment = encode_fragment(
            endpoint, sequence, logical, sizeof(logical), 0U, 2U);
        const std::uint64_t later = arrival + 20000U;
        TEST_ASSERT_EQUAL(
            static_cast<int>(RxRouter::Outcome::FragmentAccepted),
            static_cast<int>(submit(router, fragment, later, &routed)));
    }
}

// Four slots is a hard bound, so the fifth concurrent sender has to be told
// no. What must NOT happen is the fifth evicting one of the four: a peer that
// opens reassemblies and never finishes them would otherwise be able to
// destroy everyone else's traffic.
void test_fifth_concurrent_reassembly_is_rejected_not_evicting() {
    BtpEndpoint endpoint;
    configure(endpoint, 0x0C30AA5CU, 0x10203040U);
    RxRouter::Router router;
    RxRouter::RoutedMessage routed{};

    std::uint8_t logical[RxRouter::kSlotCount][kLogicalSize];
    Encoded first[RxRouter::kSlotCount];
    Encoded second[RxRouter::kSlotCount];
    const std::uint64_t now = 500U;

    for (std::size_t slot = 0U; slot < RxRouter::kSlotCount; ++slot) {
        fill_pattern(logical[slot], kLogicalSize,
                     static_cast<std::uint8_t>(0xA0U + slot));
        const std::uint32_t sequence = static_cast<std::uint32_t>(slot + 1U);
        first[slot] = encode_fragment(endpoint, sequence, logical[slot],
                                      kLogicalSize, 0U, 2U);
        second[slot] = encode_fragment(endpoint, sequence, logical[slot],
                                       kLogicalSize, 1U, 2U);
        TEST_ASSERT_EQUAL(
            static_cast<int>(RxRouter::Outcome::FragmentAccepted),
            static_cast<int>(submit(router, first[slot], now, &routed)));
    }

    // Same instant, so nothing above has expired: every slot is genuinely
    // busy and the reassembler has nowhere to put this.
    std::uint8_t overflow_payload[kLogicalSize];
    fill_pattern(overflow_payload, sizeof(overflow_payload), 0x5AU);
    const Encoded overflow = encode_fragment(
        endpoint, 99U, overflow_payload, sizeof(overflow_payload), 0U, 2U);
    TEST_ASSERT_EQUAL(static_cast<int>(RxRouter::Outcome::DroppedReassembly),
                      static_cast<int>(submit(router, overflow, now, &routed)));
    TEST_ASSERT_EQUAL_UINT32(1U, router.stats().dropped_reassembly);

    // The four in progress finish intact, with their own bytes.
    for (std::size_t slot = 0U; slot < RxRouter::kSlotCount; ++slot) {
        TEST_ASSERT_EQUAL(
            static_cast<int>(RxRouter::Outcome::Routed),
            static_cast<int>(submit(router, second[slot], now, &routed)));
        TEST_ASSERT_TRUE(routed.reassembled);
        TEST_ASSERT_EQUAL_UINT32(kLogicalSize, routed.payload_size);
        TEST_ASSERT_EQUAL_UINT8_ARRAY(logical[slot], routed.payload,
                                      kLogicalSize);
    }

    const RxRouter::Stats stats = router.stats();
    TEST_ASSERT_EQUAL_UINT32(RxRouter::kSlotCount, stats.routed);
    TEST_ASSERT_EQUAL_UINT32(RxRouter::kSlotCount, stats.fragments_accepted);
    TEST_ASSERT_EQUAL_UINT32(1U, stats.dropped_reassembly);

    // Rejecting the fifth also released nothing early: with the four now
    // complete and released, a fresh reassembly gets a slot.
    TEST_ASSERT_EQUAL(static_cast<int>(RxRouter::Outcome::FragmentAccepted),
                      static_cast<int>(submit(router, overflow, now, &routed)));
}

// A corrupt frame must never reach the routing stage, and CRC damage must
// stay distinguishable from a peer speaking a dialect we do not know.
// The property that removed a two-task data race: submit() sweeps and counts
// abandoned slots ITSELF, so nothing outside the receive context ever has to
// touch the slot table.
//
// Before this, the robot swept from its once-a-second routine task while the
// ESP-NOW callback pushed fragments -- two writers on one slot table with
// nothing between them. The sweep turned out never to be load-bearing
// (btp::Reassembler::push() expires stale slots on its own), so it existed
// only to count losses for STATUS. Moving that counting into submit() removed
// the second writer without adding a lock.
//
// What this asserts is exactly that: with NO external sweep anywhere, slots
// abandoned past the timeout are still reclaimed, still counted, and the
// reclaimed slot still works.
void test_submit_reclaims_and_counts_abandoned_slots_with_no_external_sweep() {
    BtpEndpoint endpoint;
    configure(endpoint, 0x0C30BB6DU, 0x10203040U);
    RxRouter::Router router;
    RxRouter::RoutedMessage routed{};

    std::uint8_t logical[kLogicalSize];
    fill_pattern(logical, sizeof(logical), 0x44U);

    const std::uint64_t arrival = 1000U;

    // Fill every slot with a first fragment whose second half never arrives.
    // Distinct sequences make them distinct messages, so each takes a slot.
    for (std::uint32_t i = 0U; i < RxRouter::kSlotCount; ++i) {
        const Encoded first =
            encode_fragment(endpoint, 40U + i, logical, sizeof(logical), 0U, 2U);
        TEST_ASSERT_EQUAL(
            static_cast<int>(RxRouter::Outcome::FragmentAccepted),
            static_cast<int>(submit(router, first, arrival, &routed)));
    }
    TEST_ASSERT_EQUAL_UINT32(0U, router.stats().reassembly_timeouts);

    // One more arrival, past the timeout, and with expireForTest() called
    // nowhere: submit() has to reclaim the stale slots on its own to have room.
    const std::uint64_t late = arrival + RxRouter::kReassemblyTimeoutMs + 1U;
    const Encoded freshFirst =
        encode_fragment(endpoint, 99U, logical, sizeof(logical), 0U, 2U);
    TEST_ASSERT_EQUAL(
        static_cast<int>(RxRouter::Outcome::FragmentAccepted),
        static_cast<int>(submit(router, freshFirst, late, &routed)));

    // Every loss counted, and counted once.
    TEST_ASSERT_EQUAL_UINT32(RxRouter::kSlotCount, router.stats().reassembly_timeouts);

    // The reassembly that took a reclaimed slot still completes intact -- the
    // sweep freed the abandoned slots without disturbing the live one.
    const Encoded freshSecond =
        encode_fragment(endpoint, 99U, logical, sizeof(logical), 1U, 2U);
    TEST_ASSERT_EQUAL(
        static_cast<int>(RxRouter::Outcome::Routed),
        static_cast<int>(submit(router, freshSecond, late, &routed)));
    TEST_ASSERT_EQUAL_UINT32(kLogicalSize, routed.payload_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(logical, routed.payload, kLogicalSize);
    TEST_ASSERT_EQUAL_UINT32(RxRouter::kSlotCount, router.stats().reassembly_timeouts);
}

void test_crc_and_decode_failures_are_counted_apart() {
    const std::vector<std::uint8_t> bad_crc = read_vector("invalid/crc.bin");
    const std::vector<std::uint8_t> bad_magic =
        read_vector("invalid/magic.bin");
    TEST_ASSERT_FALSE(bad_crc.empty());
    TEST_ASSERT_FALSE(bad_magic.empty());

    RxRouter::Router router;
    RxRouter::RoutedMessage routed{};
    TEST_ASSERT_EQUAL(static_cast<int>(RxRouter::Outcome::DroppedCrc),
                      static_cast<int>(router.submit(bad_crc.data(),
                                                     bad_crc.size(), 1U,
                                                     &routed)));
    TEST_ASSERT_EQUAL(static_cast<int>(RxRouter::Outcome::DroppedDecode),
                      static_cast<int>(router.submit(bad_magic.data(),
                                                     bad_magic.size(), 1U,
                                                     &routed)));
    TEST_ASSERT_EQUAL(
        static_cast<int>(RxRouter::Outcome::DroppedInvalidArgument),
        static_cast<int>(router.submit(nullptr, 10U, 1U, &routed)));

    const RxRouter::Stats stats = router.stats();
    TEST_ASSERT_EQUAL_UINT32(1U, stats.dropped_crc);
    TEST_ASSERT_EQUAL_UINT32(1U, stats.dropped_decode);
    TEST_ASSERT_EQUAL_UINT32(1U, stats.dropped_invalid_argument);
    TEST_ASSERT_EQUAL_UINT32(0U, stats.routed);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_unfragmented_message_crosses_without_touching_a_slot);
    RUN_TEST(test_fragmented_message_reassembles_in_order);
    RUN_TEST(test_fragmented_message_reassembles_out_of_order);
    RUN_TEST(test_two_sources_interleaved_reassemble_independently);
    RUN_TEST(test_duplicate_fragment_is_absorbed_without_corruption);
    RUN_TEST(test_incomplete_reassembly_expires_after_four_seconds);
    RUN_TEST(test_fifth_concurrent_reassembly_is_rejected_not_evicting);
    RUN_TEST(test_crc_and_decode_failures_are_counted_apart);
    RUN_TEST(test_submit_reclaims_and_counts_abandoned_slots_with_no_external_sweep);
    return UNITY_END();
}
