#include <unity.h>

#include <StreamFraming.h>

#include <btp/codec.hpp>
#include <btp/endpoint.hpp>
#include <btp/messages.hpp>
#include <btp/node.hpp>
#include <btp/session.hpp>

#include <cstdint>
#include <cstring>
#include <vector>

// A direct TCP / BLE link is a COBS-framed byte stream, not one frame per
// read (BTP/docs/fragmentation-and-transports.md sections 8.2 and 9) -- the
// same framing TraceView applies on both. This suite pins the robot's half:
// every outgoing frame is wrapped as 0x00 || COBS(frame) || 0x00, and any
// chunking of the incoming stream -- a byte at a time, several frames at
// once, noise before the first delimiter -- yields the same frames.

namespace {

using Frames = std::vector<std::vector<std::uint8_t>>;

bool capture(void* ctx, const std::uint8_t* frame, std::size_t size) {
    static_cast<Frames*>(ctx)->emplace_back(frame, frame + size);
    return true;
}

// One real BTP frame (header + payload + CRC), as btp::Endpoint builds it.
std::vector<std::uint8_t> make_frame(std::uint32_t source_id, std::uint16_t object_id,
                                     std::size_t payload_size, std::uint8_t seed) {
    btp::Endpoint endpoint;
    TEST_ASSERT_TRUE(endpoint.configure(source_id, 0x0000B007U));
    std::vector<std::uint8_t> payload(payload_size);
    for (std::size_t i = 0; i < payload_size; ++i) {
        payload[i] = static_cast<std::uint8_t>(seed + i);  // includes 0x00 bytes
    }
    const btp::LogicalMessage message{btp::MessageType::Log, object_id, 1ULL,
                                      {payload.data(), payload.size()}};
    Frames out;
    TEST_ASSERT_TRUE(endpoint.send_logical(message, btp::kEspNowTransport, &capture,
                                           &out, nullptr, 0U));
    TEST_ASSERT_EQUAL_UINT32(1U, out.size());
    return out[0];
}

std::vector<std::uint8_t> wire_of(const std::vector<std::uint8_t>& frame) {
    std::uint8_t wire[stream_framing::kMaxWireSize];
    std::size_t written = 0U;
    TEST_ASSERT_TRUE(stream_framing::encode_stream_frame(frame.data(), frame.size(), wire,
                                                         sizeof(wire), &written));
    return std::vector<std::uint8_t>(wire, wire + written);
}

struct Received {
    std::vector<btp::Header> headers;
    std::vector<std::vector<std::uint8_t>> payloads;
};

void collect(void* ctx, const btp::DecodedFrame& frame) {
    Received* r = static_cast<Received*>(ctx);
    r->headers.push_back(frame.header);
    r->payloads.emplace_back(frame.payload.data, frame.payload.data + frame.payload.size);
}

// ~8.2 KB each: kept off the (small, native-test) stack.
StreamDeframer g_deframer;

void test_wire_is_delimited_and_has_no_inner_zero() {
    const std::vector<std::uint8_t> frame = make_frame(0x00CAFE01U, 0x0001U, 40U, 0x00U);
    const std::vector<std::uint8_t> wire = wire_of(frame);
    TEST_ASSERT_EQUAL_HEX8(0x00U, wire.front());
    TEST_ASSERT_EQUAL_HEX8(0x00U, wire.back());
    for (std::size_t i = 1U; i + 1U < wire.size(); ++i) {
        TEST_ASSERT_NOT_EQUAL(0x00U, wire[i]);
    }
}

void test_round_trip_one_byte_at_a_time() {
    const std::vector<std::uint8_t> frame = make_frame(0x00CAFE01U, 0x0042U, 60U, 0xF0U);
    const std::vector<std::uint8_t> wire = wire_of(frame);
    g_deframer.reset();
    Received got;
    for (std::uint8_t byte : wire) g_deframer.feed(&byte, 1U, &collect, &got);
    TEST_ASSERT_EQUAL_UINT32(1U, got.headers.size());
    TEST_ASSERT_EQUAL_HEX16(0x0042U, got.headers[0].object_id);
    TEST_ASSERT_EQUAL_UINT32(60U, got.payloads[0].size());
    TEST_ASSERT_EQUAL_HEX8(0xF0U, got.payloads[0][0]);
    TEST_ASSERT_EQUAL_HEX8(0x00U, got.payloads[0][16]);  // 0xF0 + 16 wraps to 0x00
}

// Noise before the first delimiter (a fresh connection starts mid-nothing,
// section 6.3) is discarded; two frames in one read both come out, in order.
void test_noise_then_two_frames_in_one_chunk() {
    std::vector<std::uint8_t> stream = {0x11U, 0x22U, 0x33U};
    const std::vector<std::uint8_t> a = wire_of(make_frame(0x00CAFE01U, 0x0001U, 10U, 1U));
    const std::vector<std::uint8_t> b = wire_of(make_frame(0x00CAFE01U, 0x0002U, 20U, 2U));
    stream.insert(stream.end(), a.begin(), a.end());
    stream.insert(stream.end(), b.begin(), b.end());

    g_deframer.reset();
    Received got;
    TEST_ASSERT_EQUAL_UINT32(2U, g_deframer.feed(stream.data(), stream.size(), &collect, &got));
    TEST_ASSERT_EQUAL_HEX16(0x0001U, got.headers[0].object_id);
    TEST_ASSERT_EQUAL_HEX16(0x0002U, got.headers[1].object_id);
    TEST_ASSERT_EQUAL_UINT32(2U, g_deframer.stats().frames);
}

// A corrupted block is counted and dropped; the stream resynchronizes on the
// next delimiter and the following frame still arrives.
void test_corrupted_frame_is_dropped_and_stream_recovers() {
    std::vector<std::uint8_t> bad = wire_of(make_frame(0x00CAFE01U, 0x0001U, 30U, 7U));
    bad[bad.size() / 2U] ^= 0x5AU;  // flip bits mid-block: CRC (or COBS) fails
    const std::vector<std::uint8_t> good = wire_of(make_frame(0x00CAFE01U, 0x0003U, 5U, 9U));
    std::vector<std::uint8_t> stream = bad;
    stream.insert(stream.end(), good.begin(), good.end());

    g_deframer.reset();
    Received got;
    TEST_ASSERT_EQUAL_UINT32(1U, g_deframer.feed(stream.data(), stream.size(), &collect, &got));
    TEST_ASSERT_EQUAL_HEX16(0x0003U, got.headers[0].object_id);
    TEST_ASSERT_EQUAL_UINT32(1U, g_deframer.stats().cobs_errors + g_deframer.stats().frame_errors);
}

void test_encode_refuses_what_it_cannot_frame() {
    const std::uint8_t one = 0x01U;
    std::uint8_t out[stream_framing::kMaxWireSize];
    std::size_t written = 0U;
    TEST_ASSERT_FALSE(stream_framing::encode_stream_frame(&one, 0U, out, sizeof(out), &written));
    TEST_ASSERT_FALSE(stream_framing::encode_stream_frame(nullptr, 1U, out, sizeof(out), &written));
    const std::vector<std::uint8_t> frame = make_frame(0x00CAFE01U, 0x0001U, 100U, 3U);
    TEST_ASSERT_FALSE(stream_framing::encode_stream_frame(frame.data(), frame.size(), out, 50U,
                                                          &written));
}

// ---------------------------------------------------------------------------
// End to end, the way ROBOT wires a direct link: a TraceView-like peer sends
// a COBS-framed HELLO; the deframer hands the frame to node.receive_on(link)
// (session enabled on that link only); the HELLO_RESULT the link's send()
// produces is COBS-framed on the way out and decodes back on the peer side.
// ---------------------------------------------------------------------------

struct WireLink : public btp::NodeLink {
    std::vector<std::uint8_t> wire;
    WireLink() { transport = btp::kTcpTransport; }
    bool send(const std::uint8_t* frame, std::size_t size) override {
        std::uint8_t out[stream_framing::kMaxWireSize];
        std::size_t written = 0U;
        if (!stream_framing::encode_stream_frame(frame, size, out, sizeof(out), &written)) {
            return false;
        }
        wire.insert(wire.end(), out, out + written);
        return true;
    }
};

struct RadioConfig : public btp::NodeConfig {
    RadioConfig() {
        source_id = 0x00CAFE01U;
        boot_id = 0x0000B007U;
        transport = btp::kEspNowTransport;
    }
    bool send(const std::uint8_t*, std::size_t) override { return true; }
};

using TwoLinkNode = btp::StaticNode<4, 600, 1024, 512, 8, 64, 1536, 8, 4, 128, 0, 2>;

btp::Hello hello(btp::Role role) {
    std::uint8_t uuid[16];
    for (int i = 0; i < 16; ++i) uuid[i] = static_cast<std::uint8_t>(i + 1);
    return btp::HelloBuilder(role, uuid).build();
}

struct NodeFeed {
    TwoLinkNode* node;
    btp::NodeRx last;
};

void feed_node(void* ctx, const btp::DecodedFrame& frame) {
    NodeFeed* f = static_cast<NodeFeed*>(ctx);
    btp::ReceivedMessage msg{};
    f->last = f->node->receive_on(1U, frame, 10U, &msg);
}

RadioConfig g_radio;
WireLink g_tcp;
TwoLinkNode g_node(g_radio);

void test_cobs_hello_opens_the_session_and_the_reply_is_cobs_framed() {
    TEST_ASSERT_TRUE(g_node.attach_link(1U, g_tcp, 5000U));
    g_node.enable_session_on(1U, hello(btp::Role::Producer), 2000U);
    TEST_ASSERT_TRUE(g_node.begin());
    g_node.arm_session_on(1U, 0U);

    std::uint8_t body[64];
    std::size_t body_size = 0U;
    TEST_ASSERT_EQUAL(btp::MessageError::Ok,
                      btp::encode_hello(hello(btp::Role::Consumer), body, sizeof(body),
                                        &body_size));
    btp::Endpoint peer;
    TEST_ASSERT_TRUE(peer.configure(0x00B0B0FEU, 0x0000C0DEU));
    const btp::LogicalMessage msg{btp::MessageType::Control, btp::object_id::kHello, 1ULL,
                                  {body, body_size}};
    Frames frames;
    TEST_ASSERT_TRUE(peer.send_logical(msg, btp::kTcpTransport, &capture, &frames, nullptr, 0U));
    TEST_ASSERT_EQUAL_UINT32(1U, frames.size());
    const std::vector<std::uint8_t> wire = wire_of(frames[0]);

    g_deframer.reset();
    NodeFeed feed{&g_node, btp::NodeRx::Pending};
    // Split mid-block, as a socket read may.
    const std::size_t half = wire.size() / 2U;
    g_deframer.feed(wire.data(), half, &feed_node, &feed);
    g_deframer.feed(wire.data() + half, wire.size() - half, &feed_node, &feed);
    TEST_ASSERT_EQUAL(btp::NodeRx::SessionHandled, feed.last);
    TEST_ASSERT_TRUE(g_node.link_session(1U)->active());

    // The HELLO_RESULT left through the link, COBS-framed: the peer's own
    // deframer gets it back as one frame.
    TEST_ASSERT_TRUE(g_tcp.wire.size() > 2U);
    TEST_ASSERT_EQUAL_HEX8(0x00U, g_tcp.wire.front());
    g_deframer.reset();
    Received got;
    TEST_ASSERT_EQUAL_UINT32(1U, g_deframer.feed(g_tcp.wire.data(), g_tcp.wire.size(),
                                                 &collect, &got));
    TEST_ASSERT_EQUAL_HEX16(btp::object_id::kHelloResult, got.headers[0].object_id);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_wire_is_delimited_and_has_no_inner_zero);
    RUN_TEST(test_round_trip_one_byte_at_a_time);
    RUN_TEST(test_noise_then_two_frames_in_one_chunk);
    RUN_TEST(test_corrupted_frame_is_dropped_and_stream_recovers);
    RUN_TEST(test_encode_refuses_what_it_cannot_frame);
    RUN_TEST(test_cobs_hello_opens_the_session_and_the_reply_is_cobs_framed);
    return UNITY_END();
}
