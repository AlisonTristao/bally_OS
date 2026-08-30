#include <unity.h>

#include <BtpTransport.h>
#include <TerminalResponder.h>
#include <btp/codec.hpp>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// TerminalResponder is pure C++ (same shape as SubscriptionResponder), so it
// links into env:native. This suite drives it the way the robot does --
// on_terminal_in() (RX task) buffers bytes, pump() (comms task) runs the
// editor and emits TERMINAL_OUT, deliver_command_output() (shell task) hands
// back a command's output -- and checks the wire bytes it produces plus the
// per-origin pool behaviour.

namespace {

constexpr std::uint32_t kRobotSourceId = 0x11111111U;
constexpr std::uint32_t kRobotBootId = 0x22222222U;

std::vector<std::string> g_sent;  // decoded TERMINAL_OUT payloads, in order

bool capture_send(const std::uint8_t* data, std::size_t size) {
    btp::DecodedFrame frame{};
    if (btp::decode(data, size, btp::TransportProfile::EspNow, &frame) != btp::Error::Ok) {
        return false;
    }
    if (frame.header.type != btp::MessageType::Terminal ||
        frame.header.object_id != TerminalResponder::kTerminalOutObjectId) {
        return false;
    }
    g_sent.emplace_back(reinterpret_cast<const char*>(frame.payload.data), frame.payload.size);
    return true;
}

std::string all_sent() {
    std::string out;
    for (const std::string& s : g_sent) out += s;
    return out;
}

// --- submit stub -------------------------------------------------------------
struct SubmitLog {
    std::vector<std::string> lines;
    std::vector<std::uint32_t> src;
    bool accept = true;
};
SubmitLog g_submit;

bool submit_stub(void* ctx, std::uint32_t source_id, std::uint32_t boot_id, const char* line) {
    (void)ctx;
    (void)boot_id;
    if (!g_submit.accept) return false;
    g_submit.lines.emplace_back(line);
    g_submit.src.push_back(source_id);
    return true;
}

// --- fixture --------------------------------------------------------------
// BtpEndpoint and TerminalResponder hold std::atomic members (not movable),
// so they are heap objects rebuilt per test rather than reassigned.
BtpEndpoint* g_endpoint = nullptr;
TerminalResponder* g_responder = nullptr;
std::uint64_t g_now = 1000U;

void reset_fixture() {
    g_sent.clear();
    g_submit = SubmitLog{};
    g_now = 1000U;

    delete g_responder;
    delete g_endpoint;
    g_endpoint = new BtpEndpoint();
    g_responder = new TerminalResponder();

    TEST_ASSERT_TRUE(g_endpoint->configure(kRobotSourceId, kRobotBootId));
    g_endpoint->set_send_callback(capture_send);
    g_responder->configure(
        *g_endpoint, /*seal=*/nullptr, /*seal_context=*/nullptr,
        [](const std::string& input, std::string* out, std::size_t max_out) -> std::size_t {
            if (input == "he" && max_out > 0U) {
                out[0] = "help ";
                return 1U;
            }
            return 0U;
        },
        submit_stub, nullptr, "bally> ");
}

btp::Header origin_header(std::uint32_t source_id, std::uint32_t boot_id) {
    btp::Header h{};
    h.type = btp::MessageType::Terminal;
    h.source_id = source_id;
    h.boot_id = boot_id;
    h.object_id = TerminalResponder::kTerminalInObjectId;
    h.timestamp_us = g_now;
    h.fragment_count = 1U;
    return h;
}

void feed(std::uint32_t source_id, std::uint32_t boot_id, const std::string& bytes) {
    const btp::Header h = origin_header(source_id, boot_id);
    g_responder->on_terminal_in(h, btp::ByteView{reinterpret_cast<const std::uint8_t*>(bytes.data()),
                                                bytes.size()});
}

void pump() {
    g_responder->pump(g_now);
    g_now += 10U;
}

}  // namespace

void setUp() { reset_fixture(); }
void tearDown() {}

// The first TERMINAL_IN from an origin paints the prompt, then echoes.
void test_first_contact_paints_prompt_then_echoes() {
    feed(0xAAAA0001U, 0xBBBB0001U, "x");
    pump();
    const std::string out = all_sent();
    TEST_ASSERT_TRUE(out.find("bally> ") != std::string::npos);
    TEST_ASSERT_TRUE(out.rfind('x') != std::string::npos);
}

// Typed characters are echoed back in TERMINAL_OUT.
void test_keystrokes_are_echoed() {
    feed(0xAAAA0002U, 0xBBBB0002U, "ls");
    pump();
    TEST_ASSERT_TRUE(all_sent().find("ls") != std::string::npos);
}

// A completed line is submitted and its output mirrored back inline.
void test_line_submits_and_output_mirrors_back() {
    const std::uint32_t src = 0xAAAA0002U;
    const std::uint32_t boot = 0xBBBB0002U;

    feed(src, boot, "info\r");
    pump();
    TEST_ASSERT_EQUAL(1U, g_submit.lines.size());
    TEST_ASSERT_EQUAL_STRING("info", g_submit.lines[0].c_str());
    TEST_ASSERT_EQUAL_UINT32(src, g_submit.src[0]);

    // The origin is now awaiting the result: another line must not be submitted.
    feed(src, boot, "link -stats\r");
    pump();
    TEST_ASSERT_EQUAL(1U, g_submit.lines.size());

    // Deliver the command's captured output. The next pump() mirrors it back
    // AND, now that the origin is no longer awaiting, releases the held line.
    g_responder->deliver_command_output(src, boot, "armed=1\npwm_range=-100..100", 0U);
    g_sent.clear();
    pump();
    const std::string out = all_sent();
    TEST_ASSERT_TRUE(out.find("armed=1") != std::string::npos);
    TEST_ASSERT_TRUE(out.find("pwm_range=-100..100") != std::string::npos);
    TEST_ASSERT_TRUE(out.find("\r\n") != std::string::npos);  // CR+LF normalised

    TEST_ASSERT_EQUAL(2U, g_submit.lines.size());
    TEST_ASSERT_EQUAL_STRING("link -stats", g_submit.lines[1].c_str());
}

// Tab completion runs against the configured provider.
void test_tab_completion() {
    feed(0xAAAA0003U, 0xBBBB0003U, "he\t");
    pump();
    TEST_ASSERT_TRUE(all_sent().find("help ") != std::string::npos);
}

// A non-zero shell status is reported as a trailing note.
void test_nonzero_status_is_reported() {
    const std::uint32_t src = 0xAAAA0004U;
    feed(src, 0xBBBB0004U, "bogus\r");
    pump();
    g_responder->deliver_command_output(src, 0xBBBB0004U, "unknown command", 255U);
    g_sent.clear();
    pump();
    TEST_ASSERT_TRUE(all_sent().find("status=255") != std::string::npos);
}

// A 4th distinct origin evicts the least-recently-used slot; a still-live
// origin keeps its own editing state (its input buffer is not corrupted).
void test_pool_evicts_lru_and_isolates_origins() {
    // Three origins, each types a partial line.
    feed(1U, 1U, "aaa");
    feed(2U, 2U, "bbb");
    feed(3U, 3U, "ccc");
    pump();
    TEST_ASSERT_EQUAL(3U, g_responder->stats().origins_seen);

    // Touch origin 2 and 3 so origin 1 is the LRU.
    feed(2U, 2U, "");
    feed(3U, 3U, "");
    pump();

    // 4th origin: evicts origin 1.
    feed(4U, 4U, "ddd\r");
    pump();
    TEST_ASSERT_EQUAL(4U, g_responder->stats().origins_seen);
    TEST_ASSERT_EQUAL(1U, g_responder->stats().origins_evicted);
    TEST_ASSERT_EQUAL(1U, g_submit.lines.size());
    TEST_ASSERT_EQUAL_STRING("ddd", g_submit.lines[0].c_str());

    // Origin 2 completes its line -- "bbb" plus new "x" -> "bbbx", proving its
    // buffer survived origin 4's allocation.
    feed(2U, 2U, "x\r");
    pump();
    bool found = false;
    for (const std::string& l : g_submit.lines) {
        if (l == "bbbx") found = true;
    }
    TEST_ASSERT_TRUE(found);
}

// Back-to-back commands from one origin: the second must run and mirror its
// output too. Regression for "the terminal works exactly once" -- a command's
// result left the origin unable to submit the next line.
void test_consecutive_commands_each_run() {
    const std::uint32_t src = 0xAAAA0006U;
    const std::uint32_t boot = 0xBBBB0006U;

    feed(src, boot, "one\r");
    pump();
    TEST_ASSERT_EQUAL(1U, g_submit.lines.size());
    g_responder->deliver_command_output(src, boot, "first output", 0U);
    pump();

    g_sent.clear();
    feed(src, boot, "two\r");
    pump();
    TEST_ASSERT_EQUAL(2U, g_submit.lines.size());
    TEST_ASSERT_EQUAL_STRING("two", g_submit.lines[1].c_str());
    g_responder->deliver_command_output(src, boot, "second output", 0U);
    pump();
    TEST_ASSERT_TRUE(all_sent().find("second output") != std::string::npos);
}

// Keystrokes typed while a command is still running are echoed immediately
// (the editor is fed every pass), and the completed line is submitted once the
// running command's result lands -- input is never buffered until it is lost.
void test_input_while_command_in_flight_is_echoed_and_then_runs() {
    const std::uint32_t src = 0xAAAA0007U;
    const std::uint32_t boot = 0xBBBB0007U;

    feed(src, boot, "slow\r");
    pump();  // submits "slow"; awaiting its result

    g_sent.clear();
    feed(src, boot, "next\r");
    pump();  // command still running: "next" is echoed but not yet submitted
    TEST_ASSERT_TRUE(all_sent().find("next") != std::string::npos);  // echoed
    TEST_ASSERT_EQUAL(1U, g_submit.lines.size());                    // not submitted

    g_responder->deliver_command_output(src, boot, "slow done", 0U);
    pump();  // result lands -> held line runs
    TEST_ASSERT_EQUAL(2U, g_submit.lines.size());
    TEST_ASSERT_EQUAL_STRING("next", g_submit.lines[1].c_str());
    TEST_ASSERT_EQUAL(0U, g_responder->stats().in_bytes_dropped);
}

// submit() refusing (queue full) is reported and does not wedge the origin.
void test_submit_busy_is_reported() {
    g_submit.accept = false;
    feed(0xAAAA0005U, 0xBBBB0005U, "info\r");
    pump();
    TEST_ASSERT_EQUAL(0U, g_submit.lines.size());
    TEST_ASSERT_EQUAL(1U, g_responder->stats().lines_dropped_busy);
    TEST_ASSERT_TRUE(all_sent().find("busy") != std::string::npos);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_first_contact_paints_prompt_then_echoes);
    RUN_TEST(test_keystrokes_are_echoed);
    RUN_TEST(test_line_submits_and_output_mirrors_back);
    RUN_TEST(test_tab_completion);
    RUN_TEST(test_nonzero_status_is_reported);
    RUN_TEST(test_consecutive_commands_each_run);
    RUN_TEST(test_input_while_command_in_flight_is_echoed_and_then_runs);
    RUN_TEST(test_pool_evicts_lru_and_isolates_origins);
    RUN_TEST(test_submit_busy_is_reported);
    return UNITY_END();
}
