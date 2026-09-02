#ifndef RX_ROUTER_H
#define RX_ROUTER_H

#include <btp/codec.hpp>
#include <btp/fragmentation.hpp>
#include <btp/receiver.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

// The robot's receive path, up to (but not including) the routing decision:
// btp::decode() (envelope + CRC-32) and then, only for a fragmented frame,
// btp::Reassembler. What comes out is one COMPLETE logical message, or
// nothing yet.
//
// Why this exists as a stage of its own. Until the dongle became a hub, the
// only peer that ever spoke to the robot was the dongle itself, and it
// reassembled on the robot's behalf; a COMMAND arrived whole, and the two
// branches that could fragment (COMMAND and CONTROL) were special-cased
// inside ROBOT::handleReceiveStatic. With the hub, TraceView talks to the
// robot end to end and the dongle relays fragment by fragment, verbatim, on
// purpose -- reassembly is the endpoint's job, never the hub's. So a frame
// of ANY type can now arrive in pieces, and reassembly has to be a stage
// every received frame passes through instead of a branch two types take.
//
// Deliberately type-blind: this class never looks at header.type or
// object_id. Deciding "who handles this" is the ROBOT's job and happens
// after a message is whole -- which is also the order topico 31 needs, when
// aead_open() lands between the two (a sealed message can only be opened
// once its fragments are back together, because the tag was computed over
// the whole logical payload).
//
// It is now a thin adapter over btp::Receiver (BTP/include/btp/receiver.hpp,
// tested by BTP/tests/test_receiver.cpp) -- the decode + CRC + reassembly
// wiring, the timeout sweep and the STATUS counters all live there. What
// stays here: the RxRouter::* names the robot already uses, the
// RoutedMessage with its inline payload buffer, and the ESP-NOW profile.
// bally_dongle's lib/ProtocolRouter is the same btp::Receiver with a
// per-datagram MAC bolted on; keeping the four slots and the 4000 ms timeout
// identical is what makes the two ends of the radio link tolerate the same
// loss and reordering.
namespace RxRouter {

// Sized for the largest logical message the robot accepts today: a
// COMMAND_REQUEST is a 20-octet prefix plus up to 512 octets of shell text
// (btp_command::kRequestPrefixSize + kMaxShellCommandSize = 532). Rounded up
// to leave room for a slightly larger request without a silent
// MessageTooLarge. Not included from BtpTransport.h on purpose -- this lib
// must not depend on the command envelope to reassemble bytes; the test
// suite static_asserts the two agree.
constexpr std::size_t kMaxPayloadSize = 600U;

// Four concurrent reassemblies, not two. With a single peer, two was enough
// (the dongle sent one fragmented message at a time). Behind a hub, the
// robot's one radio peer multiplexes several senders -- TraceView and the
// dongle at least -- and btp::Reassembler keys slots by
// (source_id, boot_id, sequence), so concurrent senders each need a slot.
constexpr std::size_t kSlotCount = 4U;

// 4000 ms, up from 2000. This is how long a half-arrived message occupies a
// slot before expire() reclaims it. Relayed traffic crosses one more hop
// than it used to, so the window has to be the more forgiving of the two.
constexpr std::uint64_t kReassemblyTimeoutMs = 4000U;

enum class Outcome : std::uint8_t {
    Routed,             // A complete logical message is in *message_out.
    FragmentAccepted,   // Stored; the message is still incomplete.
    DuplicateFragment,  // Byte-identical retry; absorbed, nothing corrupted.
    DroppedDecode,      // btp::decode() failed for a reason other than CRC.
    DroppedCrc,
    DroppedReassembly,  // Conflict/InvalidFragment/MessageTooLarge/NoSlot.
    DroppedInvalidArgument
};

struct RoutedMessage {
    // Normalized on completion by btp::Reassembler: FRAGMENTED cleared,
    // fragment_index 0, fragment_count 1. So a consumer cannot tell -- and
    // must not care -- whether this arrived in one piece or in twelve.
    btp::Header header;
    std::uint8_t payload[kMaxPayloadSize];
    std::size_t payload_size;

    // True only when this message came out of the slot pool. STATUS section
    // 5's reassembly_completed counts reassemblies, not receptions, so the
    // caller needs to tell the two apart; the header above deliberately
    // cannot say (see the note on normalization).
    bool reassembled;
};

struct Stats {
    std::uint32_t routed;
    std::uint32_t fragments_accepted;
    std::uint32_t duplicate_fragments;
    std::uint32_t dropped_decode;
    std::uint32_t dropped_crc;
    std::uint32_t dropped_reassembly;
    std::uint32_t dropped_invalid_argument;
    // Partial messages abandoned because no further fragment arrived within
    // kReassemblyTimeoutMs. Counted by btp::Receiver as it sweeps; this is
    // what STATUS's reassembly_timeouts reports.
    std::uint32_t reassembly_timeouts;
};

// One receiver's worth of decode + reassembly state, over a btp::Receiver.
//
// CONCURRENCY. Exactly one context MUTATES this: submit(), on the robot's
// ESP-NOW receive callback -- btp::Receiver is not internally synchronised and
// does not need to be here. stats() is read from the routine/shell task while
// submit() writes; the counters are plain 32-bit words, so that read is a
// relaxed view of monotonic values -- exactly what the previous
// memory_order_relaxed atomics provided, and all a STATUS report needs. The
// timeout counter still only advances when a datagram arrives (the sweep runs
// inside submit()); a robot hearing nothing reports no new timeouts until the
// next frame, which matches how slot release was always tied to arrivals.
class Router {
public:
    Router() noexcept;

    // False only if the slot/storage wiring is malformed, which is a
    // programming error, not a runtime condition. Checked once at boot.
    bool valid() const noexcept;

    // Feeds one received datagram. On Outcome::Routed, *message_out holds the
    // complete logical message; its payload is COPIED OUT and, when it came
    // from reassembly, the slot is released immediately -- so a slow consumer
    // downstream can never starve the small slot pool. Every other outcome
    // leaves *message_out untouched.
    Outcome submit(const std::uint8_t* data,
                   std::size_t size,
                   std::uint64_t now_ms,
                   RoutedMessage* message_out) noexcept;

    // Deliberately NOT a public sweep: sweeping is submit()'s job, on
    // submit()'s context. What a caller wants from it -- how many partial
    // messages were lost -- is in Stats::reassembly_timeouts. Safe to call
    // from a single-threaded test through this name.
    std::size_t expireForTest(std::uint64_t now_ms) noexcept;

    // A snapshot, safe to read from a context other than submit()'s.
    Stats stats() const noexcept;

private:
    btp::ReassemblySlot slots_[kSlotCount];
    std::uint8_t storage_[kSlotCount][kMaxPayloadSize];
    std::array<btp::ReassemblyStorage, kSlotCount> storage_views_;
    btp::Receiver receiver_;
};

}  // namespace RxRouter

#endif  // RX_ROUTER_H
