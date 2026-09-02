#include "RxRouter.h"

namespace RxRouter {
namespace {

std::array<btp::ReassemblyStorage, kSlotCount> make_storage_views(
    std::uint8_t (&storage)[kSlotCount][kMaxPayloadSize]) noexcept {
    std::array<btp::ReassemblyStorage, kSlotCount> views{};
    for (std::size_t index = 0U; index < kSlotCount; ++index) {
        views[index] = btp::ReassemblyStorage{storage[index], kMaxPayloadSize};
    }
    return views;
}

Outcome map_outcome(btp::ReceiveOutcome outcome) noexcept {
    switch (outcome) {
        case btp::ReceiveOutcome::Complete: return Outcome::Routed;
        case btp::ReceiveOutcome::FragmentAccepted: return Outcome::FragmentAccepted;
        case btp::ReceiveOutcome::DuplicateFragment: return Outcome::DuplicateFragment;
        case btp::ReceiveOutcome::DroppedCrc: return Outcome::DroppedCrc;
        case btp::ReceiveOutcome::DroppedDecode: return Outcome::DroppedDecode;
        case btp::ReceiveOutcome::DroppedReassembly: return Outcome::DroppedReassembly;
        case btp::ReceiveOutcome::InvalidArgument: return Outcome::DroppedInvalidArgument;
    }
    return Outcome::DroppedInvalidArgument;
}

}  // namespace

// storage_views_ is declared before receiver_ so member initialization order
// gives the btp::Receiver constructor a fully built view table; reordering the
// two declarations in the header would hand it uninitialized pointers.
Router::Router() noexcept
    : slots_(),
      storage_(),
      storage_views_(make_storage_views(storage_)),
      receiver_(slots_, storage_views_.data(), kSlotCount, kReassemblyTimeoutMs,
                btp::TransportProfile::EspNow) {}

bool Router::valid() const noexcept { return receiver_.valid(); }

Outcome Router::submit(const std::uint8_t* data,
                       std::size_t size,
                       std::uint64_t now_ms,
                       RoutedMessage* message_out) noexcept {
    if (message_out == nullptr) {
        // btp::Receiver would report InvalidArgument here too, but it needs a
        // non-null message_out to be handed one; short-circuit.
        return Outcome::DroppedInvalidArgument;
    }

    btp::ReceivedMessage received{};
    const btp::ReceiveOutcome outcome = receiver_.submit(
        data, size, now_ms, message_out->payload, kMaxPayloadSize, &received);

    if (outcome == btp::ReceiveOutcome::Complete) {
        message_out->header = received.header;
        message_out->payload_size = received.payload.size;
        message_out->reassembled = received.reassembled;
    }
    return map_outcome(outcome);
}

std::size_t Router::expireForTest(std::uint64_t now_ms) noexcept {
    return receiver_.expire(now_ms);
}

Stats Router::stats() const noexcept {
    const btp::Receiver::Stats s = receiver_.stats();
    Stats snapshot{};
    snapshot.routed = s.completed;
    snapshot.fragments_accepted = s.fragments_accepted;
    snapshot.duplicate_fragments = s.duplicate_fragments;
    snapshot.dropped_decode = s.dropped_decode;
    snapshot.dropped_crc = s.dropped_crc;
    snapshot.dropped_reassembly = s.dropped_reassembly;
    snapshot.dropped_invalid_argument = s.invalid_argument;
    snapshot.reassembly_timeouts = s.reassembly_timeouts;
    return snapshot;
}

}  // namespace RxRouter
