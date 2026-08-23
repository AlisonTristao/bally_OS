#include "RxRouter.h"

#include <cstring>

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

void fill_routed(const btp::Header& header,
                 btp::ByteView payload,
                 bool reassembled,
                 RoutedMessage* out) noexcept {
    out->header = header;
    out->payload_size = payload.size;
    if (payload.size > 0U && payload.data != nullptr) {
        std::memcpy(out->payload, payload.data, payload.size);
    }
    out->reassembled = reassembled;
}

}  // namespace

// storage_views_ is declared before reassembler_ so that member
// initialization order gives the Reassembler constructor a fully built view
// table; reordering the two declarations in the header would hand it
// uninitialized pointers.
Router::Router() noexcept
    : slots_(),
      storage_(),
      storage_views_(make_storage_views(storage_)),
      reassembler_(slots_, storage_views_.data(), kSlotCount,
                   kReassemblyTimeoutMs) {}

bool Router::valid() const noexcept { return reassembler_.valid(); }

Outcome Router::submit(const std::uint8_t* data,
                       std::size_t size,
                       std::uint64_t now_ms,
                       RoutedMessage* message_out) noexcept {
    // Sweep here, on this context, rather than from a timer on another task.
    // btp::Reassembler::push() would expire stale slots anyway further down;
    // doing it explicitly first is what lets the loss be COUNTED without a
    // second task ever touching the slot table. See the concurrency note in
    // the header for what this replaced.
    const std::size_t expired = reassembler_.expire(now_ms);
    if (expired != 0U) {
        reassembly_timeouts_.fetch_add(static_cast<std::uint32_t>(expired),
                                       std::memory_order_relaxed);
    }

    if (data == nullptr || size == 0U || message_out == nullptr) {
        dropped_invalid_argument_.fetch_add(1U, std::memory_order_relaxed);
        return Outcome::DroppedInvalidArgument;
    }

    btp::DecodedFrame decoded{};
    const btp::Error decode_error = btp::decode(
        data, size, btp::TransportProfile::EspNow, &decoded);
    if (decode_error != btp::Error::Ok) {
        // A frame rejected by CRC is never also counted as a decode error:
        // STATUS reports the two separately because they mean different
        // things (radio corruption versus a peer speaking the wrong dialect).
        if (decode_error == btp::Error::CrcMismatch) {
            dropped_crc_.fetch_add(1U, std::memory_order_relaxed);
            return Outcome::DroppedCrc;
        }
        dropped_decode_.fetch_add(1U, std::memory_order_relaxed);
        return Outcome::DroppedDecode;
    }

    if ((decoded.header.flags & btp::kFlagFragmented) == 0U) {
        // decode()'s header validation already requires fragment_index == 0
        // and fragment_count == 1 here, so this datagram IS the whole logical
        // message and no slot is involved. Its payload still only points into
        // the caller's transient RX buffer, so copy it out now.
        fill_routed(decoded.header, decoded.payload, false, message_out);
        routed_.fetch_add(1U, std::memory_order_relaxed);
        return Outcome::Routed;
    }

    btp::ReassembledMessage completed{};
    const btp::ReassemblyEvent event =
        reassembler_.push(decoded, now_ms, &completed);
    switch (event) {
        case btp::ReassemblyEvent::Accepted:
            fragments_accepted_.fetch_add(1U, std::memory_order_relaxed);
            return Outcome::FragmentAccepted;
        case btp::ReassemblyEvent::Duplicate:
            duplicate_fragments_.fetch_add(1U, std::memory_order_relaxed);
            return Outcome::DuplicateFragment;
        case btp::ReassemblyEvent::Complete:
            fill_routed(completed.header, completed.payload, true,
                        message_out);
            // Release immediately. What travels onward is the copy above, not
            // the slot, so a busy handler downstream never holds a slot that
            // another sender needs -- the failure mode the four-slot pool is
            // small enough to hit.
            reassembler_.release(completed.slot_index);
            routed_.fetch_add(1U, std::memory_order_relaxed);
            return Outcome::Routed;
        case btp::ReassemblyEvent::InvalidFragment:
        case btp::ReassemblyEvent::Conflict:
        case btp::ReassemblyEvent::MessageTooLarge:
        case btp::ReassemblyEvent::NoSlot:
            dropped_reassembly_.fetch_add(1U, std::memory_order_relaxed);
            return Outcome::DroppedReassembly;
        case btp::ReassemblyEvent::InvalidArgument:
            dropped_invalid_argument_.fetch_add(1U, std::memory_order_relaxed);
            return Outcome::DroppedInvalidArgument;
    }

    // Unreachable while the enum above is exhaustive; counted rather than
    // asserted so a future BTP event can never silently become "accepted".
    dropped_invalid_argument_.fetch_add(1U, std::memory_order_relaxed);
    return Outcome::DroppedInvalidArgument;
}

std::size_t Router::expireForTest(std::uint64_t now_ms) noexcept {
    const std::size_t expired = reassembler_.expire(now_ms);
    if (expired != 0U) {
        reassembly_timeouts_.fetch_add(static_cast<std::uint32_t>(expired),
                                       std::memory_order_relaxed);
    }
    return expired;
}

Stats Router::stats() const noexcept {
    Stats snapshot{};
    snapshot.routed = routed_.load(std::memory_order_relaxed);
    snapshot.fragments_accepted = fragments_accepted_.load(std::memory_order_relaxed);
    snapshot.duplicate_fragments = duplicate_fragments_.load(std::memory_order_relaxed);
    snapshot.dropped_decode = dropped_decode_.load(std::memory_order_relaxed);
    snapshot.dropped_crc = dropped_crc_.load(std::memory_order_relaxed);
    snapshot.dropped_reassembly = dropped_reassembly_.load(std::memory_order_relaxed);
    snapshot.dropped_invalid_argument = dropped_invalid_argument_.load(std::memory_order_relaxed);
    snapshot.reassembly_timeouts = reassembly_timeouts_.load(std::memory_order_relaxed);
    return snapshot;
}

}  // namespace RxRouter
