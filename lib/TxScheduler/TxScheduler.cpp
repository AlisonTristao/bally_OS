#include <TxScheduler.h>

#include <cstring>

constexpr std::size_t TxScheduler::kQueueCapacity[TxScheduler::kPriorityCount];

void TxScheduler::configure(RadioSendCallback callback, void* context,
                            std::uint64_t delivery_timeout_ms) noexcept {
    radio_send_ = callback;
    radio_context_ = context;
    delivery_timeout_ms_ = delivery_timeout_ms == 0U
                               ? kDefaultDeliveryTimeoutMs
                               : delivery_timeout_ms;
}

bool TxScheduler::enqueue_callback(void* context, const std::uint8_t* data,
                                   std::size_t size) noexcept {
    return context != nullptr &&
           static_cast<TxScheduler*>(context)->enqueue(data, size);
}

TxScheduler::Priority TxScheduler::classify(
    const btp::DecodedFrame& frame) noexcept {
    if (frame.header.type == btp::MessageType::Command &&
        frame.header.object_id == 0x0002U) {
        return Priority::CommandResult;
    }
    if (frame.header.type == btp::MessageType::Control) {
        return Priority::Status;
    }
    if (frame.header.type == btp::MessageType::Log &&
        (frame.header.object_id == 2U || frame.header.object_id == 3U)) {
        return Priority::CriticalLog;
    }
    if (frame.header.type == btp::MessageType::Telemetry) {
        return Priority::Telemetry;
    }
    return Priority::Debug;
}

bool TxScheduler::enqueue(const std::uint8_t* data, std::size_t size) noexcept {
    if (data == nullptr || size < btp::kV1MinimumFrameSize ||
        size > btp::kEspNowMaxFrameSize) {
        dropped_.fetch_add(1U, std::memory_order_relaxed);
        return false;
    }

    btp::DecodedFrame decoded{};
    if (btp::decode(data, size, btp::TransportProfile::EspNow, &decoded) !=
        btp::Error::Ok) {
        dropped_.fetch_add(1U, std::memory_order_relaxed);
        return false;
    }

    const Priority priority = classify(decoded);
    const std::size_t queue_index = index(priority);
    Queue& queue = queues_[queue_index];
    if (queue.lock.test_and_set(std::memory_order_acquire)) {
        dropped_.fetch_add(1U, std::memory_order_relaxed);
        dropped_by_priority_[queue_index].fetch_add(
            1U, std::memory_order_relaxed);
        return false;
    }

    if (queue.count >= kQueueCapacity[queue_index]) {
        queue.lock.clear(std::memory_order_release);
        dropped_.fetch_add(1U, std::memory_order_relaxed);
        dropped_by_priority_[queue_index].fetch_add(
            1U, std::memory_order_relaxed);
        return false;
    }

    EncodedFrame& frame = queue.frames[queue.write];
    frame.size = static_cast<std::uint16_t>(size);
    std::memcpy(frame.bytes, data, size);
    queue.write = (queue.write + 1U) % kQueueCapacity[queue_index];
    ++queue.count;
    queue.lock.clear(std::memory_order_release);

    accepted_.fetch_add(1U, std::memory_order_relaxed);
    queued_by_priority_[queue_index].fetch_add(1U,
                                                std::memory_order_relaxed);
    return true;
}

bool TxScheduler::pop(Priority priority, EncodedFrame* output) noexcept {
    if (output == nullptr) return false;
    const std::size_t queue_index = index(priority);
    Queue& queue = queues_[queue_index];
    if (queue.lock.test_and_set(std::memory_order_acquire)) return false;
    if (queue.count == 0U) {
        queue.lock.clear(std::memory_order_release);
        return false;
    }
    *output = queue.frames[queue.read];
    queue.read = (queue.read + 1U) % kQueueCapacity[queue_index];
    --queue.count;
    queue.lock.clear(std::memory_order_release);
    return true;
}

bool TxScheduler::pump(std::uint64_t now_ms) noexcept {
    if (awaiting_delivery_.load(std::memory_order_acquire)) {
        const std::uint8_t event =
            delivery_event_.exchange(0U, std::memory_order_acq_rel);
        if (event == 1U) {
            delivered_.fetch_add(1U, std::memory_order_relaxed);
            awaiting_delivery_.store(false, std::memory_order_release);
        } else if (event == 2U) {
            delivery_failed_.fetch_add(1U, std::memory_order_relaxed);
            dropped_.fetch_add(1U, std::memory_order_relaxed);
            awaiting_delivery_.store(false, std::memory_order_release);
        } else if (now_ms - pending_since_ms_ >= delivery_timeout_ms_) {
            timeouts_.fetch_add(1U, std::memory_order_relaxed);
            dropped_.fetch_add(1U, std::memory_order_relaxed);
            awaiting_delivery_.store(false, std::memory_order_release);
        } else {
            return false;
        }
    }

    if (radio_send_ == nullptr) return false;
    bool found = false;
    for (std::size_t value = 0U; value < kPriorityCount; ++value) {
        if (pop(static_cast<Priority>(value), &pending_)) {
            found = true;
            break;
        }
    }
    if (!found) return false;

    // Mark pending before entering the driver: even a very fast callback is
    // then recorded against this sole outstanding frame.
    awaiting_delivery_.store(true, std::memory_order_release);
    pending_since_ms_ = now_ms;
    delivery_event_.store(0U, std::memory_order_release);
    if (!radio_send_(radio_context_, pending_.bytes, pending_.size)) {
        awaiting_delivery_.store(false, std::memory_order_release);
        dropped_.fetch_add(1U, std::memory_order_relaxed);
        return false;
    }
    return true;
}

void TxScheduler::on_delivery(bool delivered) noexcept {
    if (!awaiting_delivery_.load(std::memory_order_acquire)) return;
    delivery_event_.store(delivered ? 1U : 2U, std::memory_order_release);
}

bool TxScheduler::idle() const noexcept {
    if (awaiting_delivery_.load(std::memory_order_acquire)) return false;
    for (std::size_t value = 0U; value < kPriorityCount; ++value) {
        if (queued_count(static_cast<Priority>(value)) != 0U) return false;
    }
    return true;
}

std::size_t TxScheduler::queued_count(Priority priority) const noexcept {
    const std::size_t queue_index = index(priority);
    if (queue_index >= kPriorityCount) return 0U;
    const Queue& queue = queues_[queue_index];
    if (queue.lock.test_and_set(std::memory_order_acquire)) return 0U;
    const std::size_t count = queue.count;
    queue.lock.clear(std::memory_order_release);
    return count;
}

TxScheduler::Stats TxScheduler::stats() const noexcept {
    Stats result{
        accepted_.load(std::memory_order_relaxed),
        delivered_.load(std::memory_order_relaxed),
        timeouts_.load(std::memory_order_relaxed),
        dropped_.load(std::memory_order_relaxed),
        delivery_failed_.load(std::memory_order_relaxed),
        {},
        {},
    };
    for (std::size_t value = 0U; value < kPriorityCount; ++value) {
        result.queued_by_priority[value] =
            queued_by_priority_[value].load(std::memory_order_relaxed);
        result.dropped_by_priority[value] =
            dropped_by_priority_[value].load(std::memory_order_relaxed);
    }
    return result;
}
