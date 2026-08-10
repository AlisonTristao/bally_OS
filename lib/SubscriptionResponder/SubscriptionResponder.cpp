#include <SubscriptionResponder.h>

#include <BtpTransport.h>
#include <TelemetryPublisher.h>

namespace {

// Common result/error codes, bally_protocol/docs/COMMANDS_AND_ACTIONS.md
// section 2 -- same local-constant pattern ManifestResponder.cpp already
// uses rather than pulling in CommandProcessor for a handful of enum values.
constexpr std::uint8_t kStatusSuccess = 0x00U;
constexpr std::uint8_t kStatusRejected = 0x01U;
constexpr std::uint16_t kErrorNone = 0x0000U;
constexpr std::uint16_t kErrorInvalidArgument = 0x0003U;
constexpr std::uint16_t kErrorStaleTargetBoot = 0x0009U;
constexpr std::uint16_t kErrorNotFound = 0x000BU;

constexpr std::size_t kSubscribeRequestSize = 20U;
constexpr std::size_t kSubscribeResultSize = 28U;
constexpr std::size_t kUnsubscribeRequestSize = 12U;
constexpr std::size_t kUnsubscribeResultSize = 16U;

std::uint32_t read_u32_le(const std::uint8_t* data) noexcept {
    return static_cast<std::uint32_t>(data[0]) | (static_cast<std::uint32_t>(data[1]) << 8U) |
           (static_cast<std::uint32_t>(data[2]) << 16U) | (static_cast<std::uint32_t>(data[3]) << 24U);
}

std::uint16_t read_u16_le(const std::uint8_t* data) noexcept {
    return static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[0]) |
                                      (static_cast<std::uint16_t>(data[1]) << 8U));
}

void write_u16_le(std::uint8_t* out, std::uint16_t value) noexcept {
    out[0] = static_cast<std::uint8_t>(value);
    out[1] = static_cast<std::uint8_t>(value >> 8U);
}

void write_u32_le(std::uint8_t* out, std::uint32_t value) noexcept {
    out[0] = static_cast<std::uint8_t>(value);
    out[1] = static_cast<std::uint8_t>(value >> 8U);
    out[2] = static_cast<std::uint8_t>(value >> 16U);
    out[3] = static_cast<std::uint8_t>(value >> 24U);
}

void write_reference(std::uint8_t* out, const btp::Header& request_header) noexcept {
    write_u32_le(out, request_header.source_id);
    write_u32_le(out + 4U, request_header.boot_id);
    write_u32_le(out + 8U, request_header.sequence);
}

}  // namespace

void SubscriptionResponder::configure(BtpEndpoint& endpoint, TelemetryPublisher& publisher) noexcept {
    endpoint_ = &endpoint;
    publisher_ = &publisher;
}

bool SubscriptionResponder::handle_subscribe(const btp::Header& request_header, btp::ByteView payload,
                                             std::uint64_t timestamp_us) noexcept {
    if (endpoint_ == nullptr || publisher_ == nullptr || payload.data == nullptr ||
        payload.size < kSubscribeRequestSize) {
        return false;
    }

    const std::uint32_t targetSourceId = read_u32_le(payload.data);
    const std::uint32_t targetBootId = read_u32_le(payload.data + 4U);
    const std::uint16_t topicId = read_u16_le(payload.data + 8U);
    const std::uint32_t requestedRateMillihz = read_u32_le(payload.data + 12U);
    const std::uint32_t requestedLeaseMs = read_u32_le(payload.data + 16U);

    const std::uint32_t localSourceId = endpoint_->source_id();
    const std::uint32_t localBootId = endpoint_->boot_id();

    std::uint8_t status = kStatusSuccess;
    std::uint16_t errorCode = kErrorNone;
    std::uint32_t subscriptionId = 0U;
    std::uint32_t effectiveRateMillihz = 0U;
    std::uint32_t grantedLeaseMs = 0U;

    if (targetSourceId != 0U && targetSourceId != localSourceId) {
        status = kStatusRejected;
        errorCode = kErrorNotFound;
    } else if (targetBootId == 0U || targetBootId != localBootId) {
        status = kStatusRejected;
        errorCode = kErrorStaleTargetBoot;
    } else if (topicId == 0U || requestedRateMillihz == 0U || requestedLeaseMs == 0U) {
        status = kStatusRejected;
        errorCode = kErrorInvalidArgument;
    } else {
        const TelemetryPublisher::SubscribeOutcome outcome = publisher_->subscribe(
            topicId, request_header.source_id, request_header.boot_id, requestedRateMillihz,
            requestedLeaseMs, timestamp_us);
        if (!outcome.topic_known) {
            status = kStatusRejected;
            errorCode = kErrorNotFound;
        } else {
            subscriptionId = outcome.subscription_id;
            effectiveRateMillihz = outcome.effective_rate_millihz;
            grantedLeaseMs = outcome.granted_lease_ms;
        }
    }

    std::uint8_t responsePayload[kSubscribeResultSize];
    write_reference(responsePayload, request_header);
    responsePayload[12] = status;
    responsePayload[13] = 0U;  // reserved
    write_u16_le(responsePayload + 14U, errorCode);
    write_u32_le(responsePayload + 16U, subscriptionId);
    write_u32_le(responsePayload + 20U, effectiveRateMillihz);
    write_u32_le(responsePayload + 24U, grantedLeaseMs);

    return endpoint_->send_logical(btp::MessageType::Control, kSubscribeResultObjectId, responsePayload,
                                   sizeof(responsePayload), timestamp_us);
}

bool SubscriptionResponder::handle_unsubscribe(const btp::Header& request_header, btp::ByteView payload,
                                               std::uint64_t timestamp_us) noexcept {
    if (endpoint_ == nullptr || publisher_ == nullptr || payload.data == nullptr ||
        payload.size < kUnsubscribeRequestSize) {
        return false;
    }

    const std::uint32_t targetSourceId = read_u32_le(payload.data);
    const std::uint32_t targetBootId = read_u32_le(payload.data + 4U);
    const std::uint32_t subscriptionId = read_u32_le(payload.data + 8U);

    const std::uint32_t localSourceId = endpoint_->source_id();
    const std::uint32_t localBootId = endpoint_->boot_id();

    std::uint8_t status = kStatusSuccess;
    std::uint16_t errorCode = kErrorNone;

    if (targetSourceId != 0U && targetSourceId != localSourceId) {
        status = kStatusRejected;
        errorCode = kErrorNotFound;
    } else if (targetBootId == 0U || targetBootId != localBootId) {
        status = kStatusRejected;
        errorCode = kErrorStaleTargetBoot;
    } else {
        // Removing an already-absent subscription is SUCCESS/NONE per
        // COMMANDS_AND_ACTIONS.md section 7 ("torna retries idempotentes"),
        // so UnsubscribeOutcome::NotFound is not turned into an error here.
        (void)publisher_->unsubscribe(subscriptionId);
    }

    std::uint8_t responsePayload[kUnsubscribeResultSize];
    write_reference(responsePayload, request_header);
    responsePayload[12] = status;
    responsePayload[13] = 0U;  // reserved
    write_u16_le(responsePayload + 14U, errorCode);

    return endpoint_->send_logical(btp::MessageType::Control, kUnsubscribeResultObjectId, responsePayload,
                                   sizeof(responsePayload), timestamp_us);
}
