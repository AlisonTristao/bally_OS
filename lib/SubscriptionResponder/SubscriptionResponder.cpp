#include <SubscriptionResponder.h>

#include <BtpTransport.h>
#include <TelemetryPublisher.h>
#include <btp/messages.hpp>

namespace {

// Common result/error codes, BTP/docs/commands.md section 1 ("Result and
// error codes") -- same local-constant pattern ManifestResponder.cpp already
// uses rather than pulling in CommandProcessor for a handful of enum values.
constexpr std::uint8_t kStatusSuccess = 0x00U;
constexpr std::uint8_t kStatusRejected = 0x01U;
constexpr std::uint16_t kErrorNone = 0x0000U;
constexpr std::uint16_t kErrorInvalidArgument = 0x0003U;
constexpr std::uint16_t kErrorCapacityExhausted = 0x0005U;
constexpr std::uint16_t kErrorStaleTargetBoot = 0x0009U;
constexpr std::uint16_t kErrorNotFound = 0x000BU;

constexpr std::size_t kSubscribeRequestSize = 20U;
constexpr std::size_t kSubscribeResultSize = 28U;
constexpr std::size_t kUnsubscribeRequestSize = 12U;
constexpr std::size_t kUnsubscribeResultSize = 16U;

// The one hand read kept out of btp::messages: UNSUBSCRIBE's three u32 fields,
// because handle_unsubscribe stays deliberately lenient (btp::decode_unsubscribe
// would additionally reject a zero subscription_id, turning a spec-defined
// idempotent retry into an error).
std::uint32_t read_u32_le(const std::uint8_t* data) noexcept {
    return static_cast<std::uint32_t>(data[0]) | (static_cast<std::uint32_t>(data[1]) << 8U) |
           (static_cast<std::uint32_t>(data[2]) << 16U) | (static_cast<std::uint32_t>(data[3]) << 24U);
}

btp::RequestRef reference_of(const btp::Header& request_header) noexcept {
    return {request_header.source_id, request_header.boot_id, request_header.sequence};
}

}  // namespace

void SubscriptionResponder::configure(BtpEndpoint& endpoint, TelemetryPublisher& publisher,
                                      BtpSealFn seal_link, void* seal_link_context,
                                      BtpSealFn seal_endpoint,
                                      void* seal_endpoint_context) noexcept {
    endpoint_ = &endpoint;
    publisher_ = &publisher;
    seal_link_ = seal_link;
    seal_link_context_ = seal_link_context;
    seal_endpoint_ = seal_endpoint;
    seal_endpoint_context_ = seal_endpoint_context;
}

bool SubscriptionResponder::seal_for(bally::Channel channel, BtpSealFn* seal_out,
                                     void** context_out) const noexcept {
    const BtpSealFn seal = channel == bally::Channel::B_Endpoint ? seal_endpoint_ : seal_link_;
    void* const context =
        channel == bally::Channel::B_Endpoint ? seal_endpoint_context_ : seal_link_context_;

    // Same rule as CommandProcessor::send_result(): both nullptr means "no
    // encryption at all" (every reply unsealed, pre-channel-B behavior); once
    // ANY channel has a key configured, the channel whose own key is still
    // missing must never fall back to cleartext or to the other channel's
    // key.
    if (seal == nullptr && (seal_link_ != nullptr || seal_endpoint_ != nullptr)) {
        return false;
    }
    *seal_out = seal;
    *context_out = context;
    return true;
}

bool SubscriptionResponder::handle_subscribe(const btp::Header& request_header, btp::ByteView payload,
                                             std::uint64_t timestamp_us,
                                             bally::Channel channel) noexcept {
    if (endpoint_ == nullptr || publisher_ == nullptr || payload.data == nullptr ||
        payload.size < kSubscribeRequestSize) {
        return false;
    }

    BtpSealFn seal = nullptr;
    void* seal_context = nullptr;
    if (!seal_for(channel, &seal, &seal_context)) {
        return false;
    }

    // The 20-octet SUBSCRIBE layout (commands.md section 4), the zero flags word
    // and "every field non-zero" are btp::decode_subscribe. A well-formed
    // payload whose values are unusable still gets a REJECTED reply, as before.
    btp::Subscribe sub{};
    const bool subOk =
        btp::decode_subscribe(payload.data, payload.size, &sub) == btp::MessageError::Ok;

    const std::uint32_t localSourceId = endpoint_->source_id();
    const std::uint32_t localBootId = endpoint_->boot_id();

    std::uint8_t status = kStatusSuccess;
    std::uint16_t errorCode = kErrorNone;
    std::uint32_t subscriptionId = 0U;
    std::uint32_t effectiveRateMillihz = 0U;
    std::uint32_t grantedLeaseMs = 0U;

    if (!subOk) {
        status = kStatusRejected;
        errorCode = kErrorInvalidArgument;
    } else if (sub.target_source_id != localSourceId) {  // decode already guaranteed != 0
        status = kStatusRejected;
        errorCode = kErrorNotFound;
    } else if (sub.target_boot_id != localBootId) {  // decode already guaranteed != 0
        status = kStatusRejected;
        errorCode = kErrorStaleTargetBoot;
    } else {
        const TelemetryPublisher::SubscribeOutcome outcome = publisher_->subscribe(
            sub.topic_id, request_header.source_id, request_header.boot_id,
            sub.requested_rate_millihz, sub.requested_lease_ms, timestamp_us);
        if (!outcome.topic_known) {
            status = kStatusRejected;
            errorCode = kErrorNotFound;
        } else if (outcome.rate_below_minimum) {
            // A rate under the schema's floor cannot be granted: section 4
            // forbids answering with a rate above the requested one, so the
            // only honest answer is a rejection. Note this is the *opposite*
            // of the max case just above, where the request is clamped and
            // the client is told the clamped value (topico 17 acceptance
            // criterion "pedido acima do maximo e limitado e informado").
            status = kStatusRejected;
            errorCode = kErrorInvalidArgument;
        } else if (outcome.capacity_exhausted) {
            status = kStatusRejected;
            errorCode = kErrorCapacityExhausted;
        } else {
            subscriptionId = outcome.subscription_id;
            effectiveRateMillihz = outcome.effective_rate_millihz;
            grantedLeaseMs = outcome.granted_lease_ms;
        }
    }

    btp::SubscribeResult result{};
    result.request = reference_of(request_header);
    result.status = status;
    result.error_code = errorCode;
    result.subscription_id = subscriptionId;
    result.effective_rate_millihz = effectiveRateMillihz;
    result.granted_lease_ms = grantedLeaseMs;

    std::uint8_t responsePayload[kSubscribeResultSize];
    std::size_t written = 0U;
    if (btp::encode_subscribe_result(result, responsePayload, sizeof(responsePayload), &written) !=
        btp::MessageError::Ok) {
        return false;
    }
    return endpoint_->send_logical(btp::MessageType::Control, kSubscribeResultObjectId, responsePayload,
                                   written, timestamp_us, seal, seal_context);
}

bool SubscriptionResponder::handle_unsubscribe(const btp::Header& request_header, btp::ByteView payload,
                                               std::uint64_t timestamp_us,
                                               bally::Channel channel) noexcept {
    if (endpoint_ == nullptr || publisher_ == nullptr || payload.data == nullptr ||
        payload.size < kUnsubscribeRequestSize) {
        return false;
    }

    BtpSealFn seal = nullptr;
    void* seal_context = nullptr;
    if (!seal_for(channel, &seal, &seal_context)) {
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
        // commands.md section 4 ("makes retries idempotent"),
        // so UnsubscribeOutcome::NotFound is not turned into an error here.
        (void)publisher_->unsubscribe(subscriptionId);
    }

    btp::ControlResult result{};
    result.request = reference_of(request_header);
    result.status = status;
    result.error_code = errorCode;

    std::uint8_t responsePayload[kUnsubscribeResultSize];
    std::size_t written = 0U;
    if (btp::encode_unsubscribe_result(result, responsePayload, sizeof(responsePayload), &written) !=
        btp::MessageError::Ok) {
        return false;
    }
    return endpoint_->send_logical(btp::MessageType::Control, kUnsubscribeResultObjectId, responsePayload,
                                   written, timestamp_us, seal, seal_context);
}
