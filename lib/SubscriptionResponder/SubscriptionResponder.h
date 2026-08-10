#ifndef SUBSCRIPTION_RESPONDER_H
#define SUBSCRIPTION_RESPONDER_H

#include <cstddef>
#include <cstdint>

#include <btp/codec.hpp>

class BtpEndpoint;
class TelemetryPublisher;

// Answers CONTROL/SUBSCRIBE and CONTROL/UNSUBSCRIBE
// (bally_protocol/docs/COMMANDS_AND_ACTIONS.md section 7) for this robot's
// two static topics. Deliberately pure C++ (no Arduino/FreeRTOS), same shape
// as ManifestResponder, so it links into env:native.
//
// All subscription/rate bookkeeping lives in TelemetryPublisher (subscribe/
// unsubscribe/expire_subscriptions/topic_active/topic_period_us) -- this
// class only decodes the wire request, calls into that single source of
// truth, and encodes the wire result. That mirrors ManifestResponder reusing
// TelemetryPublisher::schemas() so the manifest can never drift from what is
// published: here, what SUBSCRIBE_RESULT grants can never drift from what
// sampleTelemetry() actually does, because they read the same table.
//
// This robot has exactly one authorized peer (the dongle, enforced upstream
// by btp_command::authorized_source before any frame reaches this class), so
// "aggregating multiple clients" is entirely the dongle's job (topico 17
// PASSO 3/5); a leaf node only ever sees one requester per topic.
class SubscriptionResponder {
public:
    static constexpr std::uint16_t kSubscribeObjectId = 0x0005U;
    static constexpr std::uint16_t kSubscribeResultObjectId = 0x0006U;
    static constexpr std::uint16_t kUnsubscribeObjectId = 0x0007U;
    static constexpr std::uint16_t kUnsubscribeResultObjectId = 0x0008U;

    void configure(BtpEndpoint& endpoint, TelemetryPublisher& publisher) noexcept;

    // Parses a CONTROL/SUBSCRIBE or CONTROL/UNSUBSCRIBE payload (already
    // reassembled if needed by the caller, same convention as
    // ManifestResponder::handle_request) and sends the matching *_RESULT
    // response over `endpoint`. Returns false only when the payload was too
    // short to parse at all (caller counts it as a drop, same as any other
    // malformed CONTROL frame); a well-formed request always gets an answer.
    bool handle_subscribe(const btp::Header& request_header, btp::ByteView payload,
                          std::uint64_t timestamp_us) noexcept;
    bool handle_unsubscribe(const btp::Header& request_header, btp::ByteView payload,
                            std::uint64_t timestamp_us) noexcept;

private:
    BtpEndpoint* endpoint_ = nullptr;
    TelemetryPublisher* publisher_ = nullptr;
};

#endif  // SUBSCRIPTION_RESPONDER_H
