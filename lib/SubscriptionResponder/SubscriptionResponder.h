#ifndef SUBSCRIPTION_RESPONDER_H
#define SUBSCRIPTION_RESPONDER_H

#include <cstddef>
#include <cstdint>

#include <BtpTransport.h>
#include <bally_channels.h>
#include <btp/codec.hpp>

class BtpEndpoint;
class TelemetryPublisher;

// Answers CONTROL/SUBSCRIBE and CONTROL/UNSUBSCRIBE
// (BTP/docs/commands.md section 4) for this robot's
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
// This robot has exactly one authorized *radio* peer (the dongle, enforced
// upstream by btp_command::authorized_source before any frame reaches this
// class), but that peer is a gateway: the (source_id, boot_id) in the
// envelope identifies the subscriber session, and nothing stops it from
// forwarding several independent desktop sessions. Subscriptions are
// therefore keyed by (session, topic_id) in TelemetryPublisher and the topic
// only stops being published when the last of them goes away (topico 17
// PASSO 5).
class SubscriptionResponder {
public:
    static constexpr std::uint16_t kSubscribeObjectId = 0x0005U;
    static constexpr std::uint16_t kSubscribeResultObjectId = 0x0006U;
    static constexpr std::uint16_t kUnsubscribeObjectId = 0x0007U;
    static constexpr std::uint16_t kUnsubscribeResultObjectId = 0x0008U;

    // `seal_link`/`seal_endpoint` follow CommandProcessor::configure()'s exact
    // contract: each *_RESULT is sealed with whichever one matches the
    // ORIGINAL request's channel (see the `channel` parameter below and
    // bally_channels.h), and send fails closed -- never with the other
    // channel's key, never in the clear -- whenever at least one of the two
    // has been configured. Both left nullptr (the default) means no
    // encryption at all, exactly as before this parameter existed; that is
    // what the native unit tests exercise.
    void configure(BtpEndpoint& endpoint, TelemetryPublisher& publisher,
                   BtpSealFn seal_link = nullptr, void* seal_link_context = nullptr,
                   BtpSealFn seal_endpoint = nullptr,
                   void* seal_endpoint_context = nullptr) noexcept;

    // Parses a CONTROL/SUBSCRIBE or CONTROL/UNSUBSCRIBE payload (already
    // reassembled if needed by the caller, same convention as
    // ManifestResponder::handle_request) and sends the matching *_RESULT
    // response over `endpoint`. Returns false only when the payload was too
    // short to parse at all (caller counts it as a drop, same as any other
    // malformed CONTROL frame); a well-formed request always gets an answer.
    //
    // `channel` is the caller's classification of the request (see
    // bally_channels.h::channel_of_peer), made BEFORE this call since only
    // the caller has the header's cleartext source_id and knows which key
    // opened the payload -- same convention as
    // CommandProcessor::intake()'s own `channel` parameter. Defaults to
    // C_Link so a caller that never passes it -- every existing native test
    // -- keeps today's behavior verbatim.
    bool handle_subscribe(const btp::Header& request_header, btp::ByteView payload,
                          std::uint64_t timestamp_us,
                          bally::Channel channel = bally::Channel::C_Link) noexcept;
    bool handle_unsubscribe(const btp::Header& request_header, btp::ByteView payload,
                            std::uint64_t timestamp_us,
                            bally::Channel channel = bally::Channel::C_Link) noexcept;

private:
    // Picks seal_link_/seal_endpoint_ by `channel`, matching
    // CommandProcessor::send_result()'s fail-closed rule: returns false
    // (never send unsealed or under the wrong key) when the matching one is
    // null but at least one of the two has been configured.
    bool seal_for(bally::Channel channel, BtpSealFn* seal_out, void** context_out) const noexcept;

    BtpEndpoint* endpoint_ = nullptr;
    TelemetryPublisher* publisher_ = nullptr;
    BtpSealFn seal_link_ = nullptr;
    void* seal_link_context_ = nullptr;
    BtpSealFn seal_endpoint_ = nullptr;
    void* seal_endpoint_context_ = nullptr;
};

#endif  // SUBSCRIPTION_RESPONDER_H
