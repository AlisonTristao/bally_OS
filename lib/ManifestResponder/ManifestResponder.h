#ifndef MANIFEST_RESPONDER_H
#define MANIFEST_RESPONDER_H

#include <cstddef>
#include <cstdint>

#include <btp/codec.hpp>

class BtpEndpoint;

// Answers CONTROL/MANIFEST_REQUEST (bally_protocol/docs/COMMANDS_AND_ACTIONS.md
// section 6) by describing this robot's own two static schemas, taken
// verbatim from TelemetryPublisher::schemas() -- the same table the robot
// already uses to pack samples, so the manifest can never drift from what is
// actually published on the wire.
//
// Deliberately pure C++ (no Arduino/FreeRTOS), same shape as BtpTransport, so
// it links into env:native if a future topico wants to test it there. This
// robot has exactly one static, compile-time catalog (no runtime schema
// changes), so config_revision is a fixed constant (1) rather than a
// persisted counter -- "a revisao e monotonica e comeca em 1" is satisfied
// trivially by never changing it within one firmware build.
//
// Every response is single-shot: MANIFEST_REQUEST addressed to this boot (or
// to the wildcard target 0, which for a leaf node with no gateway role below
// it just means "describe yourself") always describes exactly one source
// (this robot), so catalog_index=0/catalog_count=1/CATALOG_COMPLETE is set on
// every response, per COMMANDS_AND_ACTIONS.md section 6.2's "em requisicao
// direcionada".
class ManifestResponder {
public:
    static constexpr std::uint16_t kManifestRequestObjectId = 0x0003U;
    static constexpr std::uint16_t kManifestDataObjectId = 0x0004U;
    static constexpr std::uint16_t kManifestFormatVersion = 1U;
    static constexpr std::uint32_t kConfigRevision = 1U;
    static constexpr std::uint8_t kSourceRoleRobot = 0x01U;
    // Comfortably covers the two current schemas (protocol.test, robot.state)
    // with headroom for a few more fields/topics; well under the manifest's
    // 49152-octet wire ceiling (COMMANDS_AND_ACTIONS.md section 13).
    // BtpEndpoint::send_logical fragments this across ESP-NOW frames
    // (kEspNowMaxPayloadSize=210) automatically -- reusing the same
    // btp::fragment_count/make_fragment machinery every other logical send in
    // this firmware already uses, not a second scheme.
    static constexpr std::size_t kMaxManifestPayloadSize = 512U;

    // uuid is copied (16 bytes, opaque, stable identity for this boot -- see
    // BallyRobot.cpp's configureProtocolIdentity(), derived from the MAC).
    void configure(BtpEndpoint& endpoint, const std::uint8_t uuid[16]) noexcept;

    // Parses a CONTROL/MANIFEST_REQUEST payload (already validated as
    // unfragmented or reassembled by the caller) and sends the matching
    // MANIFEST_DATA response over `endpoint`. Returns false when the payload
    // was too short to parse at all (caller should count it as a drop, same
    // as any other malformed CONTROL frame); a well-formed request always
    // gets an answer (possibly NOT_FOUND/STALE_TARGET_BOOT), which still
    // returns true.
    bool handle_request(const btp::Header& request_header,
                        btp::ByteView payload,
                        std::uint64_t timestamp_us) noexcept;

private:
    BtpEndpoint* endpoint_ = nullptr;
    std::uint8_t uuid_[16] = {};
};

#endif  // MANIFEST_RESPONDER_H
