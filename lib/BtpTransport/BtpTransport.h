#ifndef BTP_TRANSPORT_H
#define BTP_TRANSPORT_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <btp/codec.hpp>
#include <btp/endpoint.hpp>
#include <btp/messages.hpp>
#include <btp/session.hpp>
#include <btp/stream.hpp>

// AEAD sealer for channel C (dongle<->robot, key L -- see
// include/bally_channels.h and lib/RadioSeal, the only real implementation
// of this). This is btp::EndpointSealFn (BTP/include/btp/endpoint.hpp) under a
// project-local name: file scope, not a member of BtpEndpoint, on purpose --
// StatusReporter.h and CommandProcessor.h need to name this type while only
// forward-declaring `class BtpEndpoint`, which a nested member typedef would
// not survive.
//
// Called ONCE on the whole logical message, before any fragmenting -- never
// once per fragment. The associated data is the CANONICAL logical header
// (FRAGMENTED cleared, fragment_index 0, fragment_count 1 --
// BTP/docs/encryption.md section 5), and fragment_count for the actual
// frames on the wire has to be computed from the SEALED size (payload_size +
// kBtpAeadTagSize), not the plaintext size, or a message that fits in one
// ESP-NOW fragment unsealed could need two once sealed and silently lose its
// tail. `header.flags` already carries btp::kFlagEncrypted when this is
// called (the AAD is computed from the header AS GIVEN, so the flag has to
// be set before sealing, not patched in after). `out` has room for exactly
// `payload_size + kBtpAeadTagSize` octets.
//
// Returns false when there is no key to seal with, or the seal genuinely
// fails. Every send path then transmits NOTHING AT ALL -- fail-closed: an
// unsealed frame must never reach the radio as a fallback.
//
// nullptr (the default everywhere below) means "do not seal" -- the state
// every send path is in before it is configured, and the mode the native
// unit tests exercise deliberately (cleartext BTP framing). Real firmware
// always passes a real SealFn: RadioSeal::seal (key L, channel C) for
// StatusReporter and CommandProcessor::send_result's channel-C replies,
// RadioSeal::seal_e (key E, channel B) for CommandProcessor::send_result's
// channel-B replies.
using BtpSealFn = btp::EndpointSealFn;

// BTP/docs/encryption.md section 2: sealing always grows the payload by
// exactly this many octets, regardless of cipher.
constexpr std::size_t kBtpAeadTagSize = btp::kEndpointAeadTagSize;

// The robot's BTP transmit endpoint: btp::Endpoint (identity, the outgoing
// sequence counter and the seal -> fragment -> encode pipeline -- all now in
// BTP/src/endpoint.cpp, tested by BTP/tests/test_endpoint.cpp) plus the two
// things that stay project-local:
//
//   * a STORED send callback. btp::Endpoint takes the transport as a per-call
//     argument; every producer in this firmware sends the same way (through
//     TxScheduler, installed once in main.cpp), so the callback lives here and
//     the call sites do not thread it. The legacy no-context overload is kept
//     because the native tests install a bare `bool(*)(bytes, size)` capture.
//   * the ESP-NOW-only shape. Every frame this robot originates is a channel-C
//     or channel-B ESP-NOW datagram, so the wrappers below pin
//     btp::kEspNowTransport and keep the old positional
//     (type, object_id, ...) signatures, so CommandProcessor / TelemetryPublisher
//     / StatusReporter / ManifestCatalog / Logger / TerminalResponder are
//     untouched.
// btp::Endpoint owns the identity and the outgoing sequence counter, and
// there must be exactly ONE of them live for this robot's source_id, or two
// producers could hand out the same sequence number -- the dongle's dedup
// keys on (source_id, boot_id, sequence). Historically BtpEndpoint WAS one
// (public inheritance); it now WRAPS one instead, defaulting to a private
// own_endpoint_ so every existing caller/test that builds a bare
// `BtpEndpoint endpoint; endpoint.configure(...)` keeps working unchanged --
// bind() below is the only new thing, and only the firmware composition root
// calls it, once btp::Node exists, to point this wrapper at NODE'S endpoint
// instead (topico 2.11+'s Round 2: btp::Node owns receive + MANIFEST_DATA
// serving; this class stays the one send path every producer already uses).
class BtpEndpoint {
public:
    using SealFn = BtpSealFn;
    using SendCallback = bool (*)(const std::uint8_t* data, std::size_t size);
    using ContextSendCallback = btp::EndpointSendFn;

    static constexpr std::size_t kAeadTagSize = btp::kEndpointAeadTagSize;

    // Identity + sequence forwarders -- see own_endpoint_/active_ below.
    bool configure(std::uint32_t source_id, std::uint32_t boot_id) noexcept {
        return active_->configure(source_id, boot_id);
    }
    bool configured() const noexcept { return active_->configured(); }
    std::uint32_t source_id() const noexcept { return active_->source_id(); }
    std::uint32_t boot_id() const noexcept { return active_->boot_id(); }
    bool reserve_sequence(std::uint32_t* sequence_out) noexcept {
        return active_->reserve_sequence(sequence_out);
    }
    bool try_reserve_sequence(std::uint32_t* sequence_out) noexcept {
        return active_->try_reserve_sequence(sequence_out);
    }

    // Points every send below at `shared` instead of this wrapper's own,
    // never-configured btp::Endpoint -- called once, after btp::Node::begin()
    // has configured `shared`'s identity from the exact same (source_id,
    // boot_id) this class's own configure() would otherwise have taken.
    // Never call configure() on this object again afterward: it would
    // reconfigure (and reset the sequence counter of) whichever btp::Endpoint
    // `active_` currently points at, shared or not.
    void bind(btp::Endpoint& shared) noexcept { active_ = &shared; }

    // ESP-NOW remains the default for the existing robot path. A direct TCP
    // session can select its own BTP limits before the first send, reusing the
    // same endpoint sequencing, sealing and fragmentation pipeline.
    void set_transport(btp::TransportLimits transport) noexcept {
        transport_ = transport;
    }

    // Bounds the sealed[] scratch a sealed send_logical() cuts fragments from.
    // The largest logical message this firmware sends is the UTF-8
    // system.monitor telemetry document
    // (TelemetryPublisher::kMaxSystemMonitorPayloadSize) -- a fragmented
    // COMMAND_RESULT / STATUS / MANIFEST_DATA never gets close.
    // node_'s own receive-slot capacity (kNodeSlotBytes, BallyRobot.h) and
    // btp_command::kMaxLogicalRequestSize are independent receive / command
    // bounds and are NOT tied to this one.
    static constexpr std::size_t kMaxLogicalPayloadSize = 1920U;

    void set_send_callback(SendCallback callback) noexcept;
    void set_send_callback(ContextSendCallback callback, void* context) noexcept;

    // A sealed message is sealed ONCE over the canonical header, then sliced
    // for the wire from the sealed bytes; `payload` / `payload_size` are always
    // the PLAINTEXT. A false from `seal` fails the whole send closed.
    bool send_logical(btp::MessageType type, std::uint16_t object_id,
                      const std::uint8_t* payload, std::size_t payload_size,
                      std::uint64_t timestamp_us, SealFn seal = nullptr,
                      void* seal_context = nullptr) noexcept;

    // Same pipeline with a sequence a non-blocking producer already reserved.
    bool send_logical_reserved(btp::MessageType type, std::uint16_t object_id,
                               std::uint32_t sequence,
                               const std::uint8_t* payload,
                               std::size_t payload_size,
                               std::uint64_t timestamp_us, SealFn seal = nullptr,
                               void* seal_context = nullptr) const noexcept;

    // Exactly one physical frame. When `seal` is non-null this frame MUST be
    // the whole logical message (fragment_count == 1): the AEAD tag covers the
    // whole logical payload, never a slice of one.
    bool encode_fragment(btp::MessageType type, std::uint16_t object_id,
                         std::uint32_t sequence, std::uint64_t timestamp_us,
                         const std::uint8_t* payload, std::size_t payload_size,
                         std::uint8_t fragment_index,
                         std::uint8_t fragment_count, std::uint8_t* output,
                         std::size_t output_capacity, std::size_t* bytes_written,
                         SealFn seal = nullptr,
                         void* seal_context = nullptr) const noexcept;

    bool send_fragment(btp::MessageType type, std::uint16_t object_id,
                       std::uint32_t sequence, std::uint64_t timestamp_us,
                       const std::uint8_t* payload, std::size_t payload_size,
                       std::uint8_t fragment_index, std::uint8_t fragment_count,
                       SealFn seal = nullptr,
                       void* seal_context = nullptr) const noexcept;

    // Used by SD playback after a complete BTP frame has been decoded.
    bool send_encoded(const std::uint8_t* frame,
                      std::size_t frame_size) const noexcept;

private:
    static bool default_send(void*, const std::uint8_t*, std::size_t) noexcept;
    static bool legacy_send(void* context, const std::uint8_t* data,
                            std::size_t size) noexcept;

    std::atomic<SendCallback> legacy_callback_{nullptr};
    std::atomic<void*> send_context_{nullptr};
    std::atomic<ContextSendCallback> send_callback_{&default_send};

    // own_endpoint_ before active_ on purpose: active_'s default member
    // initializer takes its address, which needs own_endpoint_ already built.
    // Every method above goes through active_, never own_endpoint_ directly,
    // so bind() redirecting it is the only difference the firmware path sees.
    btp::Endpoint own_endpoint_;
    btp::Endpoint* active_ = &own_endpoint_;
    btp::TransportLimits transport_ = btp::kEspNowTransport;
};

// Builds the explicit BUSY/CAPACITY_EXHAUSTED HELLO_RESULT a TCP BTP server
// must send a SECOND, concurrent connection attempt while one control
// session is already active for this robot (BTP/docs/session-and-terminal.md
// section 2.4/3.4) -- see fragmentation-and-transports.md section 9.6 and
// the TAREFAS_TCP_BLE_ANDROID.txt T21/T22 notes this implements. btp::Node/
// btp::Session never construct this status themselves (confirmed by reading
// BTP/src/session.cpp: build_hello_result() only ever writes Success or
// Unsupported) -- rejecting a second session is entirely this responder's
// job, not the library's.
//
// Deliberately transport-free (no lwip/FreeRTOS/ESP-IDF dependency, like the
// rest of this file) so it is exercised by the native test suite the same
// way btp_command::parse_request already is; TcpBtpServer/ROBOT feed it raw
// bytes read off the pending socket and, on success, send frame_out()
// verbatim before closing that connection.
//
// One instance handles ONE pending connection at a time -- reset() (or a
// fresh instance) before reusing it for the next one; there is no per-byte
// concurrency guard, same single-consumer posture as btp::SerialDecoder,
// which this wraps.
class TcpBusyResponder {
public:
    TcpBusyResponder() noexcept;

    // decoder_ holds pointers into THIS object's own cobs_buffer_/
    // decoded_buffer_ -- a copy or move would leave it aliasing the
    // source's buffers instead of its own.
    TcpBusyResponder(const TcpBusyResponder&) = delete;
    TcpBusyResponder& operator=(const TcpBusyResponder&) = delete;

    // Forgets whatever partial datagram was being collected. Call once per
    // newly-accepted pending connection, before the first feed().
    void reset() noexcept;

    // Feeds one chunk of raw bytes exactly as read off the pending TCP
    // socket (COBS-framed, 0x00-delimited -- see BTP/include/btp/stream.hpp,
    // the same wire shape used on serial). `local` is this robot's own HELLO
    // advertisement (the same one the real, active session was/will be
    // enabled with); `source_id`/`boot_id` are this robot's protocol
    // identity, used to address the reply frame.
    //
    // Returns true once a complete datagram decoded as a HELLO this
    // responder would otherwise have accepted: `*frame_out`/`*frame_size`
    // then point at a ready-to-send, already COBS-encoded and delimited BTP
    // frame (status=BUSY, error_code=CAPACITY_EXHAUSTED, every negotiated
    // limit zeroed, `selected_version` carried over from the negotiation
    // that would otherwise have accepted it -- section 2.4). The caller
    // sends it verbatim and then closes the socket; this object must not be
    // fed again without reset() first.
    //
    // Returns false when nothing conclusive happened yet: not enough bytes,
    // a COBS/frame decode error, or a datagram that is not a HELLO (a
    // non-HELLO first message is itself a protocol violation the caller's
    // own deadline -- 2000 ms, session-and-terminal.md section 3.4 -- is
    // what eventually gives up on, not this class). Safe to call again with
    // more bytes after a false return, without reset(), as long as the
    // stream has not been resynchronized.
    bool feed(const std::uint8_t* data, std::size_t size,
             const btp::Hello& local, std::uint32_t source_id,
             std::uint32_t boot_id, std::uint64_t now_ms,
             const std::uint8_t** frame_out, std::size_t* frame_size) noexcept;

private:
    // Sized off btp::kSerialTransport (kSerialMaxCobsBlockSize/
    // kSerialMaxFrameSize) even though the connection is TCP: a HELLO
    // datagram is a few dozen octets, orders of magnitude under either
    // ceiling, and btp::SerialDecoder is hardcoded to kSerialTransport
    // internally (BTP/src/stream.cpp) regardless of the buffer sizes handed
    // to it -- reusing it here is safe and avoids hand-rolling COBS framing
    // a second time.
    std::uint8_t cobs_buffer_[btp::kSerialMaxCobsBlockSize];
    std::uint8_t decoded_buffer_[btp::kSerialMaxFrameSize];
    btp::SerialDecoder decoder_;

    // The reply this class builds: HELLO_RESULT's payload (<=
    // btp::kSessionMaxReplySize) framed into one encode()d BTP frame, then
    // COBS-encoded with its leading/trailing 0x00 delimiters. Sized with
    // comfortable headroom over the worst case (52-octet payload -> ~60
    // octets encoded -> ~65 octets COBS-encoded).
    static constexpr std::size_t kReplyFrameCapacity = 128U;
    std::uint8_t reply_frame_[kReplyFrameCapacity];
};

// Link framing for the two DIRECT stream transports (TCP and BLE): both
// carry 0x00 || COBS(frame) || 0x00 over a boundary-less byte pipe
// (BTP/docs/fragmentation-and-transports.md section 8.2 for BLE, the same
// serial-style framing TraceView's BtpSession uses for TCP). btp::Node does
// NOT do this itself -- its receive() wants one whole frame (see node.hpp's
// "LINK framing" note) -- so the composition layer owns it, exactly like
// TcpBusyResponder above already does for the pending-connection path.
//
// Decoder side: one instance per link, single consumer (the task that owns
// that link's receive callback). Buffers are allocated on the first
// reset(), never in the constructor: ROBOT is a static singleton, and a
// ~8 KB allocation that size lands in PSRAM (CONFIG_SPIRAM_USE_MALLOC) only
// once the heap is up -- and keeping it out of .bss keeps it off internal
// RAM that NimBLE/Wi-Fi need.
class CobsStreamDecoder {
public:
    CobsStreamDecoder() noexcept = default;
    ~CobsStreamDecoder();

    // decoder_ points into storage_ -- copying would alias it.
    CobsStreamDecoder(const CobsStreamDecoder&) = delete;
    CobsStreamDecoder& operator=(const CobsStreamDecoder&) = delete;

    // Allocates the buffers if needed and drops any partial frame. Call once
    // per new connection, before the first feed(). False only when the
    // allocation failed; feed() is then a no-op.
    bool reset() noexcept;

    // Feeds raw link bytes; calls on_frame(const btp::DecodedFrame&) for each
    // complete, CRC-valid frame, in order. The frame view is only valid for
    // the duration of that call. Corrupt/oversized blocks are skipped (the
    // decoder resynchronizes on the next 0x00 by itself).
    template <typename OnFrame>
    void feed(const std::uint8_t* data, std::size_t size, OnFrame&& on_frame) noexcept {
        if (decoder_ == nullptr || data == nullptr) return;
        for (std::size_t i = 0U; i < size; ++i) {
            btp::DecodedFrame decoded{};
            if (decoder_->push(data[i], &decoded).event == btp::SerialDecodeEvent::Frame) {
                on_frame(decoded);
            }
        }
    }

private:
    // Same kSerialTransport sizing (and the same reason) as
    // TcpBusyResponder's own buffers: btp::SerialDecoder is hardcoded to it.
    static constexpr std::size_t kStorageSize =
        btp::kSerialMaxCobsBlockSize + btp::kSerialMaxFrameSize;
    std::uint8_t* storage_ = nullptr;
    btp::SerialDecoder* decoder_ = nullptr;
};

// Encoder side: the worst-case size of 0x00 || COBS(frame) || 0x00 for a
// frame of `frame_size` octets, and the encoding itself. False on a null
// argument or when `capacity` is too small.
std::size_t cobs_stream_capacity(std::size_t frame_size) noexcept;
bool cobs_stream_encode(const std::uint8_t* frame, std::size_t frame_size,
                        std::uint8_t* out, std::size_t capacity,
                        std::size_t* written) noexcept;

namespace btp_command {

constexpr std::uint16_t kCommandRequestObjectId = 0x0001U;
constexpr std::uint16_t kShellActionId = 0x0001U;
constexpr std::uint16_t kShellActionVersion = 0x0001U;
// A no-op, side-effect-free action a peer can send purely to measure RTT
// (round-trip to a COMMAND_RESULT and back) -- answered directly by
// CommandProcessor::intake, never reaching the shell queue.
constexpr std::uint16_t kPingActionId = 0x0002U;
constexpr std::uint16_t kPingActionVersion = 0x0001U;
constexpr std::size_t kRequestPrefixSize = 20U;
constexpr std::size_t kMaxShellCommandSize = 512U;
constexpr std::size_t kMaxLogicalRequestSize =
    kRequestPrefixSize + kMaxShellCommandSize;

enum class ParseError : std::uint8_t {
    Ok,
    WrongType,
    WrongObject,
    InvalidEnvelope,
    PayloadTooShort,
    WrongTarget,
    InvalidAction,
    InvalidFlags,
    SizeMismatch,
    ParametersTooLarge,
    UnsupportedAction,
    InvalidShellText,
    OutputTooSmall
};

struct RequestView {
    std::uint32_t target_source_id;
    std::uint32_t target_boot_id;
    std::uint16_t action_id;
    std::uint16_t action_version;
    btp::ByteView parameters;
};

ParseError parse_request(const btp::Header& header,
                         btp::ByteView payload,
                         std::uint32_t local_source_id,
                         std::uint32_t local_boot_id,
                         RequestView* request_out) noexcept;

ParseError copy_shell_command(const RequestView& request,
                              char* output,
                              std::size_t output_capacity) noexcept;

std::uint32_t source_id_from_mac(const std::uint8_t mac[6]) noexcept;

// Radio-level filter only, NOT authentication -- see the long comment on the
// definition in BtpTransport.cpp. The claimed_source_id parameter this used
// to take was removed rather than left unused on purpose: an ignored
// parameter would let a call site keep passing a source_id and keep
// believing it is being checked, which is precisely the mistake this change
// makes possible. Removing it turns every call site into a compile error
// until someone reads why.
bool authorized_source(const std::uint8_t expected_mac[6],
                       const std::uint8_t received_mac[6]) noexcept;

const char* parse_error_string(ParseError error) noexcept;

}  // namespace btp_command

#endif  // BTP_TRANSPORT_H
