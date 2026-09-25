#ifndef STREAM_FRAMING_H
#define STREAM_FRAMING_H

#include <cstddef>
#include <cstdint>

#include <btp/codec.hpp>
#include <btp/stream.hpp>

// The wire framing of a direct TCP / BLE link: every BTP frame travels as
// 0x00 || COBS(frame) || 0x00 inside an ordered, boundary-less byte stream
// (BTP/docs/fragmentation-and-transports.md section 8.2 for BLE, section 9
// for TCP -- the same framing serial uses, section 6.1). TraceView frames
// both transports this way (BtpSession::Framing::CobsStream); a socket read
// or a GATT write is NOT one frame -- it may hold part of one, or several.
//
// Transport-free (no lwip / NimBLE / FreeRTOS), so the native suite tests it
// directly (test/test_stream_framing). TcpBtpServer / BleBtpServer stay
// BTP-unaware and move opaque byte units; ROBOT wraps every outgoing frame
// with encode_stream_frame() and feeds every incoming chunk through one
// StreamDeframer per live connection.
namespace stream_framing {

// Every frame this robot sends on a direct link comes out of btp::Endpoint,
// which since BTP 2.46.0 never emits a frame past the 250-octet ESP-NOW
// ceiling, whatever the transport. Its COBS encoding plus both delimiters
// fits in kMaxWireSize.
constexpr std::size_t kMaxFrameSize = btp::kEspNowMaxFrameSize;
constexpr std::size_t kMaxWireSize = kMaxFrameSize + kMaxFrameSize / 254U + 1U + 2U;

// Writes 0x00 || COBS(frame) || 0x00 into `out`. False, writing nothing
// useful, for an empty frame or one that does not fit `capacity` -- the
// caller then sends nothing (a partial frame would desynchronize the
// peer's decoder for everything after it).
bool encode_stream_frame(const std::uint8_t* frame, std::size_t size,
                         std::uint8_t* out, std::size_t capacity,
                         std::size_t* written) noexcept;

}  // namespace stream_framing

// Reassembles BTP frames out of a direct link's byte stream. One instance per
// live connection: it starts unsynchronized (discards bytes until the first
// 0x00, section 6.3) and must be reset() -- or replaced -- for the next
// connection. Owns btp::SerialDecoder's full-size buffers (~8.2 KB, the
// serial ceilings SerialDecoder insists on), which is why ROBOT allocates one
// per connection instead of keeping one per link in static RAM.
//
// Not thread-safe: feed() from the one task that receives on this link.
class StreamDeframer {
public:
    // Called once per complete, CRC-valid frame, synchronously from inside
    // feed(). `frame` is valid only for the duration of the call.
    using FrameFn = void (*)(void* context, const btp::DecodedFrame& frame);

    struct Stats {
        std::uint32_t frames;
        std::uint32_t cobs_errors;   // a block that is not valid COBS
        std::uint32_t frame_errors;  // valid COBS, but btp::decode rejected it (CRC, header)
        std::uint32_t overflows;     // a block past the serial ceiling, discarded whole
    };

    StreamDeframer() noexcept;

    StreamDeframer(const StreamDeframer&) = delete;
    StreamDeframer& operator=(const StreamDeframer&) = delete;

    void reset() noexcept;

    // Pushes `size` raw stream bytes; calls `on_frame` for every frame they
    // complete, in order. Returns how many frames that was.
    std::size_t feed(const std::uint8_t* data, std::size_t size, FrameFn on_frame,
                     void* context) noexcept;

    const Stats& stats() const noexcept { return stats_; }

private:
    // encoded_ / decoded_ before decoder_: it is built pointing at them.
    std::uint8_t encoded_[btp::kSerialMaxCobsBlockSize];
    std::uint8_t decoded_[btp::kSerialMaxFrameSize];
    btp::SerialDecoder decoder_;
    Stats stats_;
};

#endif  // STREAM_FRAMING_H
