#include "StreamFraming.h"

namespace stream_framing {

bool encode_stream_frame(const std::uint8_t* frame, std::size_t size,
                         std::uint8_t* out, std::size_t capacity,
                         std::size_t* written) noexcept {
    if (frame == nullptr || size == 0U || out == nullptr || written == nullptr ||
        capacity < 3U) {
        return false;
    }
    std::size_t body = 0U;
    if (btp::cobs_encode(frame, size, out + 1U, capacity - 2U, &body) !=
        btp::CobsError::Ok) {
        return false;
    }
    out[0] = 0x00U;
    out[1U + body] = 0x00U;
    *written = body + 2U;
    return true;
}

}  // namespace stream_framing

StreamDeframer::StreamDeframer() noexcept
    : encoded_(),
      decoded_(),
      decoder_(encoded_, sizeof(encoded_), decoded_, sizeof(decoded_)),
      stats_() {}

void StreamDeframer::reset() noexcept {
    decoder_.reset();
    stats_ = Stats();
}

std::size_t StreamDeframer::feed(const std::uint8_t* data, std::size_t size,
                                 FrameFn on_frame, void* context) noexcept {
    if (data == nullptr) return 0U;
    std::size_t frames = 0U;
    for (std::size_t i = 0U; i < size; ++i) {
        btp::DecodedFrame frame{};
        const btp::SerialDecodeResult result = decoder_.push(data[i], &frame);
        switch (result.event) {
            case btp::SerialDecodeEvent::Frame:
                ++stats_.frames;
                ++frames;
                if (on_frame != nullptr) on_frame(context, frame);
                break;
            case btp::SerialDecodeEvent::CobsError:
                ++stats_.cobs_errors;
                break;
            case btp::SerialDecodeEvent::FrameError:
                ++stats_.frame_errors;
                break;
            case btp::SerialDecodeEvent::Overflow:
                ++stats_.overflows;
                break;
            case btp::SerialDecodeEvent::None:
            case btp::SerialDecodeEvent::InvalidConfiguration:
                break;
        }
    }
    return frames;
}
