#include "TerminalResponder.h"

#include <cstdio>
#include <utility>

namespace {

// One sealed TERMINAL_OUT chunk fits a single ESP-NOW frame: payload space
// minus the AEAD tag. Each chunk is sent as its own logical message, the same
// way Logger::send_log_direct() sends sealed LOG -- an AEAD tag cannot cover a
// slice of a logical payload, so one frame == one seal.
constexpr std::size_t kSealedChunk = btp::kEspNowMaxPayloadSize - kBtpAeadTagSize;
constexpr std::size_t kCleartextChunk = btp::kEspNowMaxPayloadSize;

// Prefix each line of a command's captured output with "! " and CR+LF, so it
// reads the same in the terminal as bally_dongle's ShellOutput::renderResponse
// output. `text` is the newline-separated capture from Logger.
std::string render_command_output(const std::string& text, std::uint8_t status) {
    std::string out;
    out.reserve(text.size() + 16U);

    std::size_t start = 0U;
    while (start <= text.size()) {
        std::size_t nl = text.find('\n', start);
        const bool last = (nl == std::string::npos);
        const std::size_t end = last ? text.size() : nl;

        // Trim trailing CR the shell may have left on the line.
        std::size_t line_end = end;
        while (line_end > start && (text[line_end - 1] == '\r' || text[line_end - 1] == ' ')) {
            --line_end;
        }
        if (line_end > start) {
            out += "! ";
            out.append(text, start, line_end - start);
            out += "\r\n";
        }
        if (last) {
            break;
        }
        start = nl + 1U;
    }

    if (status != 0U) {
        char note[32];
        std::snprintf(note, sizeof(note), "! [status=%u]\r\n", static_cast<unsigned>(status));
        out += note;
    }
    return out;
}

}  // namespace

void TerminalResponder::lock() noexcept {
    while (lock_.test_and_set(std::memory_order_acquire)) {
    }
}

void TerminalResponder::unlock() noexcept {
    lock_.clear(std::memory_order_release);
}

void TerminalResponder::configure(BtpEndpoint& endpoint, BtpSealFn seal, void* seal_context,
                                  ShellLineEditor::CompletionProvider completion, SubmitFn submit,
                                  void* submit_context, const char* prompt) noexcept {
    endpoint_ = &endpoint;
    seal_ = seal;
    seal_context_ = seal_context;
    completion_ = std::move(completion);
    submit_ = submit;
    submit_context_ = submit_context;
    prompt_ = (prompt != nullptr) ? prompt : "";
}

TerminalResponder::Slot* TerminalResponder::find(std::uint32_t source_id,
                                                 std::uint32_t boot_id) noexcept {
    for (Slot& s : slots_) {
        if (s.used && s.source_id == source_id && s.boot_id == boot_id) {
            return &s;
        }
    }
    return nullptr;
}

TerminalResponder::Slot* TerminalResponder::find_or_alloc(std::uint32_t source_id,
                                                          std::uint32_t boot_id,
                                                          std::uint64_t now_us) noexcept {
    if (Slot* existing = find(source_id, boot_id)) {
        return existing;
    }

    Slot* victim = nullptr;
    for (Slot& s : slots_) {
        if (!s.used) {
            victim = &s;
            break;
        }
        if (victim == nullptr || s.last_used_us < victim->last_used_us) {
            victim = &s;
        }
    }
    if (victim == nullptr) {
        return nullptr;
    }
    if (victim->used) {
        origins_evicted_.fetch_add(1U, std::memory_order_relaxed);
    }

    victim->used = true;
    victim->source_id = source_id;
    victim->boot_id = boot_id;
    victim->last_used_us = now_us;
    victim->editor.reset();
    victim->editor.setCompletionProvider(completion_);
    victim->prompt_painted = false;
    victim->in.clear();
    victim->pending_out.clear();
    victim->result_ready = false;
    victim->awaiting_result = false;
    origins_seen_.fetch_add(1U, std::memory_order_relaxed);
    return victim;
}

void TerminalResponder::on_terminal_in(const btp::Header& header, btp::ByteView plaintext) noexcept {
    if (plaintext.data == nullptr || plaintext.size == 0U) {
        return;
    }

    lock();
    Slot* s = find_or_alloc(header.source_id, header.boot_id, header.timestamp_us);
    if (s != nullptr) {
        const std::size_t room = (s->in.size() < kMaxPendingIn) ? (kMaxPendingIn - s->in.size()) : 0U;
        const std::size_t take = (plaintext.size < room) ? plaintext.size : room;
        if (take > 0U) {
            s->in.append(reinterpret_cast<const char*>(plaintext.data), take);
        }
        if (take < plaintext.size) {
            in_bytes_dropped_.fetch_add(static_cast<std::uint32_t>(plaintext.size - take),
                                        std::memory_order_relaxed);
        }
    }
    unlock();
}

void TerminalResponder::deliver_command_output(std::uint32_t source_id, std::uint32_t boot_id,
                                               const char* text, std::uint8_t status) noexcept {
    lock();
    Slot* s = find(source_id, boot_id);
    if (s != nullptr) {
        std::string rendered = render_command_output((text != nullptr) ? std::string(text) : std::string(),
                                                     status);
        if (rendered.size() > kMaxCommandOut) {
            rendered.resize(kMaxCommandOut);
            rendered += "\r\n! [output truncated]\r\n";
        }
        s->pending_out = std::move(rendered);
        s->result_ready = true;
    }
    unlock();
}

void TerminalResponder::pump(std::uint64_t now_us) noexcept {
    for (Slot& s : slots_) {
        // --- structural section under the lock ---
        lock();
        if (!s.used) {
            unlock();
            continue;
        }

        std::string result;
        bool have_result = false;
        if (s.result_ready) {
            result.swap(s.pending_out);
            s.result_ready = false;
            s.awaiting_result = false;
            have_result = true;
        }

        std::string input;
        if (!s.awaiting_result) {
            input.swap(s.in);
        }
        const bool need_prompt = !s.prompt_painted;
        const bool awaiting = s.awaiting_result;
        s.last_used_us = now_us;
        unlock();

        // --- editor section: only pump() touches s.editor ---
        std::string out;
        if (need_prompt) {
            s.editor.setPrompt(prompt_, out);
            s.prompt_painted = true;
        }
        if (have_result) {
            s.editor.writeResponse(result, out);
        }
        if (!awaiting) {
            if (!input.empty()) {
                s.editor.feed(input);
            }
            std::string line;
            while (s.editor.poll(out, &line)) {
                const bool ok = (submit_ != nullptr) &&
                                submit_(submit_context_, s.source_id, s.boot_id, line.c_str());
                if (ok) {
                    lines_submitted_.fetch_add(1U, std::memory_order_relaxed);
                    lock();
                    s.awaiting_result = true;
                    unlock();
                    break;  // one command at a time; leftover input stays in the editor
                }
                lines_dropped_busy_.fetch_add(1U, std::memory_order_relaxed);
                s.editor.writeResponse("! busy, command dropped", out);
            }
        }

        emit_terminal_out(out, now_us);
    }
}

void TerminalResponder::emit_terminal_out(const std::string& bytes, std::uint64_t now_us) noexcept {
    if (bytes.empty() || endpoint_ == nullptr) {
        return;
    }
    const std::size_t stride = (seal_ != nullptr) ? kSealedChunk : kCleartextChunk;
    for (std::size_t offset = 0U; offset < bytes.size(); offset += stride) {
        const std::size_t chunk = (bytes.size() - offset < stride) ? (bytes.size() - offset) : stride;
        if (endpoint_->send_logical(btp::MessageType::Terminal, kTerminalOutObjectId,
                                    reinterpret_cast<const std::uint8_t*>(bytes.data()) + offset, chunk,
                                    now_us, seal_, seal_context_)) {
            frames_out_.fetch_add(1U, std::memory_order_relaxed);
        }
    }
}

TerminalResponder::Stats TerminalResponder::stats() const noexcept {
    return Stats{
        origins_seen_.load(std::memory_order_relaxed),
        origins_evicted_.load(std::memory_order_relaxed),
        lines_submitted_.load(std::memory_order_relaxed),
        lines_dropped_busy_.load(std::memory_order_relaxed),
        frames_out_.load(std::memory_order_relaxed),
        in_bytes_dropped_.load(std::memory_order_relaxed),
    };
}
