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

// Unbounded spin. Safe ONLY for a caller whose every possible lock holder runs
// at >= its own priority on the same core, so the holder always makes progress
// and releases: deliver_command_output() (shell task, the lowest-priority user)
// qualifies -- pump() (comms) and on_terminal_in() (Wi-Fi) both outrank it and
// both now hold the lock for only a handful of field ops.
void TerminalResponder::lock() noexcept {
    while (lock_.test_and_set(std::memory_order_acquire)) {
    }
}

// Bounded try, for callers that would otherwise deadlock spin-waiting on a
// LOWER-priority holder pinned to the same core: pump() (comms, priority 4)
// and on_terminal_in() (Wi-Fi, priority ~23) can both be blocked behind the
// shell task (priority 2) on core 0. kMaxLockSpins is generous because every
// critical section is now O(1) field work -- exhausting it means real trouble,
// and the caller's fallback (skip this pass / drop these bytes) beats a hang.
bool TerminalResponder::try_lock() noexcept {
    for (unsigned spins = 0U; spins < kMaxLockSpins; ++spins) {
        if (!lock_.test_and_set(std::memory_order_acquire)) {
            return true;
        }
    }
    return false;
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
    // Touch only the lock_-guarded fields here. The editor and the other
    // pump()-owned fields (prompt_painted, awaiting_result, pending_line) are
    // reset by pump() itself when it observes needs_editor_reset -- this runs
    // on the Wi-Fi RX task, and ShellLineEditor::reset() is neither fast nor
    // concurrency-safe, so doing it here (while pump() may be mid-edit) is
    // exactly the race the deferral removes.
    victim->needs_editor_reset = true;
    victim->in.clear();
    victim->pending_out.clear();
    victim->async_out.clear();
    victim->result_ready = false;
    origins_seen_.fetch_add(1U, std::memory_order_relaxed);
    return victim;
}

void TerminalResponder::on_terminal_in(const btp::Header& header, btp::ByteView plaintext) noexcept {
    if (plaintext.data == nullptr || plaintext.size == 0U) {
        return;
    }

    // Wi-Fi RX task, priority ~23, pinned to core 0 alongside pump() (comms)
    // and deliver_command_output() (shell). A spin-wait here on a lock held by
    // either lower-priority task would wedge core 0 for good, so give up
    // instead -- the dropped bytes are counted, and with O(1) critical
    // sections everywhere this is effectively never reached.
    if (!try_lock()) {
        in_bytes_dropped_.fetch_add(static_cast<std::uint32_t>(plaintext.size),
                                    std::memory_order_relaxed);
        return;
    }
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
    // Render BEFORE taking lock_. This runs on the shell task (priority 2);
    // render_command_output() is an allocation loop over the whole capture.
    // Holding lock_ across it let the Wi-Fi RX task (on_terminal_in(), priority
    // ~23, same core) spin-wait on a lock this task could not release until it
    // was rescheduled -- which, being lower priority on a pinned core, never
    // happened: a hard core-0 deadlock. The critical section below is now just
    // two field writes.
    std::string rendered =
        render_command_output((text != nullptr) ? std::string(text) : std::string(), status);
    if (rendered.size() > kMaxCommandOut) {
        rendered.resize(kMaxCommandOut);
        rendered += "\r\n! [output truncated]\r\n";
    }

    lock();
    Slot* s = find(source_id, boot_id);
    if (s != nullptr) {
        s->pending_out = std::move(rendered);
        s->result_ready = true;
    }
    unlock();
}

void TerminalResponder::push_async_output(std::uint32_t source_id, std::uint32_t boot_id,
                                          const char* text) noexcept {
    if (text == nullptr || text[0] == '\0' || endpoint_ == nullptr) {
        return;
    }

    // Build the "! <line>\r\n" form OUTSIDE the lock (same reasoning as
    // deliver_command_output rendering before it locks): this runs on the
    // state-machine task and the critical section below must stay O(1).
    std::string line = "! ";
    line += text;
    while (!line.empty() &&
           (line.back() == '\r' || line.back() == '\n' || line.back() == ' ')) {
        line.pop_back();
    }
    line += "\r\n";

    // State-machine task (core 1) while pump()/on_terminal_in() run on core 0:
    // the lock holder is always on the other core making progress, so a bounded
    // try is enough and dropping the line on a miss beats spinning core 1.
    if (!try_lock()) {
        async_lines_dropped_.fetch_add(1U, std::memory_order_relaxed);
        return;
    }
    Slot* s = find(source_id, boot_id);
    if (s == nullptr) {
        // No terminal ever spoke to this robot from that origin (or its slot
        // was evicted): nothing to mirror to. The LOG frame still carries it.
        unlock();
        return;
    }
    if (s->async_out.size() + line.size() <= kMaxAsyncOut) {
        s->async_out += line;
    } else {
        async_lines_dropped_.fetch_add(1U, std::memory_order_relaxed);
    }
    unlock();
}

void TerminalResponder::pump(std::uint64_t now_us) noexcept {
    for (Slot& s : slots_) {
        // Take lock_ only to snapshot the hand-off fields on_terminal_in()
        // (Wi-Fi task) and deliver_command_output() (shell task) also touch.
        // The editor section that follows runs UNLOCKED: s.editor and the
        // pump()-owned flags below are never touched off this task, so nothing
        // there can race, and the Wi-Fi task never spin-waits on pump().
        bool used = false;
        bool needs_reset = false;
        bool have_result = false;
        std::uint32_t src = 0U;
        std::uint32_t boot = 0U;
        std::string result;
        std::string input;

        // Comms task, priority 4, core 0. deliver_command_output() on the
        // shell task (priority 2) also takes lock_, so a spin-wait here could
        // deadlock core 0. Its critical section is O(1), so a bounded try that
        // fails just means "come back next pass" -- pump() runs every comms
        // loop anyway.
        if (!try_lock()) {
            continue;
        }
        used = s.used;
        if (used) {
            s.last_used_us = now_us;
            src = s.source_id;
            boot = s.boot_id;
            needs_reset = s.needs_editor_reset;
            s.needs_editor_reset = false;
            if (s.result_ready) {
                result.swap(s.pending_out);
                s.result_ready = false;
                have_result = true;
            }
            if (!s.in.empty()) {
                input.swap(s.in);
            }
        }
        unlock();

        if (!used) {
            continue;
        }

        std::string out;

        if (needs_reset) {
            s.editor.reset();
            s.editor.setCompletionProvider(completion_);
            s.prompt_painted = false;
            s.awaiting_result = false;
            s.pending_line.clear();
        }
        if (!s.prompt_painted) {
            s.editor.setPrompt(prompt_, out);
            s.prompt_painted = true;
        }
        if (have_result) {
            s.awaiting_result = false;
            s.editor.writeResponse(result, out);
        }

        // Feed the editor EVERY pass, even while a command is still executing:
        // it must keep echoing keystrokes and completing lines, or s.in fills
        // (kMaxPendingIn) while the command runs and on_terminal_in() starts
        // dropping bytes -- a dropped '\r' wedges the line editor forever. Only
        // SUBMITTING waits on awaiting_result.
        if (!input.empty()) {
            s.editor.feed(input);
        }

        // A line typed ahead while the previous command ran: submit it now
        // that its result has landed, before pulling anything new from the
        // editor, so commands run in the order they were entered.
        if (!s.awaiting_result && !s.pending_line.empty()) {
            std::string next;
            next.swap(s.pending_line);
            submit_line(s, src, boot, next, out);
        }

        std::string line;
        while (s.editor.poll(out, &line)) {
            if (s.awaiting_result) {
                // One command already in flight. Hold exactly one line behind
                // it; a second means whole commands are arriving faster than
                // the robot's shell drains them.
                if (s.pending_line.empty()) {
                    s.pending_line.swap(line);
                } else {
                    lines_dropped_busy_.fetch_add(1U, std::memory_order_relaxed);
                    s.editor.writeResponse("! busy, command dropped", out);
                }
                continue;
            }
            submit_line(s, src, boot, line, out);
        }

        emit_terminal_out(out, now_us);
    }
}

// pump()-only helper: submit one completed line, or report the queue full.
// Never touches lock_ -- s.awaiting_result is pump-owned, src/boot are the
// values pump() snapshotted under the lock.
void TerminalResponder::submit_line(Slot& s, std::uint32_t src, std::uint32_t boot,
                                    const std::string& line, std::string& out) noexcept {
    if (submit_ != nullptr && submit_(submit_context_, src, boot, line.c_str())) {
        lines_submitted_.fetch_add(1U, std::memory_order_relaxed);
        s.awaiting_result = true;
        return;
    }
    lines_dropped_busy_.fetch_add(1U, std::memory_order_relaxed);
    s.editor.writeResponse("! busy, command dropped", out);
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
        async_lines_dropped_.load(std::memory_order_relaxed),
    };
}
