#ifndef TERMINAL_RESPONDER_H
#define TERMINAL_RESPONDER_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include <BtpTransport.h>
#include <ShellLineEditor.h>
#include <btp/codec.hpp>

// Serves the BTP terminal channel (MessageType::Terminal, object_id
// TERMINAL_IN 0x0001 / TERMINAL_OUT 0x0002 -- BTP/docs/session-and-terminal.md
// section 7) end to end with TraceView, so the robot's TinyShell is reachable
// from TraceView's terminal widget exactly the way bally_dongle's own shell
// is: line editing (echo, backspace, arrows/history, Tab, Ctrl+R) runs
// server-side here, in a ShellLineEditor per remote origin, and the command's
// output is mirrored back inline as TERMINAL_OUT.
//
// Same shape as ManifestCatalog: pure C++ (no Arduino/FreeRTOS), the caller
// owns the clock and the FreeRTOS glue. It is driven from three contexts:
//
//   on_terminal_in()          Wi-Fi RX task  -- only buffers bytes, never
//                                               runs the editor
//   pump()                    comms task     -- runs the editors, emits
//                                               TERMINAL_OUT, submits lines
//   deliver_command_output()  shell task     -- hands back one command's
//                                               captured output
//
// A small std::atomic_flag spinlock (same idea as CommandProcessor's
// cache_lock_) guards the slot table and the per-slot hand-off fields; the
// ShellLineEditor objects themselves are only ever touched by pump().
//
// TERMINAL has no destination field on the wire, so per-origin state (input
// buffer, cursor, history) is isolated by a small LRU pool keyed on
// (source_id, boot_id). Two terminals to the same robot therefore do not
// corrupt each other's line editing; the echo stream itself is still shared
// (the same limitation bally_dongle's single g_terminalShell has).
class TerminalResponder {
public:
    static constexpr std::uint16_t kTerminalInObjectId = 0x0001U;
    static constexpr std::uint16_t kTerminalOutObjectId = 0x0002U;

    static constexpr std::size_t kMaxOrigins = 3U;
    // Per-origin history depth. Deliberately small: kMaxOrigins editors are
    // resident, so this trades arrow-up depth for RAM.
    static constexpr std::size_t kHistoryCapacity = 16U;
    // Ceiling on un-drained TERMINAL_IN bytes held for one origin (a large
    // paste is truncated into the editor, same as bally_dongle does).
    static constexpr std::size_t kMaxPendingIn = 1024U;
    // Ceiling on one command's mirrored output before it is truncated with a
    // notice -- keeps a `help`-sized dump from flooding the TX scheduler.
    static constexpr std::size_t kMaxCommandOut = 2048U;
    // Ceiling on un-drained unsolicited output held for one origin
    // (push_async_output). pump() drains it every comms pass, so this only
    // fills if the radio backs up hard; the oldest lines are dropped then.
    static constexpr std::size_t kMaxAsyncOut = 1024U;

    // Submit one completed shell line for execution. The implementation
    // enqueues it on the robot's command queue tagged with (source_id,
    // boot_id) so deliver_command_output() can be routed back here. Returns
    // false when the queue is full (the line is dropped, with a notice).
    using SubmitFn = bool (*)(void* context, std::uint32_t source_id, std::uint32_t boot_id,
                              const char* command_line);

    // `seal` is the channel-B sealer (RadioSeal::seal_e) -- TERMINAL rides
    // channel B (TraceView<->robot, key E), same as TELEMETRY/LOG. nullptr
    // means "no encryption", the mode the native tests exercise; real
    // firmware always passes a real sealer and every TERMINAL_OUT then goes
    // out sealed or not at all.
    //
    // `completion` is forwarded to every per-origin ShellLineEditor for Tab
    // (the caller passes a TinyShell::complete_line-backed lambda, same as
    // bally_dongle). `prompt` is copied.
    void configure(BtpEndpoint& endpoint, BtpSealFn seal, void* seal_context,
                   ShellLineEditor::CompletionProvider completion, SubmitFn submit,
                   void* submit_context, const char* prompt) noexcept;

    // One reassembled, already-opened TERMINAL_IN payload from `header`'s
    // origin. Runs on the Wi-Fi RX task: it only allocates/looks up the
    // origin's slot and appends the bytes, never runs the editor.
    void on_terminal_in(const btp::Header& header, btp::ByteView plaintext) noexcept;

    // Advances every active origin: drains its buffered input through the
    // editor, emits whatever it echoed as TERMINAL_OUT frame(s), submits any
    // completed command line, and mirrors back a delivered command's output.
    // Call once per comms-task pass with a monotonic microsecond clock.
    void pump(std::uint64_t now_us) noexcept;

    // Hands one terminal-originated command's captured output (and TinyShell
    // status) back to the origin's editor. Called from the shell task after
    // run_command_line() returns. A silent no-op if the origin's slot was
    // evicted while the command ran.
    void deliver_command_output(std::uint32_t source_id, std::uint32_t boot_id,
                                const char* text, std::uint8_t status) noexcept;

    // Queue one line of unsolicited output for an existing terminal origin --
    // e.g. an asynchronous "debug" test's periodic result, which otherwise
    // only leaves the robot as a LOG frame and never shows in the terminal
    // that started it. `text` is one line WITHOUT a trailing newline; it is
    // mirrored back prefixed with "! " and CR+LF-terminated, matching a
    // command's own output, and printed above the prompt (the edit line is
    // restored once the burst pauses).
    //
    // A silent no-op if (source_id, boot_id) has no live slot (that terminal
    // never spoke to this robot, or its slot was evicted). Safe to call from
    // any task; bounded, drops the line on overflow or lock contention.
    void push_async_output(std::uint32_t source_id, std::uint32_t boot_id,
                           const char* text) noexcept;

    struct Stats {
        std::uint32_t origins_seen;
        std::uint32_t origins_evicted;
        std::uint32_t lines_submitted;
        std::uint32_t lines_dropped_busy;
        std::uint32_t frames_out;
        std::uint32_t in_bytes_dropped;
        std::uint32_t async_lines_dropped;
    };
    Stats stats() const noexcept;

private:
    struct Slot {
        bool used = false;
        std::uint32_t source_id = 0U;
        std::uint32_t boot_id = 0U;
        std::uint64_t last_used_us = 0U;

        // editor, prompt_painted, awaiting_result and pending_line are touched
        // ONLY by pump() -- never by on_terminal_in() or deliver_command_output()
        // -- so they need no lock. A slot reused for a new origin is reset by
        // pump() when it sees needs_editor_reset, not by find_or_alloc() on the
        // Wi-Fi task: reset()ing a ShellLineEditor there, under lock_, is what a
        // spin-waiting pump()/deliver_command_output() could deadlock against.
        ShellLineEditor editor{kHistoryCapacity};
        bool prompt_painted = false;
        bool awaiting_result = false;  // a line is executing; hold further input
        std::string pending_line;     // one line finished while awaiting_result

        // Hand-off fields, guarded by lock_. Every critical section that holds
        // lock_ is now just a handful of field reads/writes plus one bounded
        // string swap -- never an allocation loop, an editor call or a
        // reset() -- so the Wi-Fi RX task never spin-waits here for long.
        std::string in;            // TERMINAL_IN bytes awaiting the editor
        std::string pending_out;   // command output awaiting the editor
        std::string async_out;     // unsolicited output awaiting the editor
                                   // (push_async_output); bounded at kMaxAsyncOut
        bool result_ready = false;
        bool needs_editor_reset = false;  // slot just (re)allocated; pump() resets

        // pump()-owned (like prompt_painted/awaiting_result): an async burst
        // broke off the prompt line and it has not been repainted yet. pump()
        // sets it when it flushes async_out and clears it on the pass that
        // repaints the prompt once the burst pauses.
        bool async_prompt_dirty = false;
    };

    // Bound for try_lock()'s spin. Large only because every critical section
    // is O(1) field work -- if this is ever exhausted the caller's fallback
    // (skip a pass / drop bytes) is still far better than a core-0 hang.
    static constexpr unsigned kMaxLockSpins = 100000U;

    void lock() noexcept;      // unbounded spin -- shell task only (see .cpp)
    bool try_lock() noexcept;  // bounded -- comms and Wi-Fi tasks
    void unlock() noexcept;

    // lock_ held by caller.
    Slot* find(std::uint32_t source_id, std::uint32_t boot_id) noexcept;
    Slot* find_or_alloc(std::uint32_t source_id, std::uint32_t boot_id,
                        std::uint64_t now_us) noexcept;

    // Chops `bytes` into <= one-sealed-ESP-NOW-frame pieces and sends each as
    // its own TERMINAL_OUT logical message (mirrors Logger::send_log_direct).
    void emit_terminal_out(const std::string& bytes, std::uint64_t now_us) noexcept;

    // pump()-only: submit one completed line for execution, or write a
    // "busy" notice into `out` if the command queue refused it. `src`/`boot`
    // are the origin ids pump() snapshotted under lock_.
    void submit_line(Slot& s, std::uint32_t src, std::uint32_t boot,
                     const std::string& line, std::string& out) noexcept;

    BtpEndpoint* endpoint_ = nullptr;
    BtpSealFn seal_ = nullptr;
    void* seal_context_ = nullptr;
    ShellLineEditor::CompletionProvider completion_;
    SubmitFn submit_ = nullptr;
    void* submit_context_ = nullptr;
    std::string prompt_;

    Slot slots_[kMaxOrigins];
    std::atomic_flag lock_ = ATOMIC_FLAG_INIT;

    std::atomic<std::uint32_t> origins_seen_{0U};
    std::atomic<std::uint32_t> origins_evicted_{0U};
    std::atomic<std::uint32_t> lines_submitted_{0U};
    std::atomic<std::uint32_t> lines_dropped_busy_{0U};
    std::atomic<std::uint32_t> frames_out_{0U};
    std::atomic<std::uint32_t> in_bytes_dropped_{0U};
    std::atomic<std::uint32_t> async_lines_dropped_{0U};
};

#endif  // TERMINAL_RESPONDER_H
