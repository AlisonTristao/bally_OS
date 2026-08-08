#ifndef JUNKEBOX_H
#define JUNKEBOX_H

// autor: Alison Tristao
// email: AlisonTristao@hotmail.com

#include <atomic>
#include <cstdint>
#include <string>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "BuiltinSongs.h"

class SDCard;
class TinyShell;
class Logger;

/**
 * @brief Plays a song read from the SD card on a buzzer, off the calling task.
 *
 * A song is a plain text file, one note per line: "NOTE,DURATION_MS"
 * (e.g. "C4,250", "REST,100" for silence). play() only queues the request
 * and returns immediately; the actual note-by-note playback (and the LEDC
 * frequency changes it drives) happens on task()'s own FreeRTOS task, so
 * callers (the state machine, the shell) are never blocked while a song
 * plays. A second play() interrupts whatever is currently playing and
 * switches to the new song; stop() just silences the buzzer and goes idle.
 */
class Junkebox {
public:
    /**
     * @brief Create a buzzer manager.
     *
     * @param buzzer_pin GPIO driving the buzzer.
     * @param channel LEDC channel reserved for this buzzer (its own LEDC
     * timer is configured internally — see the .cpp for why it cannot share
     * HBridge's PWM_TIMER).
     */
    Junkebox(uint8_t buzzer_pin, uint8_t channel);
    ~Junkebox() = default;

    Junkebox(const Junkebox&) = delete;
    Junkebox& operator=(const Junkebox&) = delete;

    /**
     * @brief Configure the LEDC timer/channel for the buzzer and record the
     * SD card song files are read from.
     *
     * @param card Must already be mounted; only referenced (never owned).
     * @return ESP_OK once the buzzer is ready to play.
     */
    esp_err_t begin(SDCard& card);

    /**
     * @brief Queue a song for playback and return immediately.
     *
     * Interrupts whatever is currently playing (mid-note, if necessary) and
     * switches to this song. The task consuming the request does the actual
     * SD read, so a missing/unreadable file is only discovered there — this
     * only fails when Junkebox itself was never begin()'d.
     *
     * @param path Song file path, relative to the SD card mount point.
     * @return false when Junkebox has not finished begin()/task() startup.
     */
    bool play(const char* path);

    /**
     * @brief Queue a fixed, compiled-in sound (see BuiltinSongs.h) for
     * playback and return immediately. Never touches the SD card, so it
     * still works when the card is unmounted, missing, or exposed to a USB
     * host — meant for system/UI feedback (error, success, a button click,
     * ...), not for the longer SD-backed songs play(const char*) reads.
     * Same interrupt-and-switch semantics as play(const char*).
     * @return false when Junkebox has not finished begin()/task() startup.
     */
    bool play(BuiltinSound sound);

    /**
     * @brief Queue a single raw tone for playback and return immediately —
     * bypasses note-name parsing entirely, straight to set_tone(). Meant
     * for hardware bring-up/testing (the "robot test_buzzer" shell command),
     * not regular playback. Same interrupt-and-switch semantics as the
     * other play() overloads.
     * @param freq_hz 0 plays silence (still useful to test "buzzer off").
     * @return false when Junkebox has not finished begin()/task() startup.
     */
    bool play_tone(uint16_t freq_hz, uint32_t duration_ms);

    /// Silence the buzzer immediately, mid-note if necessary, and go idle.
    void stop();

    bool is_playing() const { return playing_.load(std::memory_order_acquire); }

    /**
     * @brief Background task entry point. Pinned to PRO_CPU_NUM from
     * src/main.cpp (see CONTRIBUTING.md's FreeRTOS task rules — core 1 is
     * reserved for the state machine alone).
     * @param param Must be the Junkebox instance (`static_cast<Junkebox*>`).
     */
    static void task(void* param);

    /**
     * @brief Register the "junkebox" shell module (play/play_builtin/stop/status).
     */
    void register_shell_commands(TinyShell& shell, Logger& logger);

private:
    // Max song file size read into file_buffer_ in one shot (see .cpp) — a
    // few hundred notes at "NOTE,DURATION\n" width; generous for a jingle
    // without needing a heap allocation on the Junkebox task's own stack.
    static constexpr size_t MAX_SONG_FILE_BYTES = 4096;
    static constexpr size_t MAX_PATH_LENGTH = 128;

    static constexpr ledc_mode_t PWM_MODE = LEDC_LOW_SPEED_MODE;
    // Own timer, never HBridge's PWM_TIMER (LEDC_TIMER_0): this timer's
    // frequency changes on every note, which would detune both motors if
    // they shared it.
    static constexpr ledc_timer_t PWM_TIMER = LEDC_TIMER_1;
    static constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_8_BIT;
    static constexpr uint32_t PWM_DUTY_50_PCT = 128; // ~50% of an 8-bit (255) duty range

    struct PlayRequest {
        enum class Kind : uint8_t { SdFile, Builtin, RawTone };
        Kind kind = Kind::SdFile;

        char path[MAX_PATH_LENGTH];  // Kind::SdFile
        // Non-null => a compiled-in sound: play this note text (BuiltinSongs.h
        // static rodata, so it's safe to keep just the pointer) instead of
        // reading `path` from the SD card. Set by play(BuiltinSound).
        const char* builtin_notes = nullptr;  // Kind::Builtin
        uint16_t raw_freq_hz = 0;             // Kind::RawTone
        uint32_t raw_duration_ms = 0;         // Kind::RawTone
    };

    void run_loop();
    void play_file(const char* path);
    void play_builtin_notes(const char* notes);
    void play_raw_tone(uint16_t freq_hz, uint32_t duration_ms);
    void play_buffer(size_t length);
    bool parse_note_line(char* line, uint16_t& freq_hz, uint32_t& duration_ms) const;
    esp_err_t set_tone(uint16_t freq_hz);

    const uint8_t buzzer_pin_;
    const uint8_t channel_;
    SDCard* card_ = nullptr;

    // Set by begin() only once every hardware step (LEDC timer/channel,
    // request queue) actually succeeds. task()/run_loop() is still created
    // unconditionally in main.cpp regardless of begin()'s result — this is
    // what stops it from publishing ready_ (and silently accepting play()
    // calls it could never act on: set_tone() on a channel begin() never
    // configured) when the hardware setup failed.
    bool hw_configured_ = false;

    // Set true by run_loop() only after confirming hw_configured_; read by
    // play()/stop() (any task) to reject calls made before startup or after
    // a failed begin().
    std::atomic<bool> ready_{false};
    // Written only by run_loop() at task entry (before ready_ is set, so no
    // reader races it); read by play()/stop() to target the notification.
    TaskHandle_t task_handle_ = nullptr;
    // Set by run_loop() before/after each song; read by is_playing() from
    // any task.
    std::atomic<bool> playing_{false};

    // Depth-1 mailbox for the pending play() request — same
    // trigger-and-forget handoff pattern as ROBOT::receivedDataQueue
    // (shell). xQueueOverwrite() lets a new play() replace a request the
    // task has not consumed yet instead of queuing both.
    QueueHandle_t request_queue_ = nullptr;

    char file_buffer_[MAX_SONG_FILE_BYTES];
};

#endif // JUNKEBOX_H
