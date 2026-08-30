#include "Junkebox.h"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <strings.h>  // strcasecmp

#include "esp_log.h"

#include <Logger.h>
#include <SDCard.h>
#include <TinyShell.h>

namespace {
constexpr const char* TAG = "JUNKEBOX";
constexpr size_t kMaxSongLineBytes = 64;
constexpr uint16_t kMaxSongLoops = 100;

// Semitone offset from C, indexed by note letter - 'A'. A=9 B=11 C=0 D=2
// E=4 F=5 G=7, i.e. standard piano key order.
constexpr int8_t kSemitoneFromC[7] = {9, 11, 0, 2, 4, 5, 7};

// Parses a note token ("C4", "C#4", "Db4", "REST") into its frequency in Hz.
// REST/PAUSE (case-insensitive) means silence and yields freq_hz = 0.
bool note_token_to_freq(const char* token, uint16_t& freq_hz) {
    if (strcasecmp(token, "REST") == 0 || strcasecmp(token, "PAUSE") == 0) {
        freq_hz = 0;
        return true;
    }

    const char letter = static_cast<char>(toupper(static_cast<unsigned char>(token[0])));
    if (letter < 'A' || letter > 'G') return false;

    size_t pos = 1;
    int8_t accidental = 0;
    if (token[pos] == '#') {
        accidental = 1;
        pos++;
    } else if (toupper(static_cast<unsigned char>(token[pos])) == 'B' &&
               isdigit(static_cast<unsigned char>(token[pos + 1]))) {
        // "Bb4"-style flat only makes sense before an octave digit; plain
        // "B4" (the note B) must not be mistaken for a flat marker.
        accidental = -1;
        pos++;
    }

    if (token[pos] == '\0' || !isdigit(static_cast<unsigned char>(token[pos]))) return false;
    const int octave = atoi(&token[pos]);

    // MIDI note number, then equal-temperament frequency referenced to A4 (MIDI 69) = 440Hz.
    const int midi = (octave + 1) * 12 + kSemitoneFromC[letter - 'A'] + accidental;
    const float freq = 440.0f * powf(2.0f, (midi - 69) / 12.0f);
    if (freq < 20.0f || freq > 20000.0f) return false;

    freq_hz = static_cast<uint16_t>(freq + 0.5f);
    return true;
}

// Optional whole-file repetition directive: "LOOP,N". Keeping it separate
// from parse_note_line() means old song files remain exactly compatible and
// built-in sounds (which contain no directive) still play once.
bool parse_loop_line(char* line, uint16_t& loop_count) {
    const size_t len = strlen(line);
    if (len > 0 && line[len - 1] == '\r') line[len - 1] = '\0';

    while (*line == ' ' || *line == '\t') line++;
    if (strncasecmp(line, "LOOP,", 5) != 0) return false;

    char* end = nullptr;
    const unsigned long value = strtoul(line + 5, &end, 10);
    while (end != nullptr && (*end == ' ' || *end == '\t')) end++;

    if (end == nullptr || end == line + 5 || *end != '\0' ||
        value == 0 || value > kMaxSongLoops) {
        return false;
    }

    loop_count = static_cast<uint16_t>(value);
    return true;
}
}  // namespace

Junkebox::Junkebox(uint8_t buzzer_pin, uint8_t channel)
    : buzzer_pin_(buzzer_pin), channel_(channel) {}

esp_err_t Junkebox::begin(SDCard& card) {
    card_ = &card;

    const gpio_num_t gpio = static_cast<gpio_num_t>(buzzer_pin_);
    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (channel_ >= static_cast<uint8_t>(LEDC_CHANNEL_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = gpio_reset_pin(gpio);
    if (err != ESP_OK) return err;

    ledc_timer_config_t timer_cfg = {};
    timer_cfg.speed_mode      = PWM_MODE;
    timer_cfg.timer_num       = PWM_TIMER;
    timer_cfg.duty_resolution = PWM_RESOLUTION;
    timer_cfg.freq_hz         = 440;  // placeholder; set_tone() overwrites this per note
    timer_cfg.clk_cfg         = LEDC_AUTO_CLK;
    err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) return err;

    // HBridge may already have installed the fade service (its channels use
    // ledc_set_duty_and_update() too, which requires it) — that's fine,
    // ESP_ERR_INVALID_STATE just means it's already there. Don't assume
    // that ordering happened, though: install it ourselves too, since
    // Junkebox must not depend on another lib's construction order.
    err = ledc_fade_func_install(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    ledc_channel_config_t ch_cfg = {};
    ch_cfg.speed_mode = PWM_MODE;
    ch_cfg.channel    = static_cast<ledc_channel_t>(channel_);
    ch_cfg.timer_sel  = PWM_TIMER;
    ch_cfg.gpio_num   = buzzer_pin_;
    ch_cfg.duty       = 0;
    ch_cfg.hpoint     = 0;
    err = ledc_channel_config(&ch_cfg);
    if (err != ESP_OK) return err;

    // Max GPIO drive strength (default is GPIO_DRIVE_CAP_2, ~20mA) — lets
    // the pin charge/discharge the piezo element faster, which is the
    // software-only lever for louder output on a passive buzzer driven
    // straight off a GPIO. Set after ledc_channel_config() so it's not the
    // one getting reset by the pin's function/mux setup. Not fatal if the
    // pin doesn't support it — worst case the buzzer is just as quiet as
    // before, not worth failing begin() over.
    gpio_set_drive_capability(gpio, GPIO_DRIVE_CAP_3);

    request_queue_ = xQueueCreate(1, sizeof(PlayRequest));
    if (request_queue_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    hw_configured_ = true;
    return ESP_OK;
}

esp_err_t Junkebox::set_tone(uint16_t freq_hz) {
    if (freq_hz == 0) {
        return ledc_set_duty_and_update(PWM_MODE, static_cast<ledc_channel_t>(channel_), 0, 0);
    }

    esp_err_t err = ledc_set_freq(PWM_MODE, PWM_TIMER, freq_hz);
    if (err != ESP_OK) return err;

    return ledc_set_duty_and_update(PWM_MODE, static_cast<ledc_channel_t>(channel_), PWM_DUTY_50_PCT, 0);
}

bool Junkebox::play(const char* path) {
    if (!ready_.load(std::memory_order_acquire) || path == nullptr) {
        return false;
    }

    PlayRequest req{};
    req.kind = PlayRequest::Kind::SdFile;
    strncpy(req.path, path, sizeof(req.path) - 1);

    // Overwrite rather than send: a song already queued but not yet
    // consumed by the task is meant to be replaced, not played after this one.
    xQueueOverwrite(request_queue_, &req);
    xTaskNotifyGive(task_handle_);  // wake the task immediately, idle or mid-note
    return true;
}

bool Junkebox::play(BuiltinSound sound) {
    if (!ready_.load(std::memory_order_acquire)) {
        return false;
    }

    PlayRequest req{};
    req.kind = PlayRequest::Kind::Builtin;
    req.builtin_notes = JunkeboxBuiltinSongs::text_for(sound);

    xQueueOverwrite(request_queue_, &req);
    xTaskNotifyGive(task_handle_);
    return true;
}

bool Junkebox::play_tone(uint16_t freq_hz, uint32_t duration_ms) {
    if (!ready_.load(std::memory_order_acquire)) {
        return false;
    }

    PlayRequest req{};
    req.kind = PlayRequest::Kind::RawTone;
    req.raw_freq_hz = freq_hz;
    req.raw_duration_ms = duration_ms;

    xQueueOverwrite(request_queue_, &req);
    xTaskNotifyGive(task_handle_);
    return true;
}

void Junkebox::stop() {
    if (!ready_.load(std::memory_order_acquire)) return;

    // Empty the mailbox so a stop() that races a play() doesn't leave a
    // stale request for the task to pick back up right after silencing.
    PlayRequest discard;
    xQueueReceive(request_queue_, &discard, 0);
    xTaskNotifyGive(task_handle_);
}

void Junkebox::task(void* param) {
    static_cast<Junkebox*>(param)->run_loop();
}

void Junkebox::run_loop() {
    task_handle_ = xTaskGetCurrentTaskHandle();

    if (!hw_configured_) {
        // begin() failed (bad pin, LEDC channel/timer already claimed,
        // ...) — ROBOT::init() already logged why. Never publish ready_:
        // staying idle here is what makes play()/stop() correctly report
        // "not initialized" instead of silently accepting a request that
        // set_tone() could only fail on a channel that was never configured.
        ESP_LOGE(TAG, "begin() never configured the buzzer hardware; Junkebox staying idle");
        vTaskSuspend(nullptr);
        return;
    }

    ready_.store(true, std::memory_order_release);

    while (true) {
        PlayRequest req;
        if (xQueueReceive(request_queue_, &req, portMAX_DELAY) != pdTRUE) continue;

        // Discard any notification built up while idle (e.g. a stop() that
        // arrived with nothing playing) so it doesn't cut the first note short.
        ulTaskNotifyTake(pdTRUE, 0);

        playing_.store(true, std::memory_order_release);
        switch (req.kind) {
            case PlayRequest::Kind::Builtin:
                play_builtin_notes(req.builtin_notes);
                break;
            case PlayRequest::Kind::RawTone:
                play_raw_tone(req.raw_freq_hz, req.raw_duration_ms);
                break;
            case PlayRequest::Kind::SdFile:
            default:
                play_file(req.path);
                break;
        }
        set_tone(0);
        playing_.store(false, std::memory_order_release);
    }
}

bool Junkebox::parse_note_line(char* line, uint16_t& freq_hz, uint32_t& duration_ms) const {
    // Trim a trailing '\r' (files edited/generated on Windows).
    const size_t len = strlen(line);
    if (len > 0 && line[len - 1] == '\r') line[len - 1] = '\0';

    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0' || *line == '#') return false;  // blank line / comment

    char* comma = strchr(line, ',');
    if (comma == nullptr) return false;
    *comma = '\0';

    if (!note_token_to_freq(line, freq_hz)) return false;

    duration_ms = static_cast<uint32_t>(atoi(comma + 1));
    return duration_ms > 0;
}

void Junkebox::play_file(const char* path) {
    if (card_ == nullptr || !card_->is_mounted()) {
        ESP_LOGW(TAG,
                 "cannot play '%s': SD card is not mounted for the robot",
                 path != nullptr ? path : "(null)");
        return;
    }

    size_t bytes_read = 0;
    if (!card_->read_file(path, file_buffer_, sizeof(file_buffer_) - 1, &bytes_read)) {
        ESP_LOGW(TAG, "failed to read song file: %s", path);
        return;
    }
    play_buffer(bytes_read);
}

void Junkebox::play_builtin_notes(const char* notes) {
    size_t len = strlen(notes);
    if (len > sizeof(file_buffer_) - 1) len = sizeof(file_buffer_) - 1;  // never happens for the built-ins above; just a guard
    memcpy(file_buffer_, notes, len);
    play_buffer(len);
}

// Single note, no parsing: set_tone() straight from the raw (freq_hz,
// duration_ms) play_tone() queued — same interruptible wait as a note
// inside play_buffer(), just without a file_buffer_ line to read it from.
void Junkebox::play_raw_tone(uint16_t freq_hz, uint32_t duration_ms) {
    const esp_err_t err = set_tone(freq_hz);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set_tone(%u) failed: 0x%x", freq_hz, err);
        return;
    }

    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(duration_ms));
}

// Parses and plays the note lines already sitting in file_buffer_[0..length)
// — shared by play_file() (SD-read bytes) and play_builtin_notes(). Each line
// is copied before parse_note_line() mutates it; preserving file_buffer_ lets
// LOOP,N replay the same text without reading the SD card again.
void Junkebox::play_buffer(size_t length) {
    file_buffer_[length] = '\0';

    uint16_t loop_count = 1;
    for (uint16_t loop = 0; loop < loop_count; ++loop) {
        const char* cursor = file_buffer_;
        while (cursor != nullptr && *cursor != '\0') {
            const char* line_start = cursor;
            const char* newline = strchr(cursor, '\n');
            const size_t line_length = newline != nullptr
                ? static_cast<size_t>(newline - line_start)
                : strlen(line_start);
            cursor = newline != nullptr ? newline + 1 : nullptr;

            // Note/directive lines are tiny. Oversized lines are malformed
            // (or long comments) and can be ignored without touching the
            // preserved source buffer used by the next loop.
            if (line_length >= kMaxSongLineBytes) continue;

            char line[kMaxSongLineBytes];
            memcpy(line, line_start, line_length);
            line[line_length] = '\0';

            uint16_t requested_loops = 1;
            if (parse_loop_line(line, requested_loops)) {
                if (loop == 0) loop_count = requested_loops;
                continue;
            }

            uint16_t freq_hz = 0;
            uint32_t duration_ms = 0;
            if (!parse_note_line(line, freq_hz, duration_ms)) continue;

            const esp_err_t tone_err = set_tone(freq_hz);
            if (tone_err != ESP_OK) {
                ESP_LOGW(TAG, "set_tone(%u) failed: 0x%x", freq_hz, tone_err);
            }

            // Waits for the note's duration, but returns immediately (>0) if
            // play()/stop() signals the task in the meantime — that's how
            // playback is interrupted mid-note, including inside a loop.
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(duration_ms)) > 0) {
                return;
            }
        }
    }
}

void Junkebox::register_shell_commands(TinyShell& shell, Logger& logger) {
    shell.create_module("junkebox", "Buzzer playback: SD-stored songs and compiled-in system sounds");

    shell.add([this, &logger](std::string path) -> uint8_t {
        if (path.empty() || path.size() >= MAX_PATH_LENGTH) {
            logger.insert_logf(logType::ERRO,
                               "Junkebox: invalid/long song path (%u bytes, max=%u)",
                               static_cast<unsigned>(path.size()),
                               static_cast<unsigned>(MAX_PATH_LENGTH - 1U));
            return RESULT_ERROR;
        }
        if (card_ == nullptr || !card_->is_mounted()) {
            logger.insert_log(
                logType::ERRO,
                "Junkebox: SD card is not mounted for robot; safely eject it from the PC and press button 2");
            return RESULT_ERROR;
        }
        if (!card_->file_exists(path.c_str())) {
            logger.insert_logf(logType::ERRO,
                               "Junkebox: song file not found: %s",
                               path.c_str());
            return RESULT_ERROR;
        }
        if (!play(path.c_str())) {
            logger.insert_log(logType::ERRO, "Junkebox is not initialized yet");
            return RESULT_ERROR;
        }
        logger.insert_logf(logType::INFO, "Junkebox: playing %s", path.c_str());
        return RESULT_OK;
    }, "play", "Play a song file from the SD card (path relative to the mount point)", "junkebox");

    shell.add([this, &logger](std::string name) -> uint8_t {
        BuiltinSound sound;
        if (!JunkeboxBuiltinSongs::from_name(name, sound)) {
            logger.insert_logf(logType::ERRO,
                               "Junkebox: unknown builtin sound '%s' (click/success/error/warning/boot/elevator)",
                               name.c_str());
            return RESULT_ERROR;
        }
        if (!play(sound)) {
            logger.insert_log(logType::ERRO, "Junkebox is not initialized yet");
            return RESULT_ERROR;
        }
        logger.insert_logf(logType::INFO, "Junkebox: playing builtin '%s'", name.c_str());
        return RESULT_OK;
    }, "play_builtin", "Play a compiled-in sound: click/success/error/warning/boot/elevator", "junkebox");

    shell.add([this, &logger]() -> uint8_t {
        stop();
        logger.insert_log(logType::INFO, "Junkebox: stopped");
        return RESULT_OK;
    }, "stop", "Silence the buzzer and stop the current song", "junkebox");

    shell.add([this, &logger]() -> uint8_t {
        logger.insert_logf(logType::INFO, "Junkebox: %s", is_playing() ? "playing" : "idle");
        return RESULT_OK;
    }, "status", "Show whether a song is currently playing", "junkebox");
}
