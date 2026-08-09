#ifndef JUNKEBOX_BUILTIN_SONGS_H
#define JUNKEBOX_BUILTIN_SONGS_H

// autor: Alison Tristao
// email: AlisonTristao@hotmail.com

#include <cstdint>
#include <string>

// Fixed, compiled-in short sounds for system/UI feedback — unlike the songs
// Junkebox::play(const char*) reads from the SD card, these never touch the
// card, so they still play even when it's unmounted/missing/exposed to a
// USB host. Written in the exact same "NOTE,DURATION_MS" text format as an
// SD song file (see Junkebox.h); Junkebox::play(BuiltinSound) just copies
// the string below into its parse buffer instead of reading a file into it.
enum class BuiltinSound : uint8_t {
    Click,    // single short blip — generic button/UI feedback
    Ping,     // single very short, higher-pitched tick — side sensor edge (can fire often, kept minimal)
    Success,  // e.g. calibration finished, OTA update applied, telemetry saved
    Error,    // e.g. ERROR state entered, calibration/telemetry/init failure
    Warning,  // e.g. IMU not detected, buzzer init failed, settings.conf line skipped
    Boot,     // ROBOT::init() finished successfully
    Elevator, // longer novelty tune (lounge/elevator-music riff) — manual "junkebox play_builtin elevator" only, no automatic trigger
};

namespace JunkeboxBuiltinSongs {

constexpr const char* kClick   = "C6,30\n";
constexpr const char* kPing    = "A6,20\n";
constexpr const char* kSuccess = "C5,80\nE5,80\nG5,120\n";
constexpr const char* kError   = "A4,120\nREST,40\nA4,120\nREST,40\nA4,220\n";
constexpr const char* kWarning = "E5,100\nREST,60\nE5,100\n";
constexpr const char* kBoot    = "C5,100\nE5,100\nG5,100\nC6,220\n";
constexpr const char* kElevator =
    "C4,300\nE4,300\nG4,300\nB4,300\nA4,300\nG4,300\nE4,300\nD4,300\n"
    "REST,150\n"
    "D4,300\nF4,300\nA4,300\nC5,300\nB4,300\nA4,300\nF4,300\nE4,300\n"
    "REST,300\n";

/// Note text for a BuiltinSound, in Junkebox's "NOTE,DURATION_MS" format.
inline const char* text_for(BuiltinSound sound) {
    switch (sound) {
        case BuiltinSound::Click:   return kClick;
        case BuiltinSound::Ping:    return kPing;
        case BuiltinSound::Success: return kSuccess;
        case BuiltinSound::Error:   return kError;
        case BuiltinSound::Warning: return kWarning;
        case BuiltinSound::Boot:    return kBoot;
        case BuiltinSound::Elevator: return kElevator;
    }
    return kClick;
}

/// Maps a lowercase name ("click", "ping", "success", "error", "warning",
/// "boot", "elevator" — used by the "junkebox play_builtin" shell command)
/// to a BuiltinSound.
/// @return false when name matches none of them.
inline bool from_name(const std::string& name, BuiltinSound& out) {
    if (name == "click")    { out = BuiltinSound::Click;    return true; }
    if (name == "ping")     { out = BuiltinSound::Ping;     return true; }
    if (name == "success")  { out = BuiltinSound::Success;  return true; }
    if (name == "error")    { out = BuiltinSound::Error;    return true; }
    if (name == "warning")  { out = BuiltinSound::Warning;  return true; }
    if (name == "boot")     { out = BuiltinSound::Boot;     return true; }
    if (name == "elevator") { out = BuiltinSound::Elevator; return true; }
    return false;
}

}  // namespace JunkeboxBuiltinSongs

#endif // JUNKEBOX_BUILTIN_SONGS_H
