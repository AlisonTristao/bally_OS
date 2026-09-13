#ifndef OTA_DEFAULTS_H
#define OTA_DEFAULTS_H

#include <cstdint>

// Canonical numeric defaults for OTAUpdater's runtime-tunable settings (see
// OtaTuning in OTAUpdater.h). Split out from OTAUpdater.h itself — which
// pulls esp_event.h/esp_http_server.h/LogTypes.h — so that
// RobotSettings.h can read these same values for SettingsData's field
// defaults without dragging OTAUpdater's full public interface into a
// generic settings-storage module. Both headers include this one instead of
// either repeating the literals or one including the other.
namespace OtaDefaults {
    inline constexpr uint32_t led_step_ms        = 150;   // carousel step while OTA is active
    inline constexpr uint32_t led_hold_ms        = 200;   // refresh window for solid status LEDs
    inline constexpr uint32_t led_fail_hold_ms   = 800;   // red "connect failed" hold before retrying
    inline constexpr uint32_t connect_timeout_ms = 10000; // per-network connect timeout
    inline constexpr uint32_t retry_scan_ms      = 5000;  // delay before re-scanning when nothing matched
    // Channel restored after leaving OTA. Must match the dongle's own
    // kEspNowChannel (bally_dongle/lib/AppRuntime/AppRuntime.cpp) exactly, or
    // the two sides simply never hear each other -- this only seeds a FRESH
    // settings.conf, so an already-deployed robot needs
    // "settings -set ota,espnow_channel,11" + "settings -save" + reboot to
    // actually move (esp_wifi_set_channel() for it runs once at boot).
    inline constexpr uint8_t  espnow_channel     = 11;
}

#endif // OTA_DEFAULTS_H
