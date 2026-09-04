#ifndef ROBOT_SETTINGS_H
#define ROBOT_SETTINGS_H

// autor: Alison Tristao

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

// Project-wide compile-time constants: OTA_MDNS_NAME_MAX_LEN and
// OTA_PASSWORD_MAX_LEN size the ota_* buffers below. Included explicitly
// (this header used to get it only transitively, via OTAUpdater.h).
#include <Settings.h>

// For OTAUpdater's canonical default values, which back the ota_* field
// defaults below — kept in one dependency-free header (shared with
// OTAUpdater.h's OtaTuning) instead of repeating the literals here, without
// pulling OTAUpdater's full interface (esp_event.h/esp_http_server.h/...)
// into this generic settings-storage module.
#include <OtaDefaults.h>

class SDCard;
class TinyShell;
class Logger;

// SD card root file where the runtime settings are persisted. Same
// convention as OTA_WIFI_LIST_FILE (Settings.h): plain text, relative to the
// SDCard mount point.
#define ROBOT_SETTINGS_FILE "settings.conf"

// Read buffer ceiling for the settings file. Heap-allocated (not a stack
// array): RobotSettings::load runs on the app_main task before any
// FreeRTOS task exists, and that task's stack is small.
#define ROBOT_SETTINGS_FILE_MAX_BYTES 8192

/**
 * @brief Plain-data snapshot of every runtime-configurable robot setting.
 *
 * Field defaults below are the factory values (what a fresh SD card, or
 * "settings reset_all", restores). Grouped in the same sections
 * include/Settings.h used before this field moved out of it.
 */
struct SettingsData {
    // -------- identity (how this robot presents itself over BTP) --------
    // Surfaced verbatim in MANIFEST_DATA's source_info block
    // (BTP/docs/commands.md section 3.12), read live on every response, so a
    // "settings -set identity name ..." shows up without a reboot. Empty by
    // default: an unconfigured robot simply omits the entry.
    char name[32]        = "";
    char description[64] = "";

    // -------- timers --------
    uint32_t sample_micros  = 1000;   // EKF sample period, us (1000 -> 1kHz)
    uint32_t sysmon_freq_ms = 10000;  // system monitor report period, ms
    uint32_t delay_flags    = 250;    // flags reset / state-change check period, ms
    char     timezone[16]   = "BRT3"; // POSIX TZ string, local log filenames

    // -------- sensor (array sensor / line follower) --------
    uint8_t  len_sensor   = 8;    // active sensor count, 1..ArraySensor::MAX_LEN
    uint8_t  samples      = 30;   // calibration sample count
    uint8_t  delay_sample = 100;  // delay between calibration samples, ms

    // -------- ekf_noise (Kalman filter process/measurement noise) --------
    float v_noise      = 0.1f;
    float w_noise      = 0.1f;
    float b_noise      = 0.0001f;
    float initial_p    = 0.1f;
    float enc_noise    = 0.5f;
    float accel_noise  = 0.5f;
    float gyro_noise   = 0.001f;

    // -------- kinematics --------
    float encoder_ppr  = 70.0f;    // pulses per wheel revolution
    float wheel_radius = 0.0325f;  // wheel radius, meters

    // -------- logger (retained PSRAM ring tuning; capacity itself is fixed
    // at compile time, see LOGGER_PSRAM_CAPACITY_BYTES in Settings.h) --------
    uint32_t logger_mutex_timeout_ms = 100;
    uint32_t max_chunks_per_flush    = 10;
    uint32_t block_size              = 16;

    // -------- pins_led (strapping pins, see include/Settings.h warning) --
    int32_t led0 = 45, led1 = 46, led2 = 43, led3 = 44;

    // -------- pins_hbridge (DRV8251A) --------
    int32_t ain1 = 21, ain2 = 47, bin1 = 3, bin2 = 9;
    int32_t current_a = 14, current_b = 8;

    // -------- pins_encoder --------
    int32_t enc_a0 = 41, enc_a1 = 42, enc_b0 = 39, enc_b1 = 40;

    // -------- pins_button (btn0 is GPIO0 / BOOT strap) --------
    int32_t btn0 = 0, btn1 = 4, btn2 = 5;

    // -------- pins_side_sensor --------
    int32_t left = 1, right = 48;

    // -------- pins_buzzer --------
    int32_t bzr = 38;

    // -------- pins_mux (analog multiplexer feeding the array sensor) --------
    int32_t s0 = 12, s1 = 11, s2 = 10, sig = 13;

    // -------- pins_i2c (IMU, ICM42688) --------
    int32_t sda_pin = 18, scl_pin = 17;

    // -------- ota (Wi-Fi OTA sub-mode timing/identity; ESP-NOW home channel) --
    // Applied via OTAUpdater::configure()/OtaTuning (lib/OTAUpdater) — see
    // ROBOT::init(). Numeric defaults are pulled from OtaDefaults (the
    // canonical copy, shared with OtaTuning's own defaults) instead of
    // repeating the literals here. espnow_channel is also read directly by
    // ROBOT::configureCommunication() to bring ESP-NOW up on the same
    // channel OTA restores on cancel().
    uint32_t ota_led_step_ms        = OtaDefaults::led_step_ms;
    uint32_t ota_led_hold_ms        = OtaDefaults::led_hold_ms;
    uint32_t ota_led_fail_hold_ms   = OtaDefaults::led_fail_hold_ms;
    uint32_t ota_connect_timeout_ms = OtaDefaults::connect_timeout_ms;
    uint32_t ota_retry_scan_ms      = OtaDefaults::retry_scan_ms;
    uint8_t  espnow_channel         = OtaDefaults::espnow_channel;
    // OtaTuning::hostname/instance_name/password default to nullptr (see
    // its own comment) precisely so these three stay the one real literal
    // home. Empty ota_password disables the check; also update the
    // X-OTA-Password header baked into platformio.ini's upload_command if
    // this changes, since that tool can't read settings.conf.
    char ota_hostname[OTA_MDNS_NAME_MAX_LEN]      = "ballyrobot";
    char ota_instance_name[OTA_MDNS_NAME_MAX_LEN] = "BallyRobot OTA";
    char ota_password[OTA_PASSWORD_MAX_LEN]       = "657585";

    // -------- error (ERROR state LED indication) --------
    // Half-period of the synchronized all-LEDs blink shown in ERROR (see
    // ROBOT::blinkErrorLeds()); 250ms -> 2Hz full on/off cycle.
    uint32_t error_blink_ms = 250;
};

enum class SettingType : uint8_t { U32, U8, FLOAT, PIN, STRING };

struct SettingEntry {
    const char* module;
    const char* key;
    SettingType type;
    size_t offset;  // offsetof(SettingsData, field)
    size_t size;    // sizeof(field) — buffer capacity for STRING
};

/**
 * @brief Runtime-configurable robot settings, backed by ROBOT_SETTINGS_FILE
 * on the SD card.
 *
 * Single table (kTable, RobotSettings.cpp) drives applying compiled-in
 * defaults, serializing to the .conf file, parsing it back and the
 * "settings" shell module (get/set/list/save/load/reset) — add one field to
 * SettingsData plus one row in kTable to expose a new setting everywhere.
 */
class RobotSettings {
public:
    RobotSettings() = default;

    /**
     * @brief Load ROBOT_SETTINGS_FILE from the SD card into memory.
     *
     * Never fails the boot: an unmounted card or missing file simply keeps
     * the compiled-in defaults already in data_ (a missing file additionally
     * writes a fresh one, so the SD ends up holding a complete, valid file).
     * Unknown keys/modules and unparsable values are skipped, leaving that
     * field at its previous (default) value.
     *
     * @param skipped_lines Optional count of ignored/invalid lines.
     * @return false only when the card is not mounted.
     */
    bool load(SDCard& card, uint16_t* skipped_lines = nullptr);

    /**
     * @brief Serialize the current in-memory values to ROBOT_SETTINGS_FILE,
     * overwriting any previous content.
     *
     * A successful call also advances save_revision() by one -- see its own
     * comment for what that is for.
     */
    bool save(SDCard& card) const;

    /**
     * @brief How many times save() has written the file successfully during
     * this boot. Starts at 0 on every boot, load() does not touch it.
     *
     * Exists so a remote client can tell "has this robot's configuration
     * changed since I last looked" without re-reading all ~55 keys of
     * "settings -list_all" to diff them itself. Deliberately NOT wired into
     * ManifestCatalog::kConfigRevision: that field's documented meaning is
     * "has the published topic/schema catalog changed" (BTP/docs/commands.md
     * section 3), which is genuinely fixed at compile time in this firmware
     * -- repurposing a wire field to mean something else would be a protocol
     * contract change made unilaterally from this side, the same class of
     * problem include/bally_channels.h exists to prevent. This counter is a
     * shell-only convenience instead.
     */
    uint32_t save_revision() const noexcept { return save_revision_; }

    /// Revert every field to its compiled-in default (in memory only).
    void reset_all();

    /// Revert one module's fields to their compiled-in defaults (in memory
    /// only). @return false when the module name is not found.
    bool reset_module(const char* module);

    bool get(const char* module, const char* key, std::string& value_out) const;
    bool set(const char* module, const char* key, const char* value);

    /// Append "module.key=value\n" lines to out. module==nullptr/"" lists
    /// every setting.
    void list(const char* module, std::string& out) const;
    void list_all(std::string& out) const;

    const SettingsData& data() const { return data_; }

    // ---------------- applying changes without a reboot ----------------
    //
    // "settings -set" only ever changed the in-memory copy, and almost
    // nothing read it again until the next boot -- so a remote edit looked
    // like it had worked and had no effect. An applier is the missing half:
    // the piece of code that knows how to push one module's values into the
    // subsystem that already consumed them at startup.
    //
    // This class deliberately does not know any of those subsystems (see the
    // README's coupling rule: RobotSettings depends on nothing). Whoever owns
    // them registers a callback here instead.

    /// Returns false and does nothing when the callback should fail; the
    /// module name is reported back to the caller as Failed.
    using ApplyFn = std::function<bool(const SettingsData&)>;

    enum class ApplyResult : uint8_t {
        Applied,        ///< the applier ran and reported success
        Failed,         ///< the applier ran and reported failure
        NoApplier,      ///< real module, but nothing can apply it live
        UnknownModule   ///< no such module in the settings table
    };

    /**
     * @brief Bind the code that applies one module's values live.
     * @return false when the table is full or the module name is unknown.
     */
    bool register_applier(const char* module, ApplyFn apply);

    /** @brief Run one module's applier against the current in-memory values. */
    ApplyResult apply(const char* module);

    /**
     * @brief Run every registered applier.
     * @param applied/failed Optional counts; modules with no applier are
     * simply skipped, since "requires a reboot" is not a failure.
     */
    void apply_all(size_t* applied, size_t* failed);

    /**
     * @brief Append the settings that differ between memory and the file on
     * the card, one "module.key memory=X file=Y" line each.
     *
     * Exists to kill a specific recurring mistake: editing values, not
     * saving, and later wondering why a reboot lost them.
     *
     * @return false when the card is not mounted or the file cannot be read.
     */
    bool diff(SDCard& card, std::string& out) const;

    /// Whether a module name appears anywhere in the settings table.
    static bool module_exists(const char* module);

    static const char* apply_result_text(ApplyResult result);

    /// Derived from sample_micros; replaces the old FREQ_EKF macro.
    float freq_ekf() const;

    /**
     * @brief Register the "settings" shell module (get/set/list/list_all/
     * save/load/reset/reset_all), backed by this instance.
     * @param mark_direct_output Called after "save" sends its confirmation,
     * so that response is not itself retained in the PSRAM log (see
     * ROBOT::sendNextShellOutputDirect — the same reasoning it documents).
     */
    void register_shell_commands(TinyShell& shell, Logger& logger, SDCard& sd_card,
                                 std::function<void()> mark_direct_output);

private:
    SettingsData data_;

    // See save_revision()'s comment. mutable because save() is logically a
    // const operation on data_ (it only serializes it) and this counter is
    // bookkeeping about the operation, not part of what data() returns.
    mutable uint32_t save_revision_ = 0U;

    static const SettingEntry kTable[];
    static const size_t kTableCount;

    // Room for one per module that can meaningfully be applied live; the
    // pins_* groups never can (see ROBOT::registerSettingsAppliers).
    static constexpr size_t kMaxAppliers = 8;

    struct ApplierEntry {
        const char* module = nullptr;
        ApplyFn     apply;
    };
    ApplierEntry appliers_[kMaxAppliers]{};
    size_t applier_count_ = 0;

    /// Shared file reader/parser behind load() and diff(): fills `out`
    /// without touching data_. Returns false when the card is not mounted or
    /// the file cannot be read.
    bool parse_file(SDCard& card, SettingsData& out,
                    uint16_t* skipped_lines) const;

    static bool format_entry(const SettingsData& d, const SettingEntry& e, std::string& out);
    static bool parse_into(SettingsData& d, const SettingEntry& e, const char* value);
    static const SettingEntry* find(const char* module, const char* key);
};

#endif // ROBOT_SETTINGS_H
