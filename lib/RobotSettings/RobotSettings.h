#ifndef ROBOT_SETTINGS_H
#define ROBOT_SETTINGS_H

// autor: Alison Tristao

#include <cstddef>
#include <cstdint>
#include <string>

// For OtaTuning's default-constructed values, which back the ota_* field
// defaults below — kept in one place (see OTAUpdater.h) instead of
// repeating the literals here too.
#include <OTAUpdater.h>

class SDCard;

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
    // ROBOT::init(). Numeric defaults are pulled from a default-constructed
    // OtaTuning (the canonical copy) instead of repeating the literals here.
    // espnow_channel is also read directly by ROBOT::configureCommunication()
    // to bring ESP-NOW up on the same channel OTA restores on cancel().
    uint32_t ota_led_step_ms        = OtaTuning{}.led_step_ms;
    uint32_t ota_led_hold_ms        = OtaTuning{}.led_hold_ms;
    uint32_t ota_led_fail_hold_ms   = OtaTuning{}.led_fail_hold_ms;
    uint32_t ota_connect_timeout_ms = OtaTuning{}.connect_timeout_ms;
    uint32_t ota_retry_scan_ms      = OtaTuning{}.retry_scan_ms;
    uint8_t  espnow_channel         = OtaTuning{}.espnow_channel;
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
     */
    bool save(SDCard& card) const;

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

    /// Derived from sample_micros; replaces the old FREQ_EKF macro.
    float freq_ekf() const;

private:
    SettingsData data_;

    static const SettingEntry kTable[];
    static const size_t kTableCount;

    static bool format_entry(const SettingsData& d, const SettingEntry& e, std::string& out);
    static bool parse_into(SettingsData& d, const SettingEntry& e, const char* value);
    static const SettingEntry* find(const char* module, const char* key);
};

#endif // ROBOT_SETTINGS_H
