#ifndef OTA_UPDATER_H
#define OTA_UPDATER_H

// autor: Alison Tristao
// email: AlisonTristao@hotmail.com

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_event.h"
#include "esp_http_server.h"

#include <Settings.h>
#include <SharedMessageTypes.h>

class SDCard;
class Flags_out;

// Reports scan/connect/upload progress and failures (with reason, when
// known) so they land in the retained log even though ESP-NOW is off the
// air for the whole time OTA is active; see Logger::insert_log. A plain
// function pointer, not std::function, so a captureless lambda at the call
// site is enough.
using OtaLogCallback = void (*)(logType type, const char* msg);

/**
 * @brief Runtime-tunable OTA settings (RobotSettings, module "ota"): timing,
 * the ESP-NOW home channel restored on cancel(), the mDNS identity, and the
 * upload password. This struct is the single canonical home for the
 * numeric defaults — SettingsData (lib/RobotSettings) derives its own field
 * defaults from a default-constructed OtaTuning instead of repeating the
 * literals, so a robot that never calls configure() still behaves the same
 * either way.
 *
 * hostname/instance_name/password are borrowed pointers (e.g. into a
 * SettingsData instance that outlives the OTAUpdater): configure() copies
 * them into its own fixed buffers immediately, so nothing needs to stay
 * valid afterwards. They default to nullptr, and configure() leaves the
 * corresponding buffer untouched rather than substituting a second,
 * driftable copy of RobotSettings' own defaults — the only place those
 * literals belong. An untouched (empty) password means /update accepts any
 * upload, same as before this existed.
 */
struct OtaTuning {
    uint32_t led_step_ms        = 150;    // carousel step while OTA is active
    uint32_t led_hold_ms        = 200;    // refresh window for solid status LEDs
    uint32_t led_fail_hold_ms   = 800;    // red "connect failed" hold before retrying
    uint32_t connect_timeout_ms = 10000;  // per-network connect timeout
    uint32_t retry_scan_ms      = 5000;   // delay before re-scanning when nothing matched
    uint8_t  espnow_channel     = 1;      // channel restored after leaving OTA
    const char* hostname        = nullptr;      // mDNS hostname: http://<name>.local/
    const char* instance_name   = nullptr;      // mDNS human-readable instance name
    const char* password        = nullptr;      // required in the X-OTA-Password header to POST /update
};

/**
 * @brief Join a known Wi-Fi network and accept a firmware upload over HTTP,
 * from inside the DEBUG state.
 *
 * Networks are read from a plain text file on the SD card
 * (OTA_WIFI_LIST_FILE, "ssid,password" per line, see Settings.h); only
 * networks that both appear in that file and are currently visible over the
 * air are attempted, in file order. Any button press cancels back to plain
 * ESP-NOW, unless a firmware upload is already in flight.
 *
 * The LEDs report progress: a one-at-a-time carousel while scanning, solid
 * yellow while connecting to a candidate, solid red (OtaTuning::led_fail_hold_ms)
 * when a connect attempt fails before the next candidate is tried, solid
 * green once connected and serving, and solid blue while a firmware upload
 * is actually being written. Once connected, the device also answers to
 * "<hostname>.local" over mDNS (see OtaTuning::hostname) so the upload URL
 * does not depend on knowing its IP.
 *
 * A write that finishes successfully does not reboot on its own: it just
 * sets the new image as the active boot partition, then leaves OTA the same
 * way a button press does (Wi-Fi down, ESP-NOW channel restored). The new
 * firmware only takes over on the next reset — power cycle, watchdog, or a
 * remote "ota reboot" — so ESP-NOW stays reachable in between to give that
 * command, instead of the robot going dark mid-air-upload for a reboot that
 * happens whether or not the upload actually worked.
 */
class OTAUpdater {
public:
    OTAUpdater() = default;
    ~OTAUpdater() = default;

    OTAUpdater(const OTAUpdater&) = delete;
    OTAUpdater& operator=(const OTAUpdater&) = delete;

    /**
     * @brief Register the SD card and LED outputs this manager will use and
     * install the Wi-Fi/IP event handlers. Call once during ROBOT::init().
     * @param log_cb Optional; receives every OTA event from this point on
     * (including failures inside begin() itself), so pass it here rather
     * than a separate setter.
     */
    bool begin(SDCard& card, Flags_out& leds, OtaLogCallback log_cb = nullptr);

    /**
     * @brief Apply runtime-tunable settings (see OtaTuning): copies the
     * timing/channel fields and snprintf's hostname/instance_name into
     * their own buffers. Safe to call before begin(), any time after, or
     * not at all — a default-constructed OtaTuning is already in effect
     * otherwise.
     */
    void configure(const OtaTuning& tuning);

    /// mDNS hostname currently in effect (without ".local"); see OtaTuning.
    const char* hostname() const { return hostname_; }

    /**
     * @brief Start scanning for a known network. Entry point for the
     * "ota_start" DEBUG shell command.
     * @return false when the SD card is not mounted or no networks are
     * stored; true when already active or the scan was scheduled.
     */
    bool start();

    /**
     * @brief Non-blocking tick: drives scanning/connecting, the LED
     * carousel and button-triggered cancellation. Call every DEBUG pass.
     * @param button_flags Current button flags; any bit set cancels OTA.
     */
    void process(uint8_t button_flags);

    /**
     * @brief Leave the OTA sub-mode: disconnect Wi-Fi, stop the HTTP
     * server, restore the ESP-NOW channel and clear the LEDs. Safe to call
     * repeatedly or while already idle. Ignored while a firmware upload is
     * being written (see is_flashing()).
     */
    void cancel();

    enum class Phase : uint8_t { IDLE, SCANNING, CONNECTING, CONNECT_FAILED, SERVING };

    Phase phase() const { return phase_.load(std::memory_order_acquire); }
    bool is_active() const { return phase() != Phase::IDLE; }
    bool is_flashing() const {
        return flashing_.load(std::memory_order_acquire);
    }
    const char* connected_ssid() const { return connected_ssid_; }
    const char* connected_ip() const { return connected_ip_; }

    /**
     * @brief Append one "ssid,password" line to OTA_WIFI_LIST_FILE.
     * Rejects entries containing a comma, since it is the file/shell
     * delimiter, or that would not fit OTA_SSID_MAX_LEN/OTA_PASS_MAX_LEN.
     */
    bool add_network(const char* ssid, const char* password);

    /**
     * @brief Remove the network at the given index (as returned by
     * get_network) by rewriting the file without that line.
     */
    bool remove_network(uint16_t index);

    /**
     * @brief Read one stored network's SSID by index. Never returns the
     * password, so it stays safe to echo back over the shell.
     * @return true when index is valid; ssid_out is always null-terminated.
     */
    bool get_network(uint16_t index, char* ssid_out, size_t capacity) const;

    /**
     * @brief Number of networks currently stored in OTA_WIFI_LIST_FILE.
     */
    uint16_t network_count() const;

private:
    struct Credential {
        char ssid[OTA_SSID_MAX_LEN];
        char password[OTA_PASS_MAX_LEN];
    };

    SDCard* card_ = nullptr;
    Flags_out* leds_ = nullptr;
    OtaLogCallback log_cb_ = nullptr;
    OtaTuning tuning_;
    // Empty until configure() is given real values (see OtaTuning); mDNS
    // just stays off, and /update accepts any upload, rather than
    // substituting a literal here too.
    char hostname_[OTA_MDNS_NAME_MAX_LEN] = {};
    char instance_name_[OTA_MDNS_NAME_MAX_LEN] = {};
    char password_[OTA_PASSWORD_MAX_LEN] = {};
    bool events_registered_ = false;
    bool mdns_ready_ = false;

    std::atomic<Phase> phase_{Phase::IDLE};
    std::atomic<bool> flashing_{false};

    // Wi-Fi scan/connect bookkeeping, updated from the esp_event loop task.
    std::atomic<bool> scan_done_{false};
    std::atomic<bool> got_ip_{false};
    std::atomic<bool> disconnected_{false};
    std::atomic<uint8_t> disconnect_reason_{0}; // WIFI_REASON_*, valid when disconnected_ is set
    bool scan_in_flight_ = false;
    uint32_t next_scan_ms_ = 0;
    uint32_t connect_deadline_ms_ = 0;
    uint32_t retry_at_ms_ = 0; // Phase::CONNECT_FAILED: when to try the next candidate

    // Networks loaded from OTA_WIFI_LIST_FILE for the active connect attempt.
    Credential candidates_[OTA_MAX_NETWORKS];
    uint8_t candidate_count_ = 0;
    uint8_t candidate_index_ = 0;

    // Separate scratch buffer for the wifi_add/list/remove admin commands,
    // so they never disturb an in-flight connect attempt's candidate list.
    mutable Credential scratch_[OTA_MAX_NETWORKS];

    char connected_ssid_[OTA_SSID_MAX_LEN] = {};
    char connected_ip_[16] = {};

    // LED carousel state.
    uint8_t carousel_index_ = 0;
    uint32_t carousel_next_ms_ = 0;

    httpd_handle_t server_ = nullptr;
    esp_event_handler_instance_t wifi_event_instance_ = nullptr;
    esp_event_handler_instance_t ip_event_instance_ = nullptr;

    uint16_t parse_file(Credential* out, uint16_t max_entries) const;
    bool load_candidates();
    void handle_scan_done();
    void try_next_candidate();
    void fail_candidate();
    void advance_carousel();
    void update_status_led(Phase phase);
    bool start_http_server();
    void stop_http_server();
    void ensure_mdns();

    void log(logType type, const char* fmt, ...) const
        __attribute__((format(printf, 3, 4)));

    static void wifi_event_handler(void* arg, esp_event_base_t base,
                                   int32_t id, void* data);
    static void ip_event_handler(void* arg, esp_event_base_t base,
                                 int32_t id, void* data);

    static esp_err_t handle_root_get(httpd_req_t* req);
    static esp_err_t handle_update_post(httpd_req_t* req);

    /**
     * @brief esp_timer callback (ESP_TIMER_TASK), scheduled from
     * handle_update_post() after a successful write. Restores ESP-NOW —
     * cancel() calls httpd_stop(), which would deadlock if called directly
     * from inside the httpd request handler that just finished. Does not
     * reboot; the new image is already the active boot partition (see
     * esp_ota_set_boot_partition() in handle_update_post()), so any
     * subsequent reset — including a remote "ota reboot" — boots into it.
     */
    static void finish_update(void* arg);
};

#endif // OTA_UPDATER_H
