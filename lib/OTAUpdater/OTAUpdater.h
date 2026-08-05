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

class SDCard;
class Flags_out;

/**
 * @brief Join a known Wi-Fi network and accept a firmware upload over HTTP,
 * from inside the DEBUG state.
 *
 * Networks are read from a plain text file on the SD card
 * (OTA_WIFI_LIST_FILE, "ssid,password" per line, see Settings.h); only
 * networks that both appear in that file and are currently visible over the
 * air are attempted, in file order. While active, LEDs run a one-at-a-time
 * carousel; any button press cancels back to plain ESP-NOW, unless a
 * firmware upload is already in flight.
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
     */
    bool begin(SDCard& card, Flags_out& leds);

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

    enum class Phase : uint8_t { IDLE, SCANNING, CONNECTING, SERVING };

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
    bool events_registered_ = false;

    std::atomic<Phase> phase_{Phase::IDLE};
    std::atomic<bool> flashing_{false};

    // Wi-Fi scan/connect bookkeeping, updated from the esp_event loop task.
    std::atomic<bool> scan_done_{false};
    std::atomic<bool> got_ip_{false};
    std::atomic<bool> disconnected_{false};
    bool scan_in_flight_ = false;
    uint32_t next_scan_ms_ = 0;
    uint32_t connect_deadline_ms_ = 0;

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
    void advance_carousel();
    bool start_http_server();
    void stop_http_server();

    static void wifi_event_handler(void* arg, esp_event_base_t base,
                                   int32_t id, void* data);
    static void ip_event_handler(void* arg, esp_event_base_t base,
                                 int32_t id, void* data);

    static esp_err_t handle_root_get(httpd_req_t* req);
    static esp_err_t handle_update_post(httpd_req_t* req);
};

#endif // OTA_UPDATER_H
