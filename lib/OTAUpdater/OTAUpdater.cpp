#include <OTAUpdater.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mdns.h"

#include <Flags.h>
#include <SDCard.h>
#include <TinyShell.h>
#include <Logger.h>
#include <USBMassStorage.h>
#include <StateMachine.h>

namespace {

uint32_t now_ms() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

// Short, human-readable hint for the common WIFI_EVENT_STA_DISCONNECTED
// reasons; the numeric code is always logged alongside this, since not
// every reason is worth naming here.
const char* disconnect_reason_str(uint8_t reason) {
    switch (reason) {
        case WIFI_REASON_NO_AP_FOUND:
        case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
        case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
        case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:
            return "AP not found";
        case WIFI_REASON_AUTH_FAIL:
        case WIFI_REASON_AUTH_EXPIRE:
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_HANDSHAKE_TIMEOUT:
            return "authentication failed, check the password";
        case WIFI_REASON_ASSOC_FAIL:
        case WIFI_REASON_ASSOC_LEAVE:
        case WIFI_REASON_ASSOC_NOT_AUTHED:
            return "association rejected by the AP";
        case WIFI_REASON_BEACON_TIMEOUT:
            return "beacon timeout, weak signal";
        default:
            return "unspecified";
    }
}

const char* auth_mode_str(wifi_auth_mode_t mode) {
    switch (mode) {
        case WIFI_AUTH_OPEN:          return "OPEN";
        case WIFI_AUTH_WEP:           return "WEP";
        case WIFI_AUTH_WPA_PSK:       return "WPA-PSK";
        case WIFI_AUTH_WPA2_PSK:      return "WPA2-PSK";
        case WIFI_AUTH_WPA_WPA2_PSK:  return "WPA/WPA2-PSK";
        case WIFI_AUTH_WPA3_PSK:      return "WPA3-PSK";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3-PSK";
        case WIFI_AUTH_OWE:           return "OWE";
        default:                      return "other";
    }
}

// Minimal upload page: a plain HTML form cannot POST raw bytes, so a small
// inline script reads the picked file and posts its bytes directly to
// /update, matching what handle_update_post expects (no multipart parsing).
// %s is the configured mDNS hostname (OtaTuning::hostname) — formatted into
// a stack buffer per request in handle_root_get(), since it's a runtime
// setting now and can no longer be pasted in at compile time.
const char kUploadPageFmt[] =
    "<!DOCTYPE html><html><body>"
    "<h3>BallyRobot OTA (%s.local)</h3>"
    "<input type='file' id='f'><button onclick='u()'>Upload</button>"
    "<pre id='s'></pre>"
    "<script>"
    "function u(){"
    "var f=document.getElementById('f').files[0];"
    "if(!f)return;"
    "var p=prompt('OTA password (leave blank if none):')||'';"
    "document.getElementById('s').textContent='Uploading...';"
    "fetch('/update',{method:'POST',headers:{'X-OTA-Password':p},body:f})"
    ".then(function(r){return r.text();})"
    ".then(function(t){document.getElementById('s').textContent=t;})"
    ".catch(function(e){document.getElementById('s').textContent='Error: '+e;});"
    "}"
    "</script></body></html>";

} // namespace

// ==============================================================================
// Lifecycle
// ==============================================================================

bool OTAUpdater::begin(SDCard& card, Flags_out& leds, OtaLogCallback log_cb) {
    card_ = &card;
    leds_ = &leds;
    log_cb_ = log_cb;

    if (events_registered_) return true;

    // The STA netif itself is created earlier, in
    // ROBOT::configureCommunication() — it has to exist before
    // esp_wifi_start() for the DHCP client to ever run (see the comment
    // there). Here we only add OTA's own event handlers on top of it.
    if (esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &OTAUpdater::wifi_event_handler,
            this, &wifi_event_instance_) != ESP_OK) {
        log(logType::ERRO, "OTA: failed to register the Wi-Fi event handler");
        return false;
    }

    if (esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, &OTAUpdater::ip_event_handler,
            this, &ip_event_instance_) != ESP_OK) {
        log(logType::ERRO, "OTA: failed to register the IP event handler");
        return false;
    }

    events_registered_ = true;

    // Best-effort: OTA still works by IP if mDNS never comes up (or was
    // never configured with a hostname at all — see ensure_mdns()).
    ensure_mdns();
    if (hostname_[0] != '\0' && !mdns_ready_) {
        log(logType::WARN,
            "OTA: mDNS init failed; use 'ota status' for the IP instead of %s.local",
            hostname_);
    }

    return true;
}

void OTAUpdater::configure(const OtaTuning& tuning) {
    tuning_ = tuning;

    // A null pointer here just means "not given" — leave whatever value is
    // already in place (e.g. from an earlier configure() call) rather than
    // blanking it out.
    if (tuning.hostname != nullptr) {
        snprintf(hostname_, sizeof(hostname_), "%s", tuning.hostname);
    }
    if (tuning.instance_name != nullptr) {
        snprintf(instance_name_, sizeof(instance_name_), "%s", tuning.instance_name);
    }
    if (tuning.password != nullptr) {
        snprintf(password_, sizeof(password_), "%s", tuning.password);
    }
}

void OTAUpdater::ensure_mdns() {
    if (mdns_ready_ || hostname_[0] == '\0') return;
    if (mdns_init() != ESP_OK) return;

    mdns_hostname_set(hostname_);
    mdns_instance_name_set(instance_name_[0] != '\0' ? instance_name_ : hostname_);
    mdns_service_add(nullptr, "_http", "_tcp", 80, nullptr, 0);
    mdns_ready_ = true;
}

void OTAUpdater::log(logType type, const char* fmt, ...) const {
    char buffer[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    // Mirrored straight to UART with the real wall-clock uptime at the
    // instant this fired, independent of ESP-NOW/PSRAM retention — during
    // OTA, insert_log()'d entries only leave the ring once cancel() gives
    // the radio back to ESP-NOW, so their delivery order/spacing over the
    // air reflects when the backlog got flushed, not when each one
    // actually happened. `pio device monitor` shows the true timing.
    switch (type) {
        case logType::ERRO: ESP_LOGE("OTA", "%s", buffer); break;
        case logType::WARN: ESP_LOGW("OTA", "%s", buffer); break;
        default:            ESP_LOGI("OTA", "%s", buffer); break;
    }

    if (log_cb_ != nullptr) log_cb_(type, buffer);
}

// ==============================================================================
// Wifi credential file (SD card)
// ==============================================================================

uint16_t OTAUpdater::parse_file(Credential* out, uint16_t max_entries) const {
    if (card_ == nullptr || out == nullptr || max_entries == 0) return 0;

    char buffer[OTA_MAX_NETWORKS * (OTA_SSID_MAX_LEN + OTA_PASS_MAX_LEN)];
    size_t bytes_read = 0;
    if (!card_->read_file(OTA_WIFI_LIST_FILE, buffer, sizeof(buffer) - 1,
                          &bytes_read)) {
        return 0;
    }
    buffer[bytes_read] = '\0';

    uint16_t count = 0;
    char* line_ctx = nullptr;
    char* line = strtok_r(buffer, "\r\n", &line_ctx);

    while (line != nullptr && count < max_entries) {
        char* comma = strchr(line, ',');
        if (comma != nullptr) {
            *comma = '\0';
            const char* ssid = line;
            const char* password = comma + 1;
            if (ssid[0] != '\0') {
                snprintf(out[count].ssid, sizeof(out[count].ssid), "%s", ssid);
                snprintf(out[count].password, sizeof(out[count].password),
                         "%s", password);
                ++count;
            }
        }
        line = strtok_r(nullptr, "\r\n", &line_ctx);
    }

    return count;
}

bool OTAUpdater::load_candidates() {
    candidate_count_ = static_cast<uint8_t>(
        parse_file(candidates_, OTA_MAX_NETWORKS));
    candidate_index_ = 0;
    return candidate_count_ > 0;
}

bool OTAUpdater::add_network(const char* ssid, const char* password) {
    if (card_ == nullptr || ssid == nullptr || password == nullptr ||
        ssid[0] == '\0') {
        return false;
    }

    // Comma is the file/shell delimiter; reject it instead of escaping it.
    if (strchr(ssid, ',') != nullptr || strchr(password, ',') != nullptr)
        return false;

    if (strlen(ssid) >= OTA_SSID_MAX_LEN ||
        strlen(password) >= OTA_PASS_MAX_LEN) {
        return false;
    }

    if (network_count() >= OTA_MAX_NETWORKS) return false;

    char line[OTA_SSID_MAX_LEN + OTA_PASS_MAX_LEN + 2];
    const int written = snprintf(line, sizeof(line), "%s,%s\n", ssid, password);
    if (written <= 0 || static_cast<size_t>(written) >= sizeof(line))
        return false;

    return card_->append_file(OTA_WIFI_LIST_FILE, line,
                              static_cast<size_t>(written));
}

bool OTAUpdater::remove_network(uint16_t index) {
    if (card_ == nullptr) return false;

    const uint16_t count = parse_file(scratch_, OTA_MAX_NETWORKS);
    if (index >= count) return false;

    char buffer[OTA_MAX_NETWORKS * (OTA_SSID_MAX_LEN + OTA_PASS_MAX_LEN)];
    size_t offset = 0;

    for (uint16_t i = 0; i < count; ++i) {
        if (i == index) continue;

        const int written = snprintf(buffer + offset, sizeof(buffer) - offset,
                                     "%s,%s\n", scratch_[i].ssid,
                                     scratch_[i].password);
        if (written <= 0 || static_cast<size_t>(written) >= sizeof(buffer) - offset)
            return false;
        offset += static_cast<size_t>(written);
    }

    return card_->write_file(OTA_WIFI_LIST_FILE, buffer, offset);
}

bool OTAUpdater::get_network(uint16_t index, char* ssid_out,
                             size_t capacity) const {
    if (ssid_out == nullptr || capacity == 0) return false;
    ssid_out[0] = '\0';

    const uint16_t count = parse_file(scratch_, OTA_MAX_NETWORKS);
    if (index >= count) return false;

    snprintf(ssid_out, capacity, "%s", scratch_[index].ssid);
    return true;
}

uint16_t OTAUpdater::network_count() const {
    return parse_file(scratch_, OTA_MAX_NETWORKS);
}

// ==============================================================================
// Scan / connect / serve state machine
// ==============================================================================

bool OTAUpdater::start() {
    if (phase_.load(std::memory_order_acquire) != Phase::IDLE) return true;

    if (card_ == nullptr || !card_->is_mounted()) {
        log(logType::ERRO, "OTA: cannot start, SD card is not mounted for the robot");
        return false;
    }
    if (!load_candidates()) {
        log(logType::ERRO,
            "OTA: cannot start, no networks stored in " OTA_WIFI_LIST_FILE
            " (use 'ota wifi_add')");
        return false;
    }

    scan_done_.store(false, std::memory_order_relaxed);
    got_ip_.store(false, std::memory_order_relaxed);
    disconnected_.store(false, std::memory_order_relaxed);
    carousel_index_ = 0;
    carousel_next_ms_ = now_ms();

    wifi_scan_config_t scan_config{};
    scan_in_flight_ = esp_wifi_scan_start(&scan_config, false) == ESP_OK;
    next_scan_ms_ = now_ms() + tuning_.retry_scan_ms;

    // From here on OTA is committed to the SCANNING phase even if this
    // particular esp_wifi_scan_start() call didn't succeed synchronously:
    // process()'s SCANNING case retries it on its own once next_scan_ms_
    // elapses. Returning false here (as this used to) left phase_ stuck in
    // SCANNING while callers -- believing start() had failed -- never drove
    // the state machine into DEBUG, so process() was never ticked and OTA
    // silently stranded itself until a full reboot. This is also what the
    // header's doc comment already promises: true "when the scan was
    // scheduled", regardless of whether it landed on the first try.
    phase_.store(Phase::SCANNING, std::memory_order_release);

    log(logType::INFO, "OTA: started, scanning for %u known network(s)%s",
        static_cast<unsigned>(candidate_count_),
        scan_in_flight_ ? "" : " (initial scan failed to start, retrying)");

    return true;
}

void OTAUpdater::handle_scan_done() {
    scan_in_flight_ = false;

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    if (ap_count > OTA_MAX_SCAN_RESULTS) ap_count = OTA_MAX_SCAN_RESULTS;

    wifi_ap_record_t records[OTA_MAX_SCAN_RESULTS];
    if (ap_count > 0) esp_wifi_scan_get_ap_records(&ap_count, records);

    // Keep the matching scan record, not just a visible/not-visible bit: its
    // channel + BSSID let esp_wifi_connect() go straight to the right AP
    // instead of re-sweeping every 2.4GHz channel to relocate it, which is
    // what was turning a normal connect into a ~20s stall.
    const wifi_ap_record_t* matched = nullptr;
    for (; candidate_index_ < candidate_count_; ++candidate_index_) {
        for (uint16_t i = 0; i < ap_count; ++i) {
            if (strncmp(reinterpret_cast<const char*>(records[i].ssid),
                        candidates_[candidate_index_].ssid,
                        sizeof(records[i].ssid)) == 0) {
                matched = &records[i];
                break;
            }
        }
        if (matched != nullptr) break;
    }

    if (matched == nullptr) {
        // None of the stored networks are visible right now; keep retrying.
        log(logType::WARN,
            "OTA: none of the %u known network(s) are visible, rescanning in %ums",
            static_cast<unsigned>(candidate_count_),
            static_cast<unsigned>(tuning_.retry_scan_ms));
        candidate_index_ = 0;
        next_scan_ms_ = now_ms() + tuning_.retry_scan_ms;
        return;
    }

    wifi_config_t wifi_config{};
    snprintf(reinterpret_cast<char*>(wifi_config.sta.ssid),
             sizeof(wifi_config.sta.ssid), "%s",
             candidates_[candidate_index_].ssid);
    snprintf(reinterpret_cast<char*>(wifi_config.sta.password),
             sizeof(wifi_config.sta.password), "%s",
             candidates_[candidate_index_].password);
    wifi_config.sta.channel = matched->primary;
    memcpy(wifi_config.sta.bssid, matched->bssid, sizeof(wifi_config.sta.bssid));
    wifi_config.sta.bssid_set = true;

    if (esp_wifi_set_config(WIFI_IF_STA, &wifi_config) != ESP_OK ||
        esp_wifi_connect() != ESP_OK) {
        log(logType::ERRO, "OTA: esp_wifi_connect() failed for '%s'",
            candidates_[candidate_index_].ssid);
        fail_candidate();
        return;
    }

    log(logType::INFO, "OTA: '%s' is visible, connecting...",
        candidates_[candidate_index_].ssid);

    connect_deadline_ms_ = now_ms() + tuning_.connect_timeout_ms;
    phase_.store(Phase::CONNECTING, std::memory_order_release);
}

void OTAUpdater::try_next_candidate() {
    ++candidate_index_;
    next_scan_ms_ = now_ms();
    phase_.store(Phase::SCANNING, std::memory_order_release);
}

void OTAUpdater::fail_candidate() {
    // Every failure path lands here, including a plain deadline timeout,
    // where the STA can still be associated (or mid-handshake) since we
    // only gave up waiting for DHCP — never actually tore down the link.
    // Without this, the next esp_wifi_connect() logs "sta is connected,
    // disconnect before connecting to new ap" and the driver eats time
    // disconnecting on its own before it can even start the new attempt.
    esp_wifi_disconnect();
    retry_at_ms_ = now_ms() + tuning_.led_fail_hold_ms;
    phase_.store(Phase::CONNECT_FAILED, std::memory_order_release);
}

void OTAUpdater::advance_carousel() {
    const uint32_t now = now_ms();
    if (static_cast<int32_t>(now - carousel_next_ms_) < 0) return;

    leds_->setFlag(carousel_index_, tuning_.led_step_ms);
    carousel_index_ = static_cast<uint8_t>((carousel_index_ + 1) % 4);
    carousel_next_ms_ = now + tuning_.led_step_ms;
}

void OTAUpdater::update_status_led(Phase phase) {
    switch (phase) {
        case Phase::SCANNING:
            advance_carousel();
            break;
        case Phase::CONNECTING:
            leds_->setFlag(LED_YELLOW, tuning_.led_hold_ms);
            break;
        case Phase::CONNECT_FAILED:
            leds_->setFlag(LED_RED, tuning_.led_hold_ms);
            break;
        case Phase::SERVING:
            leds_->setFlag(flashing_.load(std::memory_order_acquire)
                               ? LED_BLUE : LED_GREEN,
                           tuning_.led_hold_ms);
            break;
        case Phase::IDLE:
            break;
    }
}

void OTAUpdater::process(uint8_t button_flags) {
    const Phase phase = phase_.load(std::memory_order_acquire);
    if (phase == Phase::IDLE) return;

    if (button_flags != 0 && !flashing_.load(std::memory_order_acquire)) {
        cancel();
        return;
    }

    update_status_led(phase);
    const uint32_t now = now_ms();

    switch (phase) {
        case Phase::SCANNING:
            if (scan_done_.exchange(false, std::memory_order_acq_rel)) {
                handle_scan_done();
            } else if (!scan_in_flight_ &&
                      static_cast<int32_t>(now - next_scan_ms_) >= 0) {
                wifi_scan_config_t scan_config{};
                scan_in_flight_ =
                    esp_wifi_scan_start(&scan_config, false) == ESP_OK;
                if (!scan_in_flight_) next_scan_ms_ = now + tuning_.retry_scan_ms;
            }
            break;

        case Phase::CONNECTING:
            if (got_ip_.exchange(false, std::memory_order_acq_rel)) {
                if (start_http_server()) {
                    log(logType::INFO, "OTA: connected to '%s' at %s, serving updates",
                        connected_ssid_, connected_ip_);
                    phase_.store(Phase::SERVING, std::memory_order_release);
                } else {
                    log(logType::ERRO,
                        "OTA: connected to '%s' but the HTTP server failed to start",
                        connected_ssid_);
                    fail_candidate();
                }
            } else if (disconnected_.exchange(false, std::memory_order_acq_rel)) {
                const uint8_t reason =
                    disconnect_reason_.load(std::memory_order_relaxed);
                log(logType::ERRO, "OTA: '%s' disconnected (reason %u: %s)",
                    candidates_[candidate_index_].ssid,
                    static_cast<unsigned>(reason), disconnect_reason_str(reason));
                fail_candidate();
            } else if (static_cast<int32_t>(now - connect_deadline_ms_) >= 0) {
                log(logType::ERRO, "OTA: timed out connecting to '%s'",
                    candidates_[candidate_index_].ssid);
                fail_candidate();
            }
            break;

        case Phase::CONNECT_FAILED:
            if (static_cast<int32_t>(now - retry_at_ms_) >= 0) {
                try_next_candidate();
            }
            break;

        case Phase::SERVING:
        case Phase::IDLE:
            break;
    }
}

void OTAUpdater::cancel() {
    if (flashing_.load(std::memory_order_acquire)) return;
    if (phase_.load(std::memory_order_acquire) == Phase::IDLE) return;

    log(logType::INFO, "OTA: cancelled, restoring ESP-NOW");

    stop_http_server();
    esp_wifi_disconnect();
    esp_wifi_set_channel(tuning_.espnow_channel, WIFI_SECOND_CHAN_NONE);

    for (uint8_t i = 0; i < 4; ++i) leds_->setFlag(i, 1);

    connected_ssid_[0] = '\0';
    connected_ip_[0] = '\0';
    candidate_count_ = 0;
    candidate_index_ = 0;
    scan_in_flight_ = false;

    phase_.store(Phase::IDLE, std::memory_order_release);
}

// ==============================================================================
// HTTP OTA server
// ==============================================================================

bool OTAUpdater::start_http_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    if (httpd_start(&server_, &config) != ESP_OK) return false;

    const httpd_uri_t root_uri = {
        .uri = "/", .method = HTTP_GET,
        .handler = &OTAUpdater::handle_root_get, .user_ctx = this,
    };
    const httpd_uri_t update_uri = {
        .uri = "/update", .method = HTTP_POST,
        .handler = &OTAUpdater::handle_update_post, .user_ctx = this,
    };
    const httpd_uri_t status_uri = {
        .uri = "/status", .method = HTTP_GET,
        .handler = &OTAUpdater::handle_status_get, .user_ctx = this,
    };

    httpd_register_uri_handler(server_, &root_uri);
    httpd_register_uri_handler(server_, &update_uri);
    httpd_register_uri_handler(server_, &status_uri);
    return true;
}

void OTAUpdater::stop_http_server() {
    if (server_ == nullptr) return;
    httpd_stop(server_);
    server_ = nullptr;
}

esp_err_t OTAUpdater::handle_root_get(httpd_req_t* req) {
    auto* self = static_cast<OTAUpdater*>(req->user_ctx);

    char page[sizeof(kUploadPageFmt) + sizeof(self->hostname_)];
    snprintf(page, sizeof(page), kUploadPageFmt, self->hostname_);

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

// Polled by an external tool (e.g. the upload script behind
// platformio.ini's upload_command) against http://<hostname>.local/status
// to find out it is safe to POST /update, without having to guess at
// timeouts around when OTA actually finished connecting. Reachable at all
// only while phase() == SERVING, so "online" is always true here; ota_ready
// goes false while a firmware write is already in progress (is_flashing()).
esp_err_t OTAUpdater::handle_status_get(httpd_req_t* req) {
    auto* self = static_cast<OTAUpdater*>(req->user_ctx);
    const esp_app_desc_t* app_desc = esp_app_get_description();

    char body[256];
    snprintf(body, sizeof(body),
             "{\"device\":\"BallyRobot\",\"online\":true,\"firmware\":\"%s\",\"ota_ready\":%s}",
             app_desc->version,
             self->is_flashing() ? "false" : "true");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

esp_err_t OTAUpdater::handle_update_post(httpd_req_t* req) {
    auto* self = static_cast<OTAUpdater*>(req->user_ctx);

    // Empty password_ (never configured) means auth is off, same as before
    // this existed — see OtaTuning::password.
    if (self->password_[0] != '\0') {
        char given[OTA_PASSWORD_MAX_LEN] = {};
        if (httpd_req_get_hdr_value_str(req, "X-OTA-Password", given,
                                        sizeof(given)) != ESP_OK ||
            strcmp(given, self->password_) != 0) {
            self->log(logType::WARN, "OTA: upload rejected, wrong or missing password");
            httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "Wrong OTA password");
            return ESP_FAIL;
        }
    }

    if (req->content_len == 0) {
        self->log(logType::ERRO, "OTA: upload rejected, empty body");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }

    const esp_partition_t* update_partition =
        esp_ota_get_next_update_partition(nullptr);
    if (update_partition == nullptr) {
        self->log(logType::ERRO, "OTA: no free OTA partition available");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "No OTA partition available");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle = 0;
    if (esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle) !=
        ESP_OK) {
        self->log(logType::ERRO, "OTA: esp_ota_begin() failed");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "esp_ota_begin failed");
        return ESP_FAIL;
    }

    self->log(logType::INFO, "OTA: firmware upload started (%d bytes)",
              static_cast<int>(req->content_len));
    self->flashing_.store(true, std::memory_order_release);

    char buffer[1024];
    int remaining = static_cast<int>(req->content_len);
    bool write_error = false;

    while (remaining > 0) {
        const int chunk = remaining < static_cast<int>(sizeof(buffer))
                             ? remaining
                             : static_cast<int>(sizeof(buffer));
        const int received = httpd_req_recv(req, buffer, chunk);

        if (received == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (received <= 0) {
            write_error = true;
            break;
        }

        if (esp_ota_write(ota_handle, buffer, received) != ESP_OK) {
            write_error = true;
            break;
        }

        remaining -= received;
    }

    self->flashing_.store(false, std::memory_order_release);

    if (write_error || esp_ota_end(ota_handle) != ESP_OK ||
        esp_ota_set_boot_partition(update_partition) != ESP_OK) {
        self->log(logType::ERRO, "OTA: firmware write failed%s",
                  write_error ? " (transfer error)" : "");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "OTA write failed");
        return ESP_FAIL;
    }

    self->log(logType::INFO, "OTA: firmware written OK, waiting for reset");
    httpd_resp_sendstr(
        req, "OK. Reset the robot (or send 'ota reboot') to boot into it.\n");

    // Deferred to ESP_TIMER_TASK rather than run here: finish_update() calls
    // cancel(), which calls httpd_stop() — doing that from inside this very
    // request handler, on the httpd server's own task, deadlocks (the stop
    // waits for the task to go idle, but the task is stuck waiting on it).
    // The delay also lets this response finish sending before Wi-Fi drops.
    esp_timer_handle_t finish_timer;
    const esp_timer_create_args_t timer_args = {
        .callback = &OTAUpdater::finish_update,
        .arg = self,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ota_finish",
        .skip_unhandled_events = false,
    };
    esp_timer_create(&timer_args, &finish_timer);
    esp_timer_start_once(finish_timer, 500000);

    return ESP_OK;
}

void OTAUpdater::finish_update(void* arg) {
    auto* self = static_cast<OTAUpdater*>(arg);
    self->cancel();
    self->log(logType::INFO,
              "OTA: firmware staged as the active boot partition — reset "
              "the robot (or send 'ota reboot') to boot into it");
}

// ==============================================================================
// Event handlers
// ==============================================================================

void OTAUpdater::wifi_event_handler(void* arg, esp_event_base_t base,
                                    int32_t id, void* data) {
    (void)base;
    auto* self = static_cast<OTAUpdater*>(arg);

    if (id == WIFI_EVENT_SCAN_DONE) {
        self->scan_done_.store(true, std::memory_order_release);
    } else if (id == WIFI_EVENT_STA_CONNECTED) {
        // L2 association succeeded; esp_netif now starts the DHCP client on
        // its own. If GOT_IP never follows, the AP accepted the association
        // but is not (or not yet) handing out a lease to this MAC — that
        // points at the AP/router side (client cap, MAC approval, DHCP
        // pool), not at the connect attempt itself.
        auto* event = static_cast<wifi_event_sta_connected_t*>(data);
        self->log(logType::INFO,
                  "OTA: associated with '%.*s' on ch%u (%s), waiting for DHCP...",
                  static_cast<int>(event->ssid_len),
                  reinterpret_cast<const char*>(event->ssid),
                  static_cast<unsigned>(event->channel),
                  auth_mode_str(event->authmode));
    } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
        auto* event = static_cast<wifi_event_sta_disconnected_t*>(data);
        self->disconnect_reason_.store(event->reason, std::memory_order_relaxed);
        self->disconnected_.store(true, std::memory_order_release);
    }
}

void OTAUpdater::ip_event_handler(void* arg, esp_event_base_t base,
                                  int32_t id, void* data) {
    (void)base;
    auto* self = static_cast<OTAUpdater*>(arg);

    if (id == IP_EVENT_STA_GOT_IP) {
        auto* event = static_cast<ip_event_got_ip_t*>(data);
        snprintf(self->connected_ip_, sizeof(self->connected_ip_), IPSTR,
                 IP2STR(&event->ip_info.ip));
        if (self->candidate_index_ < self->candidate_count_) {
            snprintf(self->connected_ssid_, sizeof(self->connected_ssid_), "%s",
                     self->candidates_[self->candidate_index_].ssid);
        }
        self->got_ip_.store(true, std::memory_order_release);
    }
}

// ==============================================================================
// Shell commands
// ==============================================================================

void OTAUpdater::register_shell_commands(TinyShell& shell, Logger& logger, SDCard& sd_card,
                                         USBMassStorage& usb_storage,
                                         std::function<bool()> any_debug_test_active,
                                         std::function<void()> mark_direct_output) {
    shell.create_module("ota", "Wi-Fi OTA firmware updates (DEBUG state)");

    shell.add([this, &logger, &sd_card, &usb_storage, any_debug_test_active]() -> uint8_t {
        if (StateMachine::current_state.load(std::memory_order_acquire) !=
            DEBUG) {
            logger.insert_log(
                logType::ERRO, "OTA update requires DEBUG state; enter DEBUG first");
            return RESULT_ERROR;
        }

        if (usb_storage.is_active() || !sd_card.is_mounted()) {
            logger.insert_log(
                logType::ERRO, "OTA update unavailable: SD card is not mounted for robot");
            return RESULT_ERROR;
        }

        if (any_debug_test_active()) {
            logger.insert_log(
                logType::ERRO, "OTA update blocked: wait for the DEBUG test to finish");
            return RESULT_ERROR;
        }

        if (!start()) {
            // start() already logged the specific reason (unmounted card vs.
            // empty OTA_WIFI_LIST_FILE).
            logger.insert_log(logType::ERRO, "Failed to start OTA update");
            return RESULT_ERROR;
        }

        logger.insert_log(
            logType::INFO,
            "OTA update started: scanning for a known network; press any button to cancel");
        return RESULT_OK;
    }, "start", "Join a known Wi-Fi network and accept a firmware upload", "ota");

    shell.add([this, &logger, mark_direct_output]() -> uint8_t {
        const char* status = "idle";
        switch (phase()) {
            case OTAUpdater::Phase::SCANNING:       status = "scanning for a known network"; break;
            case OTAUpdater::Phase::CONNECTING:     status = "connecting"; break;
            case OTAUpdater::Phase::CONNECT_FAILED: status = "connect failed, retrying"; break;
            case OTAUpdater::Phase::SERVING:        status = "serving OTA updates"; break;
            case OTAUpdater::Phase::IDLE:            status = "idle"; break;
        }

        char text[160];
        if (phase() == OTAUpdater::Phase::SERVING) {
            snprintf(text, sizeof(text),
                     "OTA: %s, ssid=%s ip=%s (or http://%s.local/)",
                     status, connected_ssid(),
                     connected_ip(), hostname());
        } else {
            snprintf(text, sizeof(text), "OTA: %s", status);
        }

        logger.send_log_direct(logType::INFO, text);
        mark_direct_output();
        return RESULT_OK;
    }, "status", "Show the current OTA update sub-mode status", "ota");

    shell.add([&logger, mark_direct_output]() -> uint8_t {
        // Kept as an alias of "sys reboot", which is now the canonical name
        // (a restart is not an OTA concept). The dongle and TraceView already
        // depend on THIS name, so it does not go away.
        //
        // The deferred-restart timer below is duplicated in
        // utils/BallyRobot/BallyRobotShell.cpp (scheduleRestart) rather than
        // shared: OTAUpdater must not depend on the ROBOT composition root
        // (CONTRIBUTING.md, "Nova biblioteca" item 3), and a new library for
        // twelve lines of esp_timer boilerplate would cost more than it saves.
        // If one side changes, change the other.
        //
        // A successful upload sets the new image as the boot partition but
        // deliberately does not reboot on its own (see
        // OTAUpdater::finish_update) — this is the remote trigger for that,
        // reachable over ESP-NOW since OTA already handed the channel back.
        logger.send_log_direct(logType::INFO, "Rebooting...");
        mark_direct_output();

        esp_timer_handle_t reboot_timer;
        const esp_timer_create_args_t timer_args = {
            .callback = [](void*) { esp_restart(); },
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "shell_reboot",
            .skip_unhandled_events = false,
        };
        esp_timer_create(&timer_args, &reboot_timer);
        esp_timer_start_once(reboot_timer, 500000);

        return RESULT_OK;
    }, "reboot", "Alias of 'sys reboot': restart the robot now", "ota");

    shell.add([this, &logger](std::string ssid, std::string password) -> uint8_t {
        if (!add_network(ssid.c_str(), password.c_str())) {
            logger.insert_log(
                logType::ERRO,
                "Failed to add network: invalid ssid/password, list full or SD unavailable");
            return RESULT_ERROR;
        }

        logger.insert_logf(logType::INFO, "Network added: %s",
                           ssid.c_str());
        return RESULT_OK;
    }, "wifi_add", "Add a Wi-Fi network for OTA: ssid,password", "ota");

    shell.add([this, &logger](uint16_t index) -> uint8_t {
        if (!remove_network(index)) {
            logger.insert_log(logType::ERRO, "Invalid network index");
            return RESULT_ERROR;
        }

        logger.insert_logf(logType::INFO, "Network %u removed", index);
        return RESULT_OK;
    }, "wifi_remove", "Remove a stored Wi-Fi network by index", "ota");

    shell.add([this, &logger]() -> uint8_t {
        const uint16_t count = network_count();
        logger.insert_logf(logType::INFO, "OTA networks: %u", count);

        for (uint16_t index = 0; index < count; ++index) {
            char ssid[OTA_SSID_MAX_LEN];
            if (!get_network(index, ssid, sizeof(ssid))) continue;
            logger.insert_logf(logType::INFO, "[%u] %s", index, ssid);
        }

        return RESULT_OK;
    }, "wifi_list", "List Wi-Fi networks stored for OTA (SSID only)", "ota");
}
