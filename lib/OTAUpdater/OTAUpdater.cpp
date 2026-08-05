#include <OTAUpdater.h>

#include <cstdio>
#include <cstring>

#include "esp_now.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include <Flags.h>
#include <SDCard.h>

namespace {

uint32_t now_ms() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

void reboot_after_response(void*) {
    esp_restart();
}

// Minimal upload page: a plain HTML form cannot POST raw bytes, so a small
// inline script reads the picked file and posts its bytes directly to
// /update, matching what handle_update_post expects (no multipart parsing).
const char kUploadPage[] =
    "<!DOCTYPE html><html><body>"
    "<h3>BallyRobot OTA</h3>"
    "<input type='file' id='f'><button onclick='u()'>Upload</button>"
    "<pre id='s'></pre>"
    "<script>"
    "function u(){"
    "var f=document.getElementById('f').files[0];"
    "if(!f)return;"
    "document.getElementById('s').textContent='Uploading...';"
    "fetch('/update',{method:'POST',body:f})"
    ".then(function(r){return r.text();})"
    ".then(function(t){document.getElementById('s').textContent=t;})"
    ".catch(function(e){document.getElementById('s').textContent='Error: '+e;});"
    "}"
    "</script></body></html>";

} // namespace

// ==============================================================================
// Lifecycle
// ==============================================================================

bool OTAUpdater::begin(SDCard& card, Flags_out& leds) {
    card_ = &card;
    leds_ = &leds;

    if (events_registered_) return true;

    if (esp_netif_create_default_wifi_sta() == nullptr) return false;

    if (esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &OTAUpdater::wifi_event_handler,
            this, &wifi_event_instance_) != ESP_OK) {
        return false;
    }

    if (esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, &OTAUpdater::ip_event_handler,
            this, &ip_event_instance_) != ESP_OK) {
        return false;
    }

    events_registered_ = true;
    return true;
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
    if (card_ == nullptr || !card_->is_mounted()) return false;
    if (!load_candidates()) return false;

    scan_done_.store(false, std::memory_order_relaxed);
    got_ip_.store(false, std::memory_order_relaxed);
    disconnected_.store(false, std::memory_order_relaxed);
    carousel_index_ = 0;
    carousel_next_ms_ = now_ms();

    wifi_scan_config_t scan_config{};
    scan_in_flight_ = esp_wifi_scan_start(&scan_config, false) == ESP_OK;
    next_scan_ms_ = now_ms() + OTA_RETRY_SCAN_MS;

    phase_.store(Phase::SCANNING, std::memory_order_release);
    return scan_in_flight_;
}

void OTAUpdater::handle_scan_done() {
    scan_in_flight_ = false;

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    if (ap_count > OTA_MAX_SCAN_RESULTS) ap_count = OTA_MAX_SCAN_RESULTS;

    wifi_ap_record_t records[OTA_MAX_SCAN_RESULTS];
    if (ap_count > 0) esp_wifi_scan_get_ap_records(&ap_count, records);

    for (; candidate_index_ < candidate_count_; ++candidate_index_) {
        bool visible = false;
        for (uint16_t i = 0; i < ap_count; ++i) {
            if (strncmp(reinterpret_cast<const char*>(records[i].ssid),
                        candidates_[candidate_index_].ssid,
                        sizeof(records[i].ssid)) == 0) {
                visible = true;
                break;
            }
        }
        if (visible) break;
    }

    if (candidate_index_ >= candidate_count_) {
        // None of the stored networks are visible right now; keep retrying.
        candidate_index_ = 0;
        next_scan_ms_ = now_ms() + OTA_RETRY_SCAN_MS;
        return;
    }

    wifi_config_t wifi_config{};
    snprintf(reinterpret_cast<char*>(wifi_config.sta.ssid),
             sizeof(wifi_config.sta.ssid), "%s",
             candidates_[candidate_index_].ssid);
    snprintf(reinterpret_cast<char*>(wifi_config.sta.password),
             sizeof(wifi_config.sta.password), "%s",
             candidates_[candidate_index_].password);

    if (esp_wifi_set_config(WIFI_IF_STA, &wifi_config) != ESP_OK ||
        esp_wifi_connect() != ESP_OK) {
        try_next_candidate();
        return;
    }

    connect_deadline_ms_ = now_ms() + OTA_CONNECT_TIMEOUT_MS;
    phase_.store(Phase::CONNECTING, std::memory_order_release);
}

void OTAUpdater::try_next_candidate() {
    ++candidate_index_;
    next_scan_ms_ = now_ms();
    phase_.store(Phase::SCANNING, std::memory_order_release);
}

void OTAUpdater::advance_carousel() {
    const uint32_t now = now_ms();
    if (static_cast<int32_t>(now - carousel_next_ms_) < 0) return;

    leds_->setFlag(carousel_index_, OTA_LED_STEP_MS);
    carousel_index_ = static_cast<uint8_t>((carousel_index_ + 1) % 4);
    carousel_next_ms_ = now + OTA_LED_STEP_MS;
}

void OTAUpdater::process(uint8_t button_flags) {
    const Phase phase = phase_.load(std::memory_order_acquire);
    if (phase == Phase::IDLE) return;

    if (button_flags != 0 && !flashing_.load(std::memory_order_acquire)) {
        cancel();
        return;
    }

    advance_carousel();
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
                if (!scan_in_flight_) next_scan_ms_ = now + OTA_RETRY_SCAN_MS;
            }
            break;

        case Phase::CONNECTING:
            if (got_ip_.exchange(false, std::memory_order_acq_rel)) {
                if (start_http_server()) {
                    phase_.store(Phase::SERVING, std::memory_order_release);
                } else {
                    esp_wifi_disconnect();
                    try_next_candidate();
                }
            } else if (disconnected_.exchange(false, std::memory_order_acq_rel) ||
                      static_cast<int32_t>(now - connect_deadline_ms_) >= 0) {
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

    stop_http_server();
    esp_wifi_disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

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

    httpd_register_uri_handler(server_, &root_uri);
    httpd_register_uri_handler(server_, &update_uri);
    return true;
}

void OTAUpdater::stop_http_server() {
    if (server_ == nullptr) return;
    httpd_stop(server_);
    server_ = nullptr;
}

esp_err_t OTAUpdater::handle_root_get(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, kUploadPage, HTTPD_RESP_USE_STRLEN);
}

esp_err_t OTAUpdater::handle_update_post(httpd_req_t* req) {
    auto* self = static_cast<OTAUpdater*>(req->user_ctx);

    if (req->content_len == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }

    const esp_partition_t* update_partition =
        esp_ota_get_next_update_partition(nullptr);
    if (update_partition == nullptr) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "No OTA partition available");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle = 0;
    if (esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle) !=
        ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "esp_ota_begin failed");
        return ESP_FAIL;
    }

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
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "OTA write failed");
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OK, rebooting...\n");

    // Let the HTTP response flush before resetting.
    esp_timer_handle_t reboot_timer;
    const esp_timer_create_args_t timer_args = {
        .callback = &reboot_after_response,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ota_reboot",
        .skip_unhandled_events = false,
    };
    esp_timer_create(&timer_args, &reboot_timer);
    esp_timer_start_once(reboot_timer, 500000);

    return ESP_OK;
}

// ==============================================================================
// Event handlers
// ==============================================================================

void OTAUpdater::wifi_event_handler(void* arg, esp_event_base_t base,
                                    int32_t id, void* data) {
    (void)base;
    (void)data;
    auto* self = static_cast<OTAUpdater*>(arg);

    if (id == WIFI_EVENT_SCAN_DONE) {
        self->scan_done_.store(true, std::memory_order_release);
    } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
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
