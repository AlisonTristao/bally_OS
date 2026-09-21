#include "BleBtpServer.h"

#include <cstring>
#include <new>

#include "esp_log.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace {

constexpr const char* kTag = "BleBtpServer";
constexpr const char* kDeviceName = "BallyRobot";

// One chunk read/written off the RX characteristic can never exceed
// ATT_MTU-3, and this project's own negotiated ceiling never exceeds
// btp::kBleTransport's own note (BallyRobot.h) of 512 -- 512 is a
// comfortable, fixed upper bound for the scratch buffer a single write is
// copied into before handing it to the composition layer, with headroom
// over BLE 5's own max ATT_MTU (517).
constexpr std::size_t kMaxChunkSize = 512U;

// BTP v1 BLE service/characteristic UUIDs (BTP/docs/fragmentation-and-
// transports.md section 8, TAREFAS_TCP_BLE_ANDROID.txt T04's note) -- same
// three UUIDs BleTransport.cpp (TraceView side) advertises/expects. Written
// here in the SAME left-to-right byte order the UUID string itself reads
// (easy to check by eye against the doc); BLE_UUID128_INIT wants NimBLE's
// own little-endian wire order, so each array below is that same byte
// sequence REVERSED. Sanity-checked at start() via ble_uuid_to_str() logging
// the constructed UUIDs -- see its own comment for why that check exists.
//
// 547a1aae-676e-4b68-8e20-bace26cd0726
const ble_uuid128_t kServiceUuid =
    BLE_UUID128_INIT(0x26, 0x07, 0xcd, 0x26, 0xce, 0xba, 0x20, 0x8e,
                     0x68, 0x4b, 0x6e, 0x67, 0xae, 0x1a, 0x7a, 0x54);
// f9160b78-c242-42f4-8f5e-88df2c51cbe6 (RX, write)
const ble_uuid128_t kRxUuid =
    BLE_UUID128_INIT(0xe6, 0xcb, 0x51, 0x2c, 0xdf, 0x88, 0x5e, 0x8f,
                     0xf4, 0x42, 0x42, 0xc2, 0x78, 0x0b, 0x16, 0xf9);
// 20f96ede-2f3b-4e02-b2cf-be6bb75dbe35 (TX, notify)
const ble_uuid128_t kTxUuid =
    BLE_UUID128_INIT(0x35, 0xbe, 0x5d, 0xb7, 0x6b, 0xbe, 0xcf, 0xb2,
                     0x02, 0x4e, 0x3b, 0x2f, 0xde, 0x6e, 0xf9, 0x20);

std::uint16_t g_tx_val_handle = 0U;

const struct ble_gatt_chr_def kGattCharacteristics[] = {
    {
        .uuid = &kRxUuid.u,
        .access_cb = &BleBtpServer::gatt_access_handler,
        .flags = BLE_GATT_CHR_F_WRITE,
    },
    {
        .uuid = &kTxUuid.u,
        .access_cb = &BleBtpServer::gatt_access_handler,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &g_tx_val_handle,
    },
    {
        // Sentinel -- ble_gatts_count_cfg()/add_svcs() walk this array until
        // a zeroed entry (uuid == nullptr), the same convention the service
        // table below uses.
        0,
    },
};

const struct ble_gatt_svc_def kGattServices[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &kServiceUuid.u,
        .characteristics = kGattCharacteristics,
    },
    {
        0,
    },
};

}  // namespace

BleBtpServer* BleBtpServer::instance_ = nullptr;

BleBtpServer::~BleBtpServer() {
    stop();
}

bool BleBtpServer::start(const Callbacks& callbacks) noexcept {
    if (running_.load()) return true;
    if (callbacks.on_receive == nullptr) return false;
    // One physical radio, one ROBOT instance -- see the class comment.
    if (instance_ != nullptr && instance_ != this) return false;

    callbacks_ = callbacks;
    reset_send_queue();
    if (send_mutex_ == nullptr) {
        send_mutex_ = xSemaphoreCreateMutex();
        if (send_mutex_ == nullptr) return false;
    }

    instance_ = this;

    if (nimble_port_init() != 0) {
        ESP_LOGE(kTag, "nimble_port_init() failed");
        instance_ = nullptr;
        return false;
    }

    ble_hs_cfg.sync_cb = &BleBtpServer::on_sync;
    ble_hs_cfg.reset_cb = &BleBtpServer::on_reset;
    // No bonding/pairing (SMP) configured -- BTP's own AEAD (channel B, key
    // E) is this project's real confidentiality/authenticity boundary at
    // the application layer, exactly like TCP's direct session (T06's
    // decision: "criptografia obrigatoria" means BTP-level, not GATT-level).
    // Leaving the link itself unencrypted keeps this peripheral connectable
    // by any central without a pairing dialog first, matching TCP's own
    // "no TLS, BTP seals its own payload" posture.
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(kGattServices);
    if (rc != 0) {
        ESP_LOGE(kTag, "ble_gatts_count_cfg() failed: %d", rc);
        instance_ = nullptr;
        return false;
    }
    rc = ble_gatts_add_svcs(kGattServices);
    if (rc != 0) {
        ESP_LOGE(kTag, "ble_gatts_add_svcs() failed: %d", rc);
        instance_ = nullptr;
        return false;
    }

    ble_svc_gap_device_name_set(kDeviceName);

    // Logged once, at bring-up, so a real UUID transcription mistake (see
    // the array comments above) is visible in the very first boot log
    // rather than silently advertising the wrong service forever -- there is
    // no way to check these against real hardware from a native/host build.
    char uuid_str[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&kServiceUuid.u, uuid_str);
    ESP_LOGI(kTag, "BTP service UUID: %s", uuid_str);
    ble_uuid_to_str(&kRxUuid.u, uuid_str);
    ESP_LOGI(kTag, "RX characteristic UUID: %s", uuid_str);
    ble_uuid_to_str(&kTxUuid.u, uuid_str);
    ESP_LOGI(kTag, "TX characteristic UUID: %s", uuid_str);

    running_.store(true);
    nimble_port_freertos_init(&BleBtpServer::host_task);
    return true;
}

void BleBtpServer::stop() noexcept {
    if (!running_.load()) return;
    close_active_client();
    ble_gap_adv_stop();
    running_.store(false);
    nimble_port_stop();
    nimble_port_deinit();
    reset_send_queue();
    instance_ = nullptr;
}

void BleBtpServer::close_active_client() noexcept {
    const std::uint16_t handle = conn_handle_.load();
    if (handle != kInvalidHandle) {
        ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

void BleBtpServer::host_task(void* /*param*/) {
    // Blocks until nimble_port_stop() (called from stop() above) unblocks
    // it -- standard ESP-IDF NimBLE host-task shape (every bleprph-style
    // example uses this exact body).
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void BleBtpServer::on_sync() noexcept {
    // Public/random address inferred automatically -- this peripheral has
    // no bonding identity to preserve across reboots (see start()'s own
    // comment on why SMP is disabled entirely), so which kind it gets is
    // not load-bearing. The real, used-for-advertising type is re-derived
    // in start_advertising() itself; this call is only to make sure an
    // address exists at all before the first advertise attempt.
    std::uint8_t address_type = 0U;
    ble_hs_id_infer_auto(0, &address_type);
    if (instance_ != nullptr) instance_->start_advertising();
}

void BleBtpServer::on_reset(int reason) noexcept {
    ESP_LOGW(kTag, "NimBLE host reset, reason=%d", reason);
}

void BleBtpServer::start_advertising() noexcept {
    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = &kServiceUuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    fields.name = reinterpret_cast<const std::uint8_t*>(kDeviceName);
    fields.name_len = static_cast<std::uint8_t>(std::strlen(kDeviceName));
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(kTag, "ble_gap_adv_set_fields() failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params{};
    // T04's exclusivity decision: this is a peripheral any central may
    // connect to (UNDIRECTED) and any scanner may discover (GENERAL) --
    // the actual "one session at a time" enforcement is start_advertising()
    // only ever being called while conn_handle_ is kInvalidHandle (see the
    // GAP event handler's CONNECT/DISCONNECT cases), never a filter here.
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    std::uint8_t own_addr_type = 0U;
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(kTag, "ble_hs_id_infer_auto() failed: %d", rc);
        return;
    }

    rc = ble_gap_adv_start(own_addr_type, nullptr, BLE_HS_FOREVER, &adv_params,
                           &BleBtpServer::gap_event_handler, nullptr);
    if (rc != 0) {
        ESP_LOGE(kTag, "ble_gap_adv_start() failed: %d", rc);
    }
}

int BleBtpServer::gap_event_handler(struct ble_gap_event* event, void* /*arg*/) {
    BleBtpServer* self = instance_;
    if (self == nullptr) return 0;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                // Reached only while not already advertising-a-connection --
                // GAP itself already guarantees a second central cannot
                // connect while advertising is stopped (see the DISCONNECT
                // case, which is the only place advertising resumes). No
                // second, defensive check needed here.
                self->conn_handle_.store(event->connect.conn_handle);
                self->notifications_enabled_.store(false);
                self->reset_send_queue();
                if (self->callbacks_.on_connect != nullptr) {
                    self->callbacks_.on_connect(self->callbacks_.context);
                }
            } else {
                // Failed connection attempt -- still not advertising
                // (ble_gap_adv_start() is one-shot per successful/failed
                // attempt), so restart it.
                self->start_advertising();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            self->conn_handle_.store(kInvalidHandle);
            self->notifications_enabled_.store(false);
            self->att_mtu_.store(23U);
            self->reset_send_queue();
            if (self->callbacks_.on_disconnect != nullptr) {
                self->callbacks_.on_disconnect(self->callbacks_.context);
            }
            // Resume advertising -- this is the ENTIRE second-session
            // rejection mechanism (T04): nothing can discover/connect to
            // this service while this line has not run yet.
            self->start_advertising();
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == g_tx_val_handle) {
                self->notifications_enabled_.store(
                    event->subscribe.cur_notify != 0);
            }
            return 0;

        case BLE_GAP_EVENT_MTU:
            self->att_mtu_.store(static_cast<std::uint16_t>(event->mtu.value));
            return 0;

        case BLE_GAP_EVENT_NOTIFY_TX:
            // Fires once the previous notify() has actually left the
            // controller (success or failure) -- the backpressure signal
            // that lets the next queued chunk go out, mirroring
            // TraceView's own BleTransport waiting on
            // characteristicWritten() before its next chunk.
            self->notify_in_flight_.store(false);
            self->drain_send_queue();
            return 0;

        default:
            return 0;
    }
}

int BleBtpServer::gatt_access_handler(std::uint16_t conn_handle,
                                      std::uint16_t /*attr_handle*/,
                                      struct ble_gatt_access_ctxt* ctxt,
                                      void* /*arg*/) {
    BleBtpServer* self = instance_;
    if (self == nullptr) return BLE_ATT_ERR_UNLIKELY;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        // TX has no BLE_GATT_CHR_F_READ/WRITE flag (notify-only), so this
        // handler is never actually invoked for it in practice -- reachable
        // here only in the theoretical case the stack ever routes a stray
        // access; refusing explicitly is safer than assuming.
        return BLE_ATT_ERR_UNLIKELY;
    }

    const std::uint16_t length = OS_MBUF_PKTLEN(ctxt->om);
    if (length == 0U || length > kMaxChunkSize) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    std::uint8_t buffer[kMaxChunkSize];
    std::uint16_t copied_len = 0U;
    if (ble_hs_mbuf_to_flat(ctxt->om, buffer, sizeof(buffer), &copied_len) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (self->callbacks_.on_receive != nullptr &&
        conn_handle == self->conn_handle_.load()) {
        self->callbacks_.on_receive(self->callbacks_.context, buffer, copied_len);
    }
    return 0;
}

bool BleBtpServer::send(const std::uint8_t* data, std::size_t size,
                        FramePriority priority) noexcept {
    if (data == nullptr || size == 0U || !has_client()) return false;
    if (send_mutex_ == nullptr) return false;

    xSemaphoreTake(send_mutex_, portMAX_DELAY);
    const bool admitted = tcp_btp_server::admit(priority, send_queue_count_,
                                                kSendQueueDepth,
                                                kTelemetryQueueCeiling);
    if (!admitted) {
        xSemaphoreGive(send_mutex_);
        return false;
    }

    std::uint8_t* copy = new (std::nothrow) std::uint8_t[size];
    if (copy == nullptr) {
        xSemaphoreGive(send_mutex_);
        return false;
    }
    std::memcpy(copy, data, size);

    const std::size_t slot =
        (send_queue_head_ + send_queue_count_) % kSendQueueDepth;
    send_queue_[slot] = QueuedFrame{copy, size, 0U};
    ++send_queue_count_;
    xSemaphoreGive(send_mutex_);

    drain_send_queue();
    return true;
}

void BleBtpServer::drain_send_queue() noexcept {
    if (!has_client()) return;
    if (notify_in_flight_.exchange(true)) {
        // Already waiting on BLE_GAP_EVENT_NOTIFY_TX for a previous chunk --
        // that event's own handler calls back into this function once it's
        // clear to send the next one.
        return;
    }

    xSemaphoreTake(send_mutex_, portMAX_DELAY);
    if (send_queue_count_ == 0U) {
        xSemaphoreGive(send_mutex_);
        notify_in_flight_.store(false);
        return;
    }

    QueuedFrame& frame = send_queue_[send_queue_head_];
    const std::uint16_t mtu = att_mtu_.load();
    // ATT_MTU minus the 3-octet notification header -- same floor as
    // TraceView's own BleTransport::mtuPayloadSize() for the same reason:
    // 23 is BLE's own default/minimum ATT_MTU before negotiation, not a
    // BTP-specific number.
    const std::size_t chunk_capacity =
        static_cast<std::size_t>(mtu > 3U ? mtu - 3U : 20U);
    const std::size_t remaining = frame.size - frame.offset;
    const std::size_t chunk_size =
        remaining < chunk_capacity ? remaining : chunk_capacity;
    const std::uint8_t* chunk_data = frame.data + frame.offset;

    struct os_mbuf* om = ble_hs_mbuf_from_flat(chunk_data, chunk_size);
    const bool frame_complete = (frame.offset + chunk_size) >= frame.size;
    if (frame_complete) {
        delete[] frame.data;
        frame.data = nullptr;
        send_queue_head_ = (send_queue_head_ + 1U) % kSendQueueDepth;
        --send_queue_count_;
    } else {
        frame.offset += chunk_size;
    }
    xSemaphoreGive(send_mutex_);

    if (om == nullptr) {
        notify_in_flight_.store(false);
        return;
    }
    const int rc = ble_gatts_notify_custom(conn_handle_.load(), g_tx_val_handle, om);
    if (rc != 0) {
        // ble_gatts_notify_custom() frees `om` itself on every path
        // (success or failure) -- nothing to release here. No
        // BLE_GAP_EVENT_NOTIFY_TX will follow a call that failed this
        // synchronously, so this path has to clear the in-flight latch
        // itself (the event handler is what does it for the success path).
        notify_in_flight_.store(false);
    }
}

void BleBtpServer::reset_send_queue() noexcept {
    if (send_mutex_ != nullptr) xSemaphoreTake(send_mutex_, portMAX_DELAY);
    for (std::size_t i = 0U; i < kSendQueueDepth; ++i) {
        delete[] send_queue_[i].data;
        send_queue_[i] = QueuedFrame{};
    }
    send_queue_head_ = 0U;
    send_queue_count_ = 0U;
    if (send_mutex_ != nullptr) xSemaphoreGive(send_mutex_);
    notify_in_flight_.store(false);
}
