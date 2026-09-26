#include "UsbSerialBtpServer.h"

#include <cstring>
#include <new>

#include "esp_log.h"
#include "tinyusb_cdc_acm.h"
#include "tusb.h"

namespace {

const char* kTag = "UsbSerialBtp";

// esp_tinyusb's CDC callbacks are plain C function pointers with no user-arg
// slot, and TinyUSB's tud_cdc_tx_complete_cb() has none either -- one CDC
// port, one server, so one file-local pointer (same shape as bally_dongle's
// ConsoleCdc).
UsbSerialBtpServer* g_instance = nullptr;

void on_cdc_rx(int /*itf*/, cdcacm_event_t* /*event*/) {
    if (g_instance != nullptr) g_instance->handle_rx();
}

void on_cdc_line_state(int /*itf*/, cdcacm_event_t* event) {
    if (g_instance == nullptr || event == nullptr) return;
    g_instance->handle_line_state(event->line_state_changed_data.dtr);
}

}  // namespace

// TinyUSB's own weak hook (cdc_device.c): the previous IN transfer finished,
// so the CDC TX FIFO has room again -- keep draining the queue from here, on
// the TinyUSB task. Harmless in every other comm_mode: g_instance stays null
// when start() never ran.
extern "C" void tud_cdc_tx_complete_cb(uint8_t itf) {
    (void)itf;
    if (g_instance != nullptr) g_instance->drain_send_queue();
}

UsbSerialBtpServer::~UsbSerialBtpServer() {
    if (g_instance == this) g_instance = nullptr;
    reset_send_queue();
    if (send_mutex_ != nullptr) vSemaphoreDelete(send_mutex_);
}

bool UsbSerialBtpServer::start(const Callbacks& callbacks) noexcept {
    if (running_.load()) return true;
    if (callbacks.on_receive == nullptr) return false;

    if (send_mutex_ == nullptr) {
        send_mutex_ = xSemaphoreCreateMutex();
        if (send_mutex_ == nullptr) return false;
    }
    callbacks_ = callbacks;
    g_instance = this;

    tinyusb_config_cdcacm_t cfg = {};
    cfg.cdc_port = TINYUSB_CDC_ACM_0;
    cfg.callback_rx = &on_cdc_rx;
    cfg.callback_line_state_changed = &on_cdc_line_state;
    if (tinyusb_cdcacm_init(&cfg) != ESP_OK) {
        ESP_LOGE(kTag, "tinyusb_cdcacm_init() failed");
        g_instance = nullptr;
        return false;
    }

    running_.store(true);
    return true;
}

void UsbSerialBtpServer::handle_line_state(bool dtr) noexcept {
    if (dtr) {
        // A host (re)opening the port is a new client even if the previous
        // one never dropped DTR (a crashed app, a replugged cable): never let
        // it inherit the old session.
        if (connected_.load()) close_session();
        open_session();
    } else if (connected_.load()) {
        close_session();
    }
}

void UsbSerialBtpServer::handle_rx() noexcept {
    for (;;) {
        std::size_t got = 0U;
        if (tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, rx_buffer_, sizeof(rx_buffer_),
                                &got) != ESP_OK ||
            got == 0U) {
            return;
        }
        // Bytes before any DTR edge: a host that never asserts DTR.
        if (!connected_.load()) open_session();
        callbacks_.on_receive(callbacks_.context, rx_buffer_, got);
    }
}

void UsbSerialBtpServer::open_session() noexcept {
    reset_send_queue();
    connected_.store(true);
    if (callbacks_.on_connect != nullptr) callbacks_.on_connect(callbacks_.context);
}

void UsbSerialBtpServer::close_session() noexcept {
    connected_.store(false);
    reset_send_queue();
    // Whatever the old session left in TinyUSB's FIFO must not reach the
    // next host as the head of its byte stream.
    tud_cdc_n_write_clear(0);
    if (callbacks_.on_disconnect != nullptr) callbacks_.on_disconnect(callbacks_.context);
}

bool UsbSerialBtpServer::send(const std::uint8_t* data, std::size_t size,
                              FramePriority priority) noexcept {
    if (data == nullptr || size == 0U || !connected_.load()) return false;
    if (send_mutex_ == nullptr || !tud_ready()) return false;

    xSemaphoreTake(send_mutex_, portMAX_DELAY);
    if (!tcp_btp_server::admit(priority, send_queue_count_, kSendQueueDepth,
                               kTelemetryQueueCeiling)) {
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

void UsbSerialBtpServer::drain_send_queue() noexcept {
    if (send_mutex_ == nullptr || !connected_.load()) return;

    // Only this function ever writes into the CDC FIFO, always under
    // send_mutex_ and in queue order, so a frame may be split across
    // several passes (FIFO full) without ever interleaving with another.
    xSemaphoreTake(send_mutex_, portMAX_DELAY);
    bool wrote = false;
    while (send_queue_count_ > 0U) {
        QueuedFrame& frame = send_queue_[send_queue_head_];
        const std::size_t remaining = frame.size - frame.offset;
        const std::size_t written = tinyusb_cdcacm_write_queue(
            TINYUSB_CDC_ACM_0, frame.data + frame.offset, remaining);
        if (written == 0U) break;
        wrote = true;
        if (written < remaining) {
            frame.offset += written;
            break;
        }
        delete[] frame.data;
        frame = QueuedFrame{};
        send_queue_head_ = (send_queue_head_ + 1U) % kSendQueueDepth;
        --send_queue_count_;
    }
    xSemaphoreGive(send_mutex_);

    // Non-blocking: TinyUSB keeps transmitting on its own task, and
    // tud_cdc_tx_complete_cb() brings us back here for whatever is left.
    if (wrote) tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
}

void UsbSerialBtpServer::reset_send_queue() noexcept {
    if (send_mutex_ != nullptr) xSemaphoreTake(send_mutex_, portMAX_DELAY);
    for (std::size_t i = 0U; i < kSendQueueDepth; ++i) {
        delete[] send_queue_[i].data;
        send_queue_[i] = QueuedFrame{};
    }
    send_queue_head_ = 0U;
    send_queue_count_ = 0U;
    if (send_mutex_ != nullptr) xSemaphoreGive(send_mutex_);
}
