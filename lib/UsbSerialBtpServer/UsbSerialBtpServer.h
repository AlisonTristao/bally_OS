#ifndef USB_SERIAL_BTP_SERVER_H
#define USB_SERIAL_BTP_SERVER_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Same priority-admission helper TcpBtpServer/BleBtpServer use (T23).
#include <TcpSendAdmission.h>

// BTP over the native USB CDC-ACM interface (comm_mode==3, "serial"): the
// same direct control session TCP and BLE carry, with the same
// 0x00 || COBS(frame) || 0x00 framing (TraceView's BtpSession decodes a
// serial port with CobsStream as well). Deliberately BTP-unaware in the same
// sense as TcpBtpServer/BleBtpServer -- no COBS, no btp::Node; ROBOT owns
// both on this link too (CobsStreamDecoder / sendCobsFramed()).
//
// Does NOT install the USB device itself: USBMassStorage owns the one
// TinyUSB driver (CDC + MSC descriptor) and ROBOT calls its
// install_persistent_usb_device() before start(). The CDC RX callback -- and
// therefore on_receive/on_connect/on_disconnect -- runs on that TinyUSB
// task.
//
// A "connection" is the host asserting DTR (TraceView does on open and drops
// it on close; see its SerialTransport's DTR contract). Bytes arriving while
// DTR is low still open one, for a host that never touches DTR. There is
// only one CDC port and one host, so no second-session rejection exists here
// -- a fresh HELLO on a live link is handled by ROBOT (isStaleHello()).
class UsbSerialBtpServer {
public:
    using ReceiveCallback = void (*)(void* context, const std::uint8_t* data,
                                     std::size_t size) noexcept;
    using ConnectCallback = void (*)(void* context) noexcept;
    using DisconnectCallback = void (*)(void* context) noexcept;

    using FramePriority = tcp_btp_server::FramePriority;

    // Frames waiting for room in TinyUSB's own CDC TX FIFO
    // (CONFIG_TINYUSB_CDC_TX_BUFSIZE). Same depth and same Normal-only top
    // quarter as BleBtpServer's queue.
    static constexpr std::size_t kSendQueueDepth = 16U;
    static constexpr std::size_t kTelemetryQueueCeiling =
        (kSendQueueDepth * 3U) / 4U;

    struct Callbacks {
        ReceiveCallback on_receive = nullptr;
        ConnectCallback on_connect = nullptr;
        DisconnectCallback on_disconnect = nullptr;
        void* context = nullptr;
    };

    UsbSerialBtpServer() noexcept = default;
    ~UsbSerialBtpServer();

    UsbSerialBtpServer(const UsbSerialBtpServer&) = delete;
    UsbSerialBtpServer& operator=(const UsbSerialBtpServer&) = delete;

    // Initializes the CDC-ACM interface on the already-installed USB device
    // and registers its RX/line-state callbacks. `callbacks.on_receive` is
    // required. Idempotent.
    bool start(const Callbacks& callbacks) noexcept;

    // Enqueues one complete, already COBS-encoded frame for the host. Never
    // blocks; false when no host is connected, the queue refuses it
    // (admission, see kTelemetryQueueCeiling) or the copy cannot be
    // allocated.
    bool send(const std::uint8_t* data, std::size_t size,
              FramePriority priority = FramePriority::Normal) noexcept;

    bool has_client() const noexcept { return connected_.load(); }

    // TinyUSB/esp_tinyusb C callbacks (file-scope trampolines in the .cpp).
    void handle_line_state(bool dtr) noexcept;
    void handle_rx() noexcept;
    void drain_send_queue() noexcept;

private:
    struct QueuedFrame {
        std::uint8_t* data = nullptr;
        std::size_t size = 0U;
        std::size_t offset = 0U;
    };

    void open_session() noexcept;
    void close_session() noexcept;
    void reset_send_queue() noexcept;

    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    Callbacks callbacks_{};
    SemaphoreHandle_t send_mutex_ = nullptr;

    // Guarded by send_mutex_. Each entry owns a heap copy, freed once fully
    // written into the CDC FIFO or when the session closes.
    QueuedFrame send_queue_[kSendQueueDepth];
    std::size_t send_queue_head_ = 0U;
    std::size_t send_queue_count_ = 0U;

    // One tinyusb_cdcacm_read() chunk; only touched on the TinyUSB task.
    std::uint8_t rx_buffer_[512];
};

#endif  // USB_SERIAL_BTP_SERVER_H
