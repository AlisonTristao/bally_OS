#ifndef BLE_BTP_SERVER_H
#define BLE_BTP_SERVER_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Same portable, priority-admission helper TcpBtpServer uses (T23) -- see
// its own comment for why a reserved-headroom scheme on one small FIFO is
// the proportional response here too, rather than replicating TxScheduler's
// six fixed-priority queues for a queue this shallow.
#include <TcpSendAdmission.h>

// NimBLE peripheral for the BTP v1 BLE service (BTP/docs/fragmentation-and-
// transports.md section 8, TAREFAS_TCP_BLE_ANDROID.txt T33/T34). Deliberately
// BTP-unaware in the same sense TcpBtpServer is -- no COBS, no btp::Node --
// but unlike TcpBtpServer this class DOES own one piece of framing the
// composition layer would otherwise have to duplicate: GATT's own MTU
// ceiling. A COBS-wrapped BTP frame handed to send() is usually larger than
// one ATT write/notification can carry, so send() fragments it into
// <= (negotiated MTU - 3) chunks -- the exact mirror of what TraceView's own
// BleTransport does for its writes (see that class's own comment for why
// this is symmetric and correct: both ends just forward a raw byte stream).
// The COBS layer itself is NOT btp::Node's -- ROBOT owns it on both links
// (CobsStreamDecoder on receive, sendCobsFramed() on send; BtpTransport.h).
//
// Exclusivity is enforced at the GAP level (T04's decision), NOT by
// accepting-then-rejecting the way TcpBtpServer's pending-connection dance
// does: this peripheral simply stops advertising the instant a central
// connects and resumes once it disconnects, so a second central can never
// even discover the service while one control session is active. There is
// therefore no PENDING-connection concept here at all, unlike TcpBtpServer.
class BleBtpServer {
public:
    using ReceiveCallback = void (*)(void* context, const std::uint8_t* data,
                                     std::size_t size) noexcept;
    // A central just connected (there was no active session, since a second
    // one can never reach this point -- see the class comment) -- fires
    // before any bytes are read from it, mirroring TcpBtpServer::
    // ConnectCallback so the composition layer can arm its per-connection
    // btp::Node right when the link comes up.
    using ConnectCallback = void (*)(void* context) noexcept;
    // The active central disconnected (it closed the link, a radio error, or
    // stop() tearing the whole server down). Fires at most once per
    // accepted connection.
    using DisconnectCallback = void (*)(void* context) noexcept;

    // BTP/docs/fragmentation-and-transports.md section 8.2: 8 frames queued
    // on the bally_OS side, at most kBleMaxFrameSize (512 octets, once
    // btp::kBleTransport exists -- see BallyRobot.h's own note on the
    // kSerialTransport stand-in used meanwhile) each.
    //
    // Raised to 16: drain_send_queue() now packs several queued frames into
    // each notification, so the queue drains faster, but a telemetry burst
    // right as a command reply is produced still needs the headroom.
    static constexpr std::size_t kSendQueueDepth = 16U;

    using FramePriority = tcp_btp_server::FramePriority;

    // T23's same reservation, reused verbatim for BLE's queue: the top
    // quarter reserved for Normal-priority frames (COMMAND_RESULT,
    // TERMINAL_OUT, catalog/session replies), so a telemetry burst can never
    // crowd out a reply that has to leave right after it. 12 == 75% of 16.
    static constexpr std::size_t kTelemetryQueueCeiling =
        (kSendQueueDepth * 3U) / 4U;

    BleBtpServer() noexcept = default;
    ~BleBtpServer();

    BleBtpServer(const BleBtpServer&) = delete;
    BleBtpServer& operator=(const BleBtpServer&) = delete;

    struct Callbacks {
        ReceiveCallback on_receive = nullptr;
        ConnectCallback on_connect = nullptr;
        DisconnectCallback on_disconnect = nullptr;
        void* context = nullptr;
    };

    // Brings up the NimBLE host + controller (nimble_port_init()), registers
    // the BTP GATT service/RX/TX characteristics and starts advertising.
    // `callbacks.on_receive` is required (start() fails without one), the
    // other two are optional, same convention as TcpBtpServer::start().
    // Idempotent: a second call while already running is a no-op returning
    // true.
    bool start(const Callbacks& callbacks) noexcept;
    // Disconnects the active central (if any), stops advertising and tears
    // the whole NimBLE host down. Safe to call when not running.
    void stop() noexcept;

    // Enqueues one complete BTP frame (already COBS-encoded and delimited)
    // for the connected central. Never blocks: on success the frame is
    // copied into the queue for the NimBLE host task to drain as a sequence
    // of MTU-sized notifications; on failure (no connected central, the
    // queue already holding kSendQueueDepth frames, a Telemetry-priority
    // frame finding the queue at/above kTelemetryQueueCeiling, or the copy
    // failing) nothing is queued and the caller must retain/retry at the
    // BTP queue boundary, exactly like TcpBtpServer::send()'s own contract.
    bool send(const std::uint8_t* data, std::size_t size,
             FramePriority priority = FramePriority::Normal) noexcept;

    bool running() const noexcept { return running_.load(); }
    // True once a central is connected AND this peripheral's RX/TX are
    // usable from the central's point of view -- concretely, once the
    // central has subscribed to TX notifications, the same "RX/TX usable"
    // gate TraceView's own BleTransport applies before it lets HELLO go out
    // (T28's acceptance criterion, this class's own mirror of it). A central
    // connected but not yet subscribed is NOT "has a client" here: sending
    // a HELLO_RESULT (or anything else) toward it would silently vanish --
    // NimBLE accepts the notify call but the stack drops it with nothing
    // subscribed to hear it.
    bool has_client() const noexcept {
        return conn_handle_.load() != kInvalidHandle && notifications_enabled_.load();
    }

    // Drops the active central without stopping the whole server (a new one
    // may connect right after, once advertising resumes). Mirrors
    // TcpBtpServer::close_active_client() -- for the composition layer's own
    // use when it cannot go on serving a just-connected central (e.g. its
    // per-connection btp::Node failed to initialize).
    void close_active_client() noexcept;

    // NimBLE callbacks -- see BleBtpServer.cpp for the ble_gap_event_fn/
    // ble_gatt_access_fn signatures these wrap. Public (not private) only
    // because the file-scope GATT service table in BleBtpServer.cpp's own
    // anonymous namespace needs their address; nothing outside that
    // translation unit has a reason to call them directly.
    static int gap_event_handler(struct ble_gap_event* event, void* arg);
    static int gatt_access_handler(std::uint16_t conn_handle,
                                   std::uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt* ctxt,
                                   void* arg);

private:
    static constexpr std::uint16_t kInvalidHandle = 0xFFFFU; // BLE_HS_CONN_HANDLE_NONE

    struct QueuedFrame {
        std::uint8_t* data = nullptr;
        std::size_t size = 0U;
        std::size_t offset = 0U;
    };

    static void host_task(void* param);
    void start_advertising() noexcept;
    void drain_send_queue() noexcept;
    void reset_send_queue() noexcept;

    // Static + a single instance_ pointer, same "one physical radio, one
    // server" posture TcpBtpServer's own statics have (this project builds
    // exactly one ROBOT).
    static BleBtpServer* instance_;
    static void on_sync() noexcept;
    static void on_reset(int reason) noexcept;

    std::atomic<bool> running_{false};
    std::atomic<std::uint16_t> conn_handle_{kInvalidHandle};
    std::atomic<bool> notifications_enabled_{false};
    // ATT_MTU negotiated for the current connection (BLE_ATT_MTU_DFLT, 23,
    // until MTU exchange completes -- see BLE_GAP_EVENT_MTU in the .cpp).
    // Reset to the default on every disconnect.
    std::atomic<std::uint16_t> att_mtu_{23U};
    Callbacks callbacks_{};
    SemaphoreHandle_t send_mutex_ = nullptr;

    // Bounded FIFO of frames still owed to the connected central -- see
    // kSendQueueDepth. Guarded by send_mutex_. Each entry owns a heap
    // allocation freed once fully drained or the connection closes.
    QueuedFrame send_queue_[kSendQueueDepth];
    std::size_t send_queue_head_ = 0U;
    std::size_t send_queue_count_ = 0U;
    // Deferred-drain trigger: NimBLE's own host task calls back into
    // drain_send_queue() once a notification actually clears the stack's
    // internal buffer (BLE_GAP_EVENT_NOTIFY_TX), the same "next chunk only
    // after the previous one is acknowledged" backpressure TraceView's own
    // BleTransport applies on its side.
    std::atomic<bool> notify_in_flight_{false};
    // One notification's worth of bytes, assembled from as many queued
    // frames as fit (see drain_send_queue()). Only touched by whoever won
    // notify_in_flight_, and copied into an mbuf before that is released.
    std::uint8_t notify_scratch_[512];
};

#endif  // BLE_BTP_SERVER_H
