#ifndef TCP_BTP_SERVER_H
#define TCP_BTP_SERVER_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// See TcpSendAdmission.h's own comment for why this portable, priority-
// admission helper lives in lib/BtpTransport/ rather than here (T23).
#include <TcpSendAdmission.h>

// Pure socket transport for a direct BTP control session over TCP
// (BTP/docs/fragmentation-and-transports.md section 9,
// BTP/docs/session-and-terminal.md sections 2.4/3.4). Deliberately BTP-
// unaware -- no COBS, no btp::Node, no Session -- so it stays native-
// testable-free/ESP-IDF-only on its own terms and the actual protocol
// composition (see utils/BallyRobot/BallyRobot.cpp) stays in one place.
//
// Accepts exactly one ACTIVE client (the one control session this robot
// serves) plus, transiently, one PENDING second connection attempt: a TCP
// server accepts the connection at the socket level even when a session is
// already active (session-and-terminal.md section 3.4: "the server does not
// refuse the accept()") and only rejects it once it reaches HELLO, with an
// explicit HELLO_RESULT status=BUSY -- which requires decoding a few bytes
// of BTP, the composition layer's job (btp_command::TcpBusyResponder in
// BtpTransport.h). This class only carries that second connection's bytes up
// to the composition layer and, on its say-so (or its own 2000 ms deadline,
// matching the HELLO deadline session-and-terminal.md section 3.4 already
// defines), closes it.
class TcpBtpServer {
public:
    using ReceiveCallback = void (*)(void* context, const std::uint8_t* data,
                                     std::size_t size) noexcept;
    // A new ACTIVE client was just accepted (there was no session already
    // active -- see the class comment on PENDING). Fires before any bytes
    // are read from it, so the composition layer can construct/arm its
    // per-connection BTP session (btp::Node::enable_session()/arm_session())
    // right when the link comes up, matching session-and-terminal.md section
    // 3.4 ("arm ... once the link is up") instead of waiting for the first
    // byte to trigger it lazily.
    using ConnectCallback = void (*)(void* context) noexcept;
    // Bytes read off the PENDING (second, not-yet-rejected) connection --
    // see the class comment. Never fires for the active client.
    using PendingCallback = void (*)(void* context, const std::uint8_t* data,
                                     std::size_t size) noexcept;
    // The active client disconnected (peer closed, a recv()/send() error, or
    // stop() tearing the whole server down) -- the composition layer's cue
    // to reset whatever per-session BTP state (a btp::Node instance,
    // subscriptions, ...) must not leak into the next connection
    // (session-and-terminal.md section 3.4, "does not resume or inherit any
    // state"). Fires at most once per accepted active client.
    using DisconnectCallback = void (*)(void* context) noexcept;

    static constexpr std::uint16_t kDefaultPort = 44300U;

    // BTP/docs/fragmentation-and-transports.md section 9.7: 16 frames
    // queued per connection on the bally_OS side, at most kTcpMaxFrameSize
    // (8192 octets) each -- at most 128 KiB buffered. send() enqueues here
    // rather than writing inline so a BTP-processing caller (the task that
    // drives btp::Node::receive()/tick()) never blocks on a slow TCP peer;
    // this class's own task drains the queue non-blockingly.
    static constexpr std::size_t kSendQueueDepth = 16U;

    using FramePriority = tcp_btp_server::FramePriority;

    // T23: reserve the top quarter of the queue for Normal-priority frames
    // (COMMAND_RESULT, TERMINAL_OUT, catalog/session replies) -- a telemetry
    // send() is refused once the queue already holds this many frames, well
    // before it is completely full. See TcpSendAdmission.h's own comment for
    // why this specific shape (a reserved margin, not a second queue) is
    // proportional to a single 16-deep FIFO. 12 == 75% of kSendQueueDepth,
    // matching the T23 task notes' own suggested threshold.
    static constexpr std::size_t kTelemetryQueueCeiling =
        (kSendQueueDepth * 3U) / 4U;

    // BTP/docs/session-and-terminal.md section 3.4: "If no valid HELLO
    // arrives within 2000 ms the server closes the socket without a reply."
    // Applied here to the PENDING connection only -- the active connection's
    // own HELLO deadline is enforced by its btp::Session (enable_session()'s
    // hello_deadline_ms), not by this class.
    static constexpr std::uint32_t kPendingHelloDeadlineMs = 2000U;

    TcpBtpServer() noexcept = default;
    ~TcpBtpServer();

    TcpBtpServer(const TcpBtpServer&) = delete;
    TcpBtpServer& operator=(const TcpBtpServer&) = delete;

    struct Callbacks {
        ReceiveCallback on_receive = nullptr;
        ConnectCallback on_connect = nullptr;
        PendingCallback on_pending_receive = nullptr;
        DisconnectCallback on_disconnect = nullptr;
        void* context = nullptr;
    };

    // `callbacks.on_receive` is required (start() fails without one); the
    // other two are optional (a null one is simply never invoked -- a
    // pending second connection is then only ever closed by this class's
    // own kPendingHelloDeadlineMs timeout, silently, and a disconnect is
    // never reported to the caller).
    bool start(std::uint16_t port, const Callbacks& callbacks) noexcept;
    void stop() noexcept;

    // Enqueues one complete BTP frame (already COBS-encoded and delimited,
    // or whatever unit the caller wants sent atomically) for the ACTIVE
    // client. Never blocks and never truncates: on success the whole frame
    // is either written immediately or copied into the queue for this
    // class's own task to drain; on failure (no active client, size above
    // kTcpMaxFrameSize-shaped limits the caller already enforces, the queue
    // already holding kSendQueueDepth frames, a Telemetry-priority frame
    // finding the queue at/above kTelemetryQueueCeiling -- see
    // TcpSendAdmission.h -- or the copy failing) nothing is queued and the
    // caller must retain/retry at the BTP queue boundary. `priority`
    // defaults to Normal: every existing caller (RobotTcpLink::send(),
    // COMMAND_RESULT/TERMINAL_OUT via protocol_tcp_) keeps today's behavior
    // unchanged; only TelemetryPublisher's TCP target passes Telemetry.
    bool send(const std::uint8_t* data, std::size_t size,
             FramePriority priority = FramePriority::Normal) noexcept;

    // Best-effort, single attempt, for the PENDING connection only (the
    // rejection reply is small -- see TcpBusyResponder -- and one-shot: on
    // congestion the caller still gets its exclusivity guarantee from the
    // socket being closed right after, even if the diagnostic BUSY payload
    // itself does not make it across). Not queued, unlike send().
    bool send_pending(const std::uint8_t* data, std::size_t size) noexcept;
    // Closes the pending connection immediately (the composition layer just
    // sent its BUSY reply, or has given up on it). Safe to call with no
    // pending connection.
    void close_pending() noexcept;

    bool running() const noexcept { return running_.load(); }
    bool has_client() const noexcept { return client_fd_.load() >= 0; }
    bool has_pending() const noexcept { return pending_fd_.load() >= 0; }

    // Drops the active client without stopping the whole server (a new one
    // may connect right after). For the composition layer's own use when it
    // cannot go on serving a just-accepted connection (e.g. its per-
    // connection btp::Node failed to initialize) -- fires on_disconnect()
    // like any other disconnect, so cleanup runs the same way.
    void close_active_client() noexcept { close_client(); }

private:
    static constexpr std::size_t kReceiveBufferSize = 4096U;
    // The second connection only needs enough bytes to recognize HELLO.
    static constexpr std::size_t kPendingReceiveBufferSize = 512U;

    // Owned by the server for its entire lifetime, accessed only by run().
    // Receive callbacks borrow these bytes synchronously and must not retain
    // them. Keeping these 4608 bytes off the task stack leaves room for
    // select(), the BTP callbacks and their nested calls.
    std::uint8_t receive_buffer_[kReceiveBufferSize]{};
    std::uint8_t pending_receive_buffer_[kPendingReceiveBufferSize]{};

    static void task_entry(void* context) noexcept;
    void run() noexcept;
    void close_client() noexcept;
    void drain_send_queue() noexcept;
    void reset_send_queue() noexcept;

    struct QueuedFrame {
        std::uint8_t* data = nullptr;
        std::size_t size = 0U;
        std::size_t offset = 0U;
    };

    int listen_fd_ = -1;
    std::atomic<int> client_fd_{-1};
    std::atomic<int> pending_fd_{-1};
    std::uint32_t pending_started_ms_ = 0U;
    std::atomic<bool> running_{false};
    TaskHandle_t task_ = nullptr;
    Callbacks callbacks_{};
    SemaphoreHandle_t send_mutex_ = nullptr;

    // Bounded FIFO of frames still owed to the active client -- see
    // kSendQueueDepth. Guarded by send_mutex_ (already needed for send()'s
    // own exclusion with the drain task). Each entry owns a heap allocation
    // freed once fully written or the connection closes.
    QueuedFrame send_queue_[kSendQueueDepth];
    std::size_t send_queue_head_ = 0U;
    std::size_t send_queue_count_ = 0U;
};

#endif
