#include "TcpBtpServer.h"

#include <cerrno>
#include <cstring>
#include <new>

#include <esp_log.h>
#include <esp_timer.h>
#include <lwip/inet.h>
#include <lwip/sockets.h>
// Only the shared task-stack size alias is needed from this header.
#include <Settings.h>

namespace {

constexpr char kTag[] = "TCP_BTP";
constexpr int kListenBacklog = 2;
// Also the worst-case wait for a frame queued by ANOTHER task (telemetry,
// terminal output, command results): select() only watches for writability
// when the queue was already non-empty as it went to sleep, so anything
// enqueued meanwhile waits for this timeout. At 200 ms that turned 50 Hz
// telemetry into 5 Hz bursts and the terminal visibly laggy.
constexpr int kPollTimeoutMs = 10;

std::uint32_t now_ms() {
    return static_cast<std::uint32_t>(esp_timer_get_time() / 1000LL);
}

}  // namespace

TcpBtpServer::~TcpBtpServer() {
    stop();
}

bool TcpBtpServer::start(std::uint16_t port, const Callbacks& callbacks) noexcept {
    if (running_.load() || callbacks.on_receive == nullptr || port == 0U) {
        return false;
    }

    const int listen_fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_fd < 0) {
        ESP_LOGE(kTag, "socket failed: %d", errno);
        return false;
    }

    int reuse = 1;
    (void)::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(port);
    if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 ||
        ::listen(listen_fd, kListenBacklog) < 0) {
        ESP_LOGE(kTag, "bind/listen failed: %d", errno);
        ::close(listen_fd);
        return false;
    }

    send_mutex_ = xSemaphoreCreateMutex();
    if (send_mutex_ == nullptr) {
        ::close(listen_fd);
        return false;
    }

    listen_fd_ = listen_fd;
    callbacks_ = callbacks;
    send_queue_head_ = 0U;
    send_queue_count_ = 0U;
    running_.store(true);
    // ESP-IDF takes stack depth in bytes. RX storage lives in the server;
    // reserve stack headroom for socket, protocol and crypto call chains.
    if (xTaskCreate(&TcpBtpServer::task_entry, "btp_tcp", M8KB, this, 4, &task_) != pdPASS) {
        running_.store(false);
        vSemaphoreDelete(send_mutex_);
        send_mutex_ = nullptr;
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    ESP_LOGI(kTag, "listening on TCP port %u", static_cast<unsigned>(port));
    return true;
}

void TcpBtpServer::stop() noexcept {
    if (!running_.exchange(false)) {
        return;
    }

    const int listen_fd = listen_fd_;
    listen_fd_ = -1;
    if (listen_fd >= 0) {
        ::shutdown(listen_fd, SHUT_RDWR);
        ::close(listen_fd);
    }
    close_client();
    close_pending();

    while (task_ != nullptr) {
        vTaskDelay(1);
    }
    reset_send_queue();
    if (send_mutex_ != nullptr) {
        vSemaphoreDelete(send_mutex_);
        send_mutex_ = nullptr;
    }
    callbacks_ = Callbacks{};
}

bool TcpBtpServer::send(const std::uint8_t* data, std::size_t size,
                        FramePriority priority) noexcept {
    if (data == nullptr || size == 0U || send_mutex_ == nullptr) {
        return false;
    }
    if (xSemaphoreTake(send_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
        return false;
    }

    bool queued = false;
    if (client_fd_.load() >= 0 &&
        tcp_btp_server::admit(priority, send_queue_count_, kSendQueueDepth,
                              kTelemetryQueueCeiling)) {
        std::uint8_t* copy = new (std::nothrow) std::uint8_t[size];
        if (copy != nullptr) {
            std::memcpy(copy, data, size);
            const std::size_t tail =
                (send_queue_head_ + send_queue_count_) % kSendQueueDepth;
            send_queue_[tail] = QueuedFrame{copy, size, 0U};
            ++send_queue_count_;
            queued = true;
        }
    }

    xSemaphoreGive(send_mutex_);
    return queued;
}

bool TcpBtpServer::send_pending(const std::uint8_t* data, std::size_t size) noexcept {
    if (data == nullptr || size == 0U) {
        return false;
    }
    const int pending_fd = pending_fd_.load();
    if (pending_fd < 0) {
        return false;
    }
    const int written = ::send(pending_fd, data, size, MSG_DONTWAIT);
    return written == static_cast<int>(size);
}

void TcpBtpServer::close_pending() noexcept {
    // pending_fd_ is only ever touched here, in run()'s accept path and in
    // stop() -- no send_queue_/send_mutex_ involvement, unlike the active
    // client, so a plain atomic exchange is enough (matches close_client()'s
    // own idempotent-on-already-closed shape).
    const int pending_fd = pending_fd_.exchange(-1);
    if (pending_fd >= 0) {
        ::shutdown(pending_fd, SHUT_RDWR);
        ::close(pending_fd);
    }
}

void TcpBtpServer::task_entry(void* context) noexcept {
    static_cast<TcpBtpServer*>(context)->run();
    vTaskDelete(nullptr);
}

void TcpBtpServer::run() noexcept {
    while (running_.load()) {
        fd_set read_set;
        fd_set write_set;
        FD_ZERO(&read_set);
        FD_ZERO(&write_set);
        const int listen_fd = listen_fd_;
        if (listen_fd < 0) {
            break;
        }
        FD_SET(listen_fd, &read_set);
        int max_fd = listen_fd;

        const int client_fd = client_fd_.load();
        if (client_fd >= 0) {
            FD_SET(client_fd, &read_set);
            if (send_queue_count_ != 0U) {
                FD_SET(client_fd, &write_set);
            }
            max_fd = client_fd > max_fd ? client_fd : max_fd;
        }

        const int pending_fd = pending_fd_.load();
        if (pending_fd >= 0) {
            FD_SET(pending_fd, &read_set);
            max_fd = pending_fd > max_fd ? pending_fd : max_fd;
        }

        timeval timeout{};
        timeout.tv_sec = kPollTimeoutMs / 1000;
        timeout.tv_usec = (kPollTimeoutMs % 1000) * 1000;
        const int ready =
            ::select(max_fd + 1, &read_set, &write_set, nullptr, &timeout);

        // Pending-connection HELLO deadline: enforced here regardless of
        // whether select() returned anything, so a peer that never sends a
        // byte still gets closed on time.
        if (pending_fd_.load() >= 0 &&
            (now_ms() - pending_started_ms_) >= kPendingHelloDeadlineMs) {
            close_pending();
        }

        if (ready <= 0) {
            continue;
        }

        if (FD_ISSET(listen_fd, &read_set)) {
            const int accepted = ::accept(listen_fd, nullptr, nullptr);
            if (accepted >= 0) {
                int expected = -1;
                if (client_fd_.compare_exchange_strong(expected, accepted)) {
                    // First/only session: now the active client. Tell the
                    // composition layer before any bytes are read from it --
                    // see ConnectCallback's own comment.
                    if (callbacks_.on_connect != nullptr) {
                        callbacks_.on_connect(callbacks_.context);
                    }
                } else {
                    // A control session is already active. Session-and-
                    // terminal.md section 3.4: the accept() itself is not
                    // refused, but this connection gets exactly one chance
                    // to reach HELLO before the composition layer (via
                    // on_pending_receive) or this class's own
                    // kPendingHelloDeadlineMs closes it. A THIRD
                    // simultaneous attempt, while one is already pending,
                    // is dropped outright -- one pending slot is enough to
                    // honor the BUSY contract without unbounded fan-out.
                    int pending_expected = -1;
                    if (pending_fd_.compare_exchange_strong(pending_expected,
                                                            accepted)) {
                        pending_started_ms_ = now_ms();
                    } else {
                        ::shutdown(accepted, SHUT_RDWR);
                        ::close(accepted);
                    }
                }
            }
        }

        const int active_fd = client_fd_.load();
        if (active_fd >= 0 && FD_ISSET(active_fd, &read_set)) {
            const int received = ::recv(active_fd, receive_buffer_, sizeof(receive_buffer_), 0);
            if (received <= 0) {
                if (received < 0) {
                    ESP_LOGW(kTag, "recv() failed (errno %d), closing the client", errno);
                }
                close_client();
            } else if (callbacks_.on_receive != nullptr) {
                callbacks_.on_receive(callbacks_.context, receive_buffer_,
                                      static_cast<std::size_t>(received));
            }
        }

        if (client_fd_.load() >= 0 && FD_ISSET(client_fd_.load(), &write_set)) {
            drain_send_queue();
        }

        const int watched_pending = pending_fd_.load();
        if (watched_pending >= 0 && FD_ISSET(watched_pending, &read_set)) {
            const int received =
                ::recv(watched_pending, pending_receive_buffer_, sizeof(pending_receive_buffer_), 0);
            if (received <= 0) {
                close_pending();
            } else {
                if (callbacks_.on_pending_receive != nullptr) {
                    callbacks_.on_pending_receive(
                        callbacks_.context, pending_receive_buffer_,
                        static_cast<std::size_t>(received));
                }
                // A pending peer that keeps sending without ever completing
                // a recognizable HELLO datagram (or that overruns this
                // class's own small buffer across repeated reads) is not
                // this class's problem to keep parsing -- the composition
                // layer already saw every byte via on_pending_receive();
                // this class's own kPendingHelloDeadlineMs is the backstop
                // that eventually closes it either way.
            }
        }
    }

    close_client();
    close_pending();
    task_ = nullptr;
}

void TcpBtpServer::drain_send_queue() noexcept {
    if (send_mutex_ == nullptr ||
        xSemaphoreTake(send_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
        return;
    }

    const int client_fd = client_fd_.load();
    bool broken = false;
    while (client_fd >= 0 && send_queue_count_ != 0U) {
        QueuedFrame& front = send_queue_[send_queue_head_];
        const int written =
            ::send(client_fd, front.data + front.offset,
                  front.size - front.offset, MSG_DONTWAIT);
        if (written > 0) {
            front.offset += static_cast<std::size_t>(written);
            if (front.offset < front.size) {
                // Partial write: keep this frame at the head and stop for
                // now, never interleaving another frame's bytes ahead of
                // the rest of this one (fragmentation-and-transports.md
                // section 9.7: a frame in transit is never truncated or
                // reordered against itself).
                break;
            }
            delete[] front.data;
            front.data = nullptr;
            send_queue_head_ = (send_queue_head_ + 1U) % kSendQueueDepth;
            --send_queue_count_;
            continue;
        }
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK ||
                            errno == ENOMEM || errno == ENOBUFS)) {
            // Try again next pass. ENOMEM/ENOBUFS is lwIP out of pbufs /
            // send-buffer space -- transient under a telemetry burst, and
            // treating it as fatal was closing a perfectly good session
            // ("remote host closed the connection" on the client).
            break;
        }
        ESP_LOGW(kTag, "send() failed (errno %d), closing the client", errno);
        // A real write error: the connection is gone. Note it and stop --
        // close_client() (which also empties the queue) runs after this
        // mutex is released below; it takes the same mutex internally and
        // this one is not recursive.
        broken = true;
        break;
    }

    xSemaphoreGive(send_mutex_);
    if (broken) {
        close_client();
    }
}

void TcpBtpServer::reset_send_queue() noexcept {
    for (std::size_t i = 0U; i < send_queue_count_; ++i) {
        const std::size_t index = (send_queue_head_ + i) % kSendQueueDepth;
        delete[] send_queue_[index].data;
        send_queue_[index].data = nullptr;
    }
    send_queue_head_ = 0U;
    send_queue_count_ = 0U;
}

void TcpBtpServer::close_client() noexcept {
    const int client_fd = client_fd_.exchange(-1);
    if (client_fd >= 0) {
        ::shutdown(client_fd, SHUT_RDWR);
        ::close(client_fd);
        if (send_mutex_ != nullptr &&
            xSemaphoreTake(send_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            reset_send_queue();
            xSemaphoreGive(send_mutex_);
        } else {
            reset_send_queue();
        }
        if (callbacks_.on_disconnect != nullptr) {
            callbacks_.on_disconnect(callbacks_.context);
        }
    }
}
