#ifndef TCP_SEND_ADMISSION_H
#define TCP_SEND_ADMISSION_H

#include <cstddef>
#include <cstdint>

// T23 (TAREFAS_TCP_BLE_ANDROID.txt): TcpBtpServer::send_queue_ is a single
// 16-frame FIFO shared by every producer that reaches a direct-TCP session
// (COMMAND_RESULT, TERMINAL_OUT, catalog/session replies via RobotTcpLink,
// and TELEMETRY via TelemetryPublisher's TCP target -- see BallyRobot.cpp's
// onTcpConnect()). Unlike the ESP-NOW path (TxScheduler.h, 6 fixed-priority
// circular queues: CommandResult > Status > Terminal > CriticalLog >
// Telemetry > Debug), that FIFO does not distinguish frame classes: once
// full, ANY frame is refused equally, including a COMMAND_RESULT that needs
// to leave right after a burst of telemetry filled the queue.
//
// Replicating TxScheduler's 6 queues here would be over-engineering for a
// single 16-deep FIFO -- see the T23 task notes. Instead this is the
// smallest fix that restores the same "telemetry falls first" principle
// T05 already settled: telemetry is refused a bit before the queue is
// completely full, reserving a fixed margin exclusively for every other
// (higher-priority) frame class. Telemetry itself never retries a refused
// sample (TelemetryPublisher::flush() drops it and only counts
// send_failed_), so this margin can never be eaten into by a retry storm --
// it only ever has to absorb the very next COMMAND_RESULT/TERMINAL_OUT/
// catalog reply.
//
// Deliberately header-only and free of FreeRTOS/lwip so it is usable from
// the native unit test suite (test/btp_integration/test_main.cpp) the same
// way TcpBusyResponder already is, even though TcpBtpServer itself (the
// caller) is ESP-IDF-only. Lives in lib/BtpTransport/ rather than
// lib/TcpBtpServer/ for the same reason TcpBusyResponder does (see that
// class's own comment in BtpTransport.h): PlatformIO's library dependency
// finder builds every .cpp in a library's folder once any of its headers is
// included, and lib/TcpBtpServer/TcpBtpServer.cpp itself pulls in
// freertos/lwip -- including it from the native-only lib/BtpTransport/ (a
// library already proven ESP-IDF-free, and already a dependency of the
// native suite) avoids dragging that in.
namespace tcp_btp_server {

enum class FramePriority : std::uint8_t {
    // Everything that is not telemetry: COMMAND_RESULT, TERMINAL_OUT, and
    // btp::Node's own automatic replies (HELLO_RESULT, MANIFEST_DATA,
    // SUBSCRIBE_RESULT/UNSUBSCRIBE_RESULT, SESSION_CLOSE) sent via
    // RobotTcpLink::send(). Only ever refused once the queue is genuinely
    // full (no reserved margin below it -- there is nothing lower-priority
    // than Telemetry to protect Normal frames from).
    Normal = 0,
    // TELEMETRY samples (TelemetryPublisher's TCP target). Refused early --
    // see admit() -- so a burst never crowds out a Normal frame that arrives
    // moments later.
    Telemetry = 1,
};

// Whether one more frame of `priority` may be queued, given the queue
// currently holds `queue_count` of at most `queue_depth`, with the top
// `queue_depth - telemetry_ceiling` slots reserved for Normal frames only.
//
// `telemetry_ceiling` <= `queue_depth`; passing telemetry_ceiling ==
// queue_depth disables the reservation entirely (Telemetry then behaves
// exactly like Normal, the pre-T23 behavior).
inline bool admit(FramePriority priority, std::size_t queue_count,
                   std::size_t queue_depth,
                   std::size_t telemetry_ceiling) noexcept {
    if (queue_count >= queue_depth) {
        // Hard cap: nothing is admitted once the queue is truly full,
        // regardless of priority -- matches send()'s pre-T23 behavior for
        // every class other than telemetry, and is what keeps the worst-case
        // memory bound (queue_depth * kTcpMaxFrameSize) unchanged by this
        // reservation.
        return false;
    }
    if (priority == FramePriority::Telemetry && queue_count >= telemetry_ceiling) {
        return false;
    }
    return true;
}

}  // namespace tcp_btp_server

#endif  // TCP_SEND_ADMISSION_H
