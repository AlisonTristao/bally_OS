#ifndef SETUP_COMM_TRIGGER_H
#define SETUP_COMM_TRIGGER_H

// autor: Alison Tristao

#include <cstdint>

// T25b (TAREFAS_TCP_BLE_ANDROID.txt, ETAPA 3B): the pure decision behind
// SETUP's boot-time COMM_CONFIG trigger, pulled out of
// src/robot/01_Setup.cpp so it is testable under env:native without
// FreeRTOS/GPIO -- same "header-only, no ESP-IDF dependency" shape as
// lib/BtpTransport/TcpSendAdmission.h (T23).
//
// SETUP itself still owns the actual polling loop (vTaskDelay + a real
// gpio_get_level(cfg.btn0) read, see 01_Setup.cpp) -- this header only
// answers "given the button's CURRENT level and how long it has been polled
// since SETUP started checking, what should happen next", which needs no
// hardware access to test.
namespace setup_comm_trigger {

// Hold duration required to enter COMM_CONFIG instead of falling straight
// through to WAIT. Chosen by the user (2026-09-21): long enough that the
// normal, momentary release of a strapping-adjacent button during a routine
// boot never trips it, short enough not to feel like a hang when deliberate.
inline constexpr std::uint32_t kHoldThresholdMs = 1500U;

enum class Decision : std::uint8_t {
    KeepWaiting,     // still held, threshold not reached yet -- poll again
    GoToWait,        // not held (never was, or released before the threshold)
    GoToCommConfig,  // held continuously for >= kHoldThresholdMs
};

/**
 * @brief Pure decision for one poll of SETUP's btn0-hold check.
 * @param held Current level of btn0, already read by the caller
 * (gpio_get_level(cfg.btn0) == 0, pull-up wiring -- see 01_Setup.cpp).
 * @param elapsed_ms Milliseconds since SETUP started polling (0 on the very
 * first call).
 */
inline Decision decide(bool held, std::uint32_t elapsed_ms) {
    if (!held) return Decision::GoToWait;
    if (elapsed_ms >= kHoldThresholdMs) return Decision::GoToCommConfig;
    return Decision::KeepWaiting;
}

}  // namespace setup_comm_trigger

#endif  // SETUP_COMM_TRIGGER_H
