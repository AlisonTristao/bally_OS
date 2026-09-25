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

// How long SETUP waits for btn0 to START being held. btn0 is GPIO0 (BOOT
// strap): it cannot already be held when the application starts, because
// holding it through reset selects the ROM bootloader. Without this window
// the very first poll saw "not held" and went straight to WAIT, so the
// trigger was unreachable in practice -- the press has to come after reset.
inline constexpr std::uint32_t kPressWindowMs = 3000U;

enum class Decision : std::uint8_t {
    KeepWaiting,     // inside the press window, or held but below threshold
    GoToWait,        // window over and not held, or released before threshold
    GoToCommConfig,  // held continuously for >= kHoldThresholdMs
};

/**
 * @brief Pure decision for one poll of SETUP's btn0-hold check.
 * @param held Current level of btn0, already read by the caller
 * (gpio_get_level(cfg.btn0) == 0, pull-up wiring -- see 01_Setup.cpp).
 * @param held_ms How long btn0 has been held continuously (0 if not held).
 * @param elapsed_ms Milliseconds since SETUP started polling (0 on the very
 * first call).
 * @param was_held Whether btn0 was held at any earlier poll. A release after
 * a press ends the check (GoToWait) instead of re-arming the window.
 */
inline Decision decide(bool held, std::uint32_t held_ms,
                       std::uint32_t elapsed_ms, bool was_held) {
    if (held) {
        return held_ms >= kHoldThresholdMs ? Decision::GoToCommConfig
                                           : Decision::KeepWaiting;
    }
    if (was_held) return Decision::GoToWait;
    return elapsed_ms < kPressWindowMs ? Decision::KeepWaiting
                                       : Decision::GoToWait;
}

}  // namespace setup_comm_trigger

#endif  // SETUP_COMM_TRIGGER_H
