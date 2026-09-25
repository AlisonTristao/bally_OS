#include "StatesManager.h"

#include <SetupCommTrigger.h>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
// Polling granularity for the btn0-hold check below. Short enough that the
// up-to-1500ms wait feels responsive to an immediate release, long enough
// not to hammer gpio_get_level()/the scheduler.
constexpr uint32_t kPollMs = 20U;
}  // namespace

stateName States::setup_function() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        robot_.logger.insert_log(logType::INFO, "Setup function called");
    #endif

    // T25b (TAREFAS_TCP_BLE_ANDROID.txt, ETAPA 3B): press btn0 (BOOT/GPIO0)
    // within kPressWindowMs of SETUP starting and hold it ~1.5s to enter
    // COMM_CONFIG instead of falling straight through to WAIT. Deliberately NOT during reset --
    // holding btn0 through reset drops the chip into the Espressif ROM
    // bootloader instead of running this firmware at all (see
    // ROBOT::init()'s own comment on cfg.btn0), so this check only ever
    // sees a press that happens after the application is already running.
    //
    // Reads the GPIO LEVEL directly (gpio_get_level), never the ISR-fed
    // Flags_in buttons object -- this runs on the "state_machine" task
    // (main.cpp: priority 10, pinned to APP_CPU_NUM/core 1), created in the
    // same start_freertos_tasks() call as "interrupts" (priority 0, pinned
    // to PRO_CPU_NUM/core 0), which is what actually installs the button
    // ISR handlers (ROBOT::initInterruptions()). Which of the two tasks
    // gets CPU time first is a genuine scheduler race (confirmed by reading
    // main.cpp, not assumed) -- polling the raw level sidesteps it entirely,
    // the same reasoning ROBOT::init() already uses to check btn1/btn2 held
    // at boot (boot_enter_ota/boot_enter_storage in BallyRobot.cpp).
    //
    // This blocks the state-machine task for up to ~3s (press window) plus
    // the ~1.5s hold. Acceptable here:
    // SETUP runs exactly once, before anything (WAIT/RUN/telemetry
    // subscribers) depends on this task being responsive.
    const SettingsData& cfg = robot_.settings.data();
    const gpio_num_t btn0 = static_cast<gpio_num_t>(cfg.btn0);

    #if defined(LOG_ALL) || defined(LOG_INFO)
        robot_.logger.insert_logf(
            logType::INFO,
            "Setup: hold BOOT (btn0) within %ums for %ums to enter COMM_CONFIG",
            static_cast<unsigned>(setup_comm_trigger::kPressWindowMs),
            static_cast<unsigned>(setup_comm_trigger::kHoldThresholdMs));
    #endif

    uint32_t elapsed_ms = 0;
    uint32_t held_ms = 0;
    bool was_held = false;
    while (true) {
        const bool held = gpio_get_level(btn0) == 0;  // pull-up: LOW = pressed
        const setup_comm_trigger::Decision decision =
            setup_comm_trigger::decide(held, held_ms, elapsed_ms, was_held);

        if (decision == setup_comm_trigger::Decision::GoToWait) {
            return go_to(SETUP, WAIT);
        }
        if (decision == setup_comm_trigger::Decision::GoToCommConfig) {
            #if defined(LOG_ALL) || defined(LOG_INFO)
                robot_.logger.insert_log(
                    logType::INFO,
                    "Setup: btn0 held through the threshold, entering COMM_CONFIG");
            #endif
            return go_to(SETUP, COMM_CONFIG);
        }

        // KeepWaiting: inside the press window, or held below the threshold.
        vTaskDelay(pdMS_TO_TICKS(kPollMs));
        elapsed_ms += kPollMs;
        if (held) {
            held_ms += kPollMs;
            was_held = true;
        }
    }
}