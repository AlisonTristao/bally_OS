#include "StatesManager.h"

stateName States::telemetry_function() {
    // One-shot on entry, same pattern as setup_function()/calibrate_function():
    // persist the retained PSRAM log to a new dated file on SD (the streamed
    // copy already goes out over ESP-NOW continuously, in every state, via
    // routine()'s flush_logs() — this is what makes the run's full log
    // survive range/packet loss), then self-transition back to WAIT.
    char filename[SDFileInfo::MAX_NAME_LENGTH] = {};
    const bool ok = robot_.logger.flush_to_sd(robot_.sd_card, false, filename, sizeof(filename));

    #if defined(LOG_ALL) || defined(LOG_INFO)
        if (ok) robot_.logger.insert_logf(logType::INFO, "Telemetry saved to %s", filename);
        else    robot_.logger.insert_log(logType::ERRO, "Telemetry function called: failed to save logs to SD");
    #endif

    return go_to(TELEMETRY, WAIT);
}
