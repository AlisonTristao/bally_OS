#include "StatesManager.h"

stateName States::calibrate_function() {
    const SettingsData& cfg = robot_.settings.data();
    const bool calib = robot_.array_sensor->calibrate(cfg.samples, cfg.delay_sample);

    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        robot_.logger.insert_log(logType::INFO, ("Calibrate function called: " + std::string(!calib ? "failed" : "success")).c_str());
        robot_.logger.insert_log(logType::INFO, ("Values:\n\n" + robot_.array_sensor->calibrate_status()).c_str());
    #endif

    return go_to(CALIBRATE, WAIT);
}