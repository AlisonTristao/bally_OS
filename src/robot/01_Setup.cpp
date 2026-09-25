#include "StatesManager.h"

stateName States::setup_function() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        robot_.logger.insert_log(logType::INFO, "Setup function called");
    #endif

    // Boot-time modes (OTA, USB storage, COMM_CONFIG) are picked in
    // ROBOT::init() from the buttons held through reset -- by the time SETUP
    // runs, a normal boot is the only thing left to do.
    return go_to(SETUP, WAIT);
}
