#include <States.h>

stateName States::setup_function() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        robot_.logger.insert_log(logType::INFO, "Setup function called");
    #endif

    // setup completed, go to wait state
    return go_to(SETUP, WAIT);
}