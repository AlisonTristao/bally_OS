#include "StatesManager.h"
    
stateName States::debug_function() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        robot_.logger.insert_log(logType::INFO, (robot_.array_sensor.debug()).c_str());
    #endif

    // safety delay default outside RUN
    vTaskDelay(WDOG_TIMEOUT_TK);
    
    return DEBUG;
}