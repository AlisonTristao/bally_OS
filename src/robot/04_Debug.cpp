#include "States.h"
    
stateName States::debug_function() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        robot_.logger.insert_log(logType::INFO, (robot_.array_sensor.debug()).c_str());
    #endif

    // safety delay default outside RUN
    vTaskDelay(DELAY_SAMPLE/portTICK_PERIOD_MS);
    
    return DEBUG;
}