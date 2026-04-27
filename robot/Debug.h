#ifndef DEBUG_H
#define DEBUG_H

// static library includes
#include <Logger.h>
#include <StaticObjects.h>

stateName debug_to_finish() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("state_changed: debug -> finish", logType::INFO);
    #endif

    // return the stateName of the next state
    return FINISH;
}

stateName debug_to_wait() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("state_changed: debug -> wait", logType::INFO);
    #endif

    // return the stateName of the next state
    return WAIT;
}

stateName debug_function() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log((ROBOT::array_sensor.debug()).c_str(), logType::INFO);
    #endif

    // safety default outside RUN
    delay(10);

    return DEBUG;
}

stateName next_state_debug(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0))
        return debug_to_finish();

    // if button 2 is pressed
    if(buttons & (1 << BIT_1))
        return debug_to_wait();

    // stay in the same state
    return DEBUG;
}

#endif