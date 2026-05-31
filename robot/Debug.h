#ifndef DEBUG_H
#define DEBUG_H

// static library includes
#include <Logger.h>
#include <BallyRobot.h>

stateName debug_to_finish() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log(logType::INFO, "state_changed: debug -> finish");
    #endif

    // return the stateName of the next state
    return FINISH;
}

stateName debug_to_wait() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log(logType::INFO, "state_changed: debug -> wait");
    #endif

    // return the stateName of the next state
    return WAIT;
}

stateName debug_function() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log(logType::INFO, (ROBOT::array_sensor.debug()).c_str());
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