#ifndef DEBUG_H
#define DEBUG_H

// static library includes
#include <Logger.h>
#include <BallyRobot.h>

ROBOT& debug_robot_instance = ROBOT::getInstance();

stateName debug_to_finish() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        debug_robot_instance.logger.insert_log(logType::INFO, "state_changed: debug -> finish");
    #endif

    // return the stateName of the next state
    return FINISH;
}

stateName debug_to_wait() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        debug_robot_instance.logger.insert_log(logType::INFO, "state_changed: debug -> wait");
    #endif

    // return the stateName of the next state
    return WAIT;
}

stateName debug_function() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        debug_robot_instance.logger.insert_log(logType::INFO, (debug_robot_instance.array_sensor.debug()).c_str());
    #endif

    // safety delay default outside RUN
    vTaskDelay(DELAY_SAMPLE/portTICK_PERIOD_MS);
    
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