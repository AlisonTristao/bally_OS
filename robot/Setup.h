#ifndef SETUP_H
#define SETUP_H

// header
#include <Settings.h>
#include <StaticObjects.h>

stateName setup_to_wait() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("state_changed: setup -> wait", logType::INFO);
    #endif

    // return the stateName of the next state
    return WAIT;
}

stateName setup_to_error() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("state_changed: setup -> error", logType::INFO);
    #endif

    // return the stateName of the next state
    return ERROR;
}

stateName setup_function() {
    // init pins direction, settings, i2c communication...
    if (!configure_pins())
        return setup_to_error();

    // init shell wrappers
    if (!start_shell_wrappers())
        return setup_to_error();

    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("Setup function called", logType::INFO);
    #endif

    // setup completed, go to wait state
    return setup_to_wait();
}

stateName next_state_setup(uint8_t buttons){
    // stay in the same state
    return SETUP;
}

#endif