#ifndef CALIBRATE_H
#define CALIBRATE_H

// header
#include <Settings.h>

// custom libraries
#include <ArraySensor.h>

// static library
#include <BallyRobot.h>

// global instance of the robot class
ROBOT& calibrate_robot_instance = ROBOT::getInstance();

stateName calibrate_to_wait() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        calibrate_robot_instance.logger.insert_log(logType::INFO, "state_changed: calibrate -> wait");
    #endif

    // return the stateName of the next state
    return WAIT;
}

stateName calibrate_to_error() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        calibrate_robot_instance.logger.insert_log(logType::INFO, "state_changed: calibrate -> error");
    #endif

    // return the stateName of the next state
    return ERROR;
}

stateName calibrate_function() {
    const bool calib = calibrate_robot_instance.array_sensor.calibrate(SAMPLES, DELAY_SAMPLE);

    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        calibrate_robot_instance.logger.insert_log(logType::INFO, ("Calibrate function called: " + std::string(!calib ? "failed" : "success")).c_str());
        calibrate_robot_instance.logger.insert_log(logType::INFO, ("Values:\n\n" + calibrate_robot_instance.array_sensor.calibrate_status()).c_str());
    #endif

    return calibrate_to_wait();
}

stateName next_state_calibrate(uint8_t buttons){
    // stay in the same state
    return CALIBRATE;
}

#endif