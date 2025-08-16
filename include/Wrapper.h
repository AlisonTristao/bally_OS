#ifndef WRAPPER_H
#define WRAPPER_H

#include <Arduino.h>
#include <TinyShell.h>
#include <Logger.h>
#include <StaticObjects.h>

uint8_t wrapper_h() {
    // get help
    String result = StaticObjects::shell.get_help("").c_str();
    #if defined(LOG_ALL) || defined(LOG_CMD)
        Logger::insert_log(result, logType::CMD);
    #endif
    return RESULT_OK;  // return 0 to indicate success
}

uint8_t wrapper_test() {
    // test command
    String result = "Test command executed";
    #if defined(LOG_ALL) || defined(LOG_CMD)
        Logger::insert_log(result, logType::CMD);
    #endif
    return RESULT_OK;  // return 0 to indicate success
}

uint8_t apply_pwm(int32_t pwm_left, int32_t pwm_right) {
    // apply PWM to motors
    StaticObjects::motor_left.applyPWM(pwm_left);
    StaticObjects::motor_right.applyPWM(pwm_right);

    String result = "PWM applied: Left = " + String(pwm_left) + ", Right = " + String(pwm_right);
    #if defined(LOG_ALL) || defined(LOG_CMD)
        Logger::insert_log(result, logType::CMD);
    #endif
    return RESULT_OK;  // return 0 to indicate success
}

bool start_shell() {
    StaticObjects::shell.create_module("help", "Module for help and information");
    StaticObjects::shell.add(wrapper_h, "h", "List all modules", "help");

    StaticObjects::shell.create_module("HB", "Module for H-Bridge control");
    StaticObjects::shell.add(apply_pwm, "apply_pwm", "Apply PWM to motors", "HB");

    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::insert_log("Shell started", logType::CMD);
    #endif
    return true;
}

#endif // WRAPPER_H