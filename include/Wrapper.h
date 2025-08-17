#ifndef WRAPPER_H
#define WRAPPER_H

// native libraries
#include <Arduino.h>

// header
#include <EspNow.h>

// static libraries
#include <TinyShell.h>
#include <Logger.h>
#include <StaticObjects.h>
#include <StateMachine.h>

// wrapper function to log error messages
void log_error_state_machine(const char* message) {
    Logger::insert_log(message, logType::ERROR);
}

// wrapper functions for shell commands
void run_command(const String& cmd) {
    // run the command using TinyShell
    String result = StaticObjects::shell.run_line_command(cmd.c_str()).c_str();

    // log the command result
    #if defined(LOG_ALL) || defined(LOG_CMD)
        Logger::insert_log(result, logType::CMD);
    #endif
}

// wrapper function to send data and cmd
bool send_data(const uint8_t *data, size_t len) {
    // send data using esp_now
    uint8_t peer_mac[] = MAC_ADDR; // Use the defined MAC address
    esp_err_t result = esp_now_send(peer_mac, data, len);
    return result == ESP_OK;
}

static IRAM_ATTR void sampleISR(void* arg) {
    #if defined(LOG_ALL) || defined(LOG_TELEMETRY)
        if(!(StateMachine::current_state == RUN)) return;
        Logger::insert_log(
                            String(StaticObjects::encoder_left.getCount())    + ";" +
                            String(StaticObjects::encoder_right.getCount()),
                            logType::TELEMETRY);
    #endif
}

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