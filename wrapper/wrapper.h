#ifndef WRAPPER_H
#define WRAPPER_H

#include <Arduino.h>
#include <TinyShell.h>
#include <Logger.h>

TinyShell shell;

uint8_t wrapper_h() {
    // get help
    String result = shell.get_help("").c_str();
    #if defined(LOG_ALL) || defined(LOG_CMD)
        Logger::IN_LOG(result, logType::CMD);
    #endif
    return RESULT_OK;  // return 0 to indicate success
}

uint8_t wrapper_test() {
    // test command
    String result = "Test command executed";
    #if defined(LOG_ALL) || defined(LOG_CMD)
        Logger::IN_LOG(result, logType::CMD);
    #endif
    return RESULT_OK;  // return 0 to indicate success
}

bool start_shell() {
    shell.create_module("help", "Module for help and information");
    shell.add(wrapper_h, "h", "List all modules", "help");

    shell.create_module("test", "Module for testing");    
    shell.add(wrapper_test, "t", "Test command", "test");
    shell.add(wrapper_test, "k", "Test command", "test");

    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::IN_LOG("Shell started", logType::CMD);
    #endif
    return true;
}

#endif // WRAPPER_H