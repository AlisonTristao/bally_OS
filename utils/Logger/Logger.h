#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <vector>

// types of logs 
enum class logType {
    NONE,
    INFO,
    TELEMETRY,
    ERROR,
    DEBUG
};

// message structure
struct message {
    uint32_t timer;
    String msg;
    logType type;
}; 

// logger class
class Logger {
    public:
        /*
            @brief: insert a message into the logger
            @param: msg -> message to be inserted
            @param: type -> type of the message
        */
        static void IN_LOG(String msg, logType type);

        /*
            @brief: print all messages inserted
            @param: type -> type of the message
        */
        static void OUT_LOGGER(logType type);

        /*
            @brief: clear all messages inserted 
            @param: type -> type of the message
        */
        static void CLEAR_LOG();

        /*
            @brief: print last message inserted 
            @param: type -> type of the message
        */
        static void OUT_LOGGER_LIVE();
    private:
        // messages vector
        static std::vector<message> messages;

        // save the index of the last message printed
        static uint32_t last_index;
};

#endif // LOGGER_H