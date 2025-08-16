#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <vector>
#include <Settings.h>

// maximum message size
#define MAX_MESSAGE_SIZE 230
bool send_data(const uint8_t* data, size_t len);

// types of logs 
enum class logType {
    NONE,
    INFO,
    TELEMETRY,
    ERROR,
    DEBUG
};

// message structure
typedef struct message {
    uint32_t timer;
    char msg[MAX_MESSAGE_SIZE + 1]; // +1 for null terminator
    logType type;
} message;

// logger class
class Logger {
    public:
        /*
            @brief: insert a message into the logger
            @param: msg -> message to be inserted
            @param: type -> type of the message
        */
        static void IN_LOG(const String& msg, logType type) {
            IN_LOG_impl(msg, type, millis());
        }

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

        /*
            @brief: convert logType to string
            @param: type -> type of the message
        */
        static String logTypeToString(logType type);
    private:
        // messages vector
        static std::vector<message> messages;

        // save the index of the last message printed
        static uint32_t last_index;

        // log message
        static void IN_LOG_impl(const String& msg, logType type, unsigned long ts);
};

#endif // LOGGER_H