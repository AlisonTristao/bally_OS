#ifndef LOGGER_H
#define LOGGER_H

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

#include <Arduino.h>

// maximum message size
#define MAX_MESSAGE_SIZE 230
#define MAX_MESSAGES 1024

// wrapper function to send data and cmd 
bool send_data(const uint8_t* data, size_t len);

// wrapper function to run commands
void run_command(const String& cmd);

// types of logs 
enum class logType {
    NONE,
    INFO,
    CMD,
    TELEMETRY,
    ERROR,
    DEBUG
};

// message structure
typedef struct {
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
        static void insert_log(const String& msg, logType type) {
            insert_log_impl(msg, type, millis());
        }

        /** 
         * @brief: register a command
         * @param: cmd -> command to be registered
        */
        static void insert_cmd(const String& cmd) {
            insert_cmd_impl(cmd);
        }

        /*
            @brief: print all messages inserted
            @param: type -> type of the message
        */
        static void send_logger(logType type);

        /*
            @brief: clear all messages inserted 
            @param: type -> type of the message
        */
        static void clear_logger();

        /*
            @brief: print last message inserted 
            @param: type -> type of the message
        */
        static void send_logger_live();

        /*
            @brief: convert logType to string
            @param: type -> type of the message
        */
        static String logTypeToString(logType type);
    private:
        // messages array
        static message messages[MAX_MESSAGES];
        static uint32_t message_count;

        // save the index of the last message printed
        static uint32_t last_index;

        // log message
        static void insert_log_impl(const String& msg, logType type, uint32_t ts);

        // log commands
        static void insert_cmd_impl(const String& cmd);

        // wait for the logger to be free
        static bool mutex;
};

#endif // LOGGER_H