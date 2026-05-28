#ifndef LOGGER_H
#define LOGGER_H

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <SharedMessageTypes.h>
#include <stdarg.h>

#ifndef SHARED_MESSAGE_TYPES_H
#error "This library depends on SharedMessageTypes.h, to use the data structures and definitions for the log messages. "
#endif

class Logger {
public:
    // Define a type for the send callback function
    using SendCallback = bool (*)(const uint8_t *data, size_t len);

    // initialize metods
    Logger();
    ~Logger();
    void begin();

    // acess methods for the logger
    /*
    * @brief Set the callback function that will be called to send the log messages. 
    * This allows the logger to be flexible and adaptable to different output methods (e.g., serial, 
    * network, etc.). If a null pointer is passed, the logger will use a default callback that does nothing.
    *
    * @param callback A function pointer to the callback function that will be called to send the log messages.
    */
    void set_send_callback(SendCallback callback);   
    
    /*
    * @brief Insert a log message into the logger's buffer. 
    * This method is thread-safe and can be called from multiple tasks concurrently.
    * 
    * @param msg The log message to be inserted. This is a char*object that can contain any text.
    * @param type The type of the log message, using the logType enum to categorize
    */
    void insert_log(logType type, const char* msg);
    void insert_logf(logType type, const char* format, ...);

    /*
    * @brief Send the log messages that are currently in the logger's buffer using the configured send callback.
    * This method should be called periodically (e.g., in a task) to ensure that the log messages are sent out. 
    * It will send all the messages in the buffer and then clear it
    */
    void flush_logs();

private:
    // private members for the logger
    message messages[MAX_PACKETS_IN_RAM] = {};
    uint32_t message_count = 0;
    uint32_t last_index = 0;
    SemaphoreHandle_t mutex_ = NULL;
    SendCallback send_callback_;
    bool initialized_ = false;

    // private methods for the logger
    /*
    * @brief Calculate the checksum for a log message.
    * This is used to ensure the integrity of the log messages.
    *
    * @param msg The log message for which to calculate the checksum.
    * @return The calculated checksum.
    */
    uint8_t calculate_checksum(const message &msg);

    /*
    * @brief Reset the loop counter, which is used to keep track of the number of messages in the buffer. 
    * This method is called when inserting a new log message to ensure that the buffer does not overflow.
    */
    void reset_loop_counter();

    /*
    * @brief Internal method to insert a log message into the buffer without locking.
    * This method is called by the public insert_log methods, which handle the locking to ensure thread safety.
    * The insert_log_impl method assumes that the caller has already acquired the mutex lock before calling it.
    *
    * @param data Pointer to payload bytes
    * @param len Payload length in bytes
    * @param type The type of the log message, using the logType enum to categorize
    * @param ts The timestamp in milliseconds when the log message was created. This is used for tracking the age of the log messages and for debugging purposes.
    * @param null_terminate Whether to add a null terminator for text payloads
    */
    void insert_log_impl(const uint8_t* data, size_t len, logType type, uint32_t ts);

    // funtion to menager the mutex
    bool wait_for_mutex();  // wait indefinitely for the mutex to be available
    bool check_mutex();     // check if the mutex is available without waiting
    void free_mutex();      // release the mutex

    static bool defaultSendCallback(const uint8_t *data, size_t len);
    static void defaultCommandCallback(const char* cmd);
};

#endif // LOGGER_H
