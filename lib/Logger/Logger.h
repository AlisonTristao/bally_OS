#ifndef LOGGER_H
#define LOGGER_H

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <SharedMessageTypes.h>

#ifndef SHARED_MESSAGE_TYPES_H
#error "This library depends on SharedMessageTypes.h, to use the data structures and definitions for the log messages. "
#endif

class Logger {
public:
    // Define a type for the send callback function
    using SendCallback = bool (*)(const uint8_t *data, size_t len);
    using StorageCallback = bool (*)(const uint8_t *data, size_t len, void *context);

    // initialize metods
    Logger();
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
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

    /**
     * @brief Save all retained messages using a storage callback and release
     * their PSRAM only after each complete message is stored.
     */
    bool flush_logs_to(StorageCallback callback, void* context);

    /**
     * @brief Send an already formatted message without storing it in PSRAM.
     * Used to replay log files without creating duplicated retained logs.
     */
    bool send_message(const message& msg);

    /**
     * @brief Get the current write index of the logger's buffer. This is used for debugging purposes to track where the next log message will be written in the buffer.
     * 
     * @return The current write index of the logger's buffer.
     */
    float get_write_pct() const {
        return (used_bytes_ * 100.0f) /
               static_cast<float>(storage_capacity_);
    }
    uint32_t get_used_bytes() const { return used_bytes_; }
    uint32_t get_capacity_bytes() const { return storage_capacity_; }
    uint32_t get_pending_send_bytes() const { return pending_send_bytes_; }

private:
    // private members for the logger
    struct StoredLogHeader {
        uint32_t timer;
        uint16_t length;
        logType type;
        uint8_t reserved;
    };

    static_assert(sizeof(StoredLogHeader) == 8, "Unexpected stored log header size");
    static_assert(sizeof(message) <= MAX_PACKET_SIZE, "ESP-NOW message exceeds packet limit");

    uint8_t* storage_ = nullptr; // variable-length byte ring in external PSRAM
    const uint32_t storage_capacity_ = LOGGER_PSRAM_CAPACITY_BYTES;
    uint32_t write_offset_ = 0;
    uint32_t read_offset_ = 0;
    uint32_t used_bytes_ = 0;

    // ESP-NOW has its own cursor. Sending does not release retained PSRAM.
    uint32_t send_offset_ = 0;
    uint32_t pending_send_bytes_ = 0;
    bool send_record_active_ = false;
    StoredLogHeader send_header_{};
    uint16_t send_packet_index_ = 0;

    // Only one consumer (ESP-NOW or SD) may walk the ring at a time.
    bool consumer_busy_ = false;

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

    // byte-ring helpers; caller must hold mutex_
    void write_to_ring(const void* source, size_t len);
    void read_from_ring(uint32_t offset, void* destination, size_t len) const;
    bool read_header_at(uint32_t offset, uint32_t available,
                        StoredLogHeader& header) const;
    void clear_ring();
    bool begin_consumer();
    void end_consumer();

    // funtion to menager the mutex
    bool wait_for_mutex();  // wait indefinitely for the mutex to be available
    bool check_mutex();     // check if the mutex is available without waiting
    void free_mutex();      // release the mutex

    static bool defaultSendCallback(const uint8_t *data, size_t len);
    static void defaultCommandCallback(const char* cmd);
};

#endif // LOGGER_H
