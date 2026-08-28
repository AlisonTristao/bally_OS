#ifndef LOGGER_H
#define LOGGER_H

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

#include <cstddef>
#include <cstdint>
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <LogTypes.h>
#include <Settings.h>

// Only for btp::Header, named in the LogRadioSeal function-pointer type
// below. BtpTransport.h defines the equivalent alias BtpSealFn, but Logger.h
// is pulled in by RobotSettings/OTAUpdater, whose PlatformIO libraries do not
// depend on BtpTransport -- including it here breaks their `chain` LDF
// resolution on the esp32-s3 build. btp/codec.hpp is header-only and already
// on every consumer's path (the btp ESP-IDF component's public include).
#include <btp/codec.hpp>

class SDCard;
class TinyShell;
class RobotSettings;
class BtpEndpoint;

class Logger {
public:
    using StorageCallback = bool (*)(const uint8_t *data, size_t len, void *context);

    // initialize metods
    Logger();
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    void begin();

    // Structurally identical to BtpTransport.h's BtpSealFn (and to
    // BtpEndpoint::SealFn): a channel sealer passed straight through to
    // BtpEndpoint::send_fragment. Redeclared here rather than pulled in with
    // <BtpTransport.h> -- see the include note above.
    using LogRadioSeal = bool (*)(void* context, const btp::Header& header,
                                  std::uint16_t payload_size,
                                  const std::uint8_t* plaintext,
                                  std::uint8_t* out);

    // acess methods for the logger
    /*
    * @brief Attach the process-wide BTP endpoint used to allocate sequences,
    * encode LOG frames and hand exact wire bytes to the selected transport.
    *
    * `seal`/`seal_context` (default nullptr) are the channel-B sealer for LOG
    * over the radio. LOG rides channel B (TraceView<->robot, key E --
    * bally_channels.h), so the real caller passes RadioSeal::seal_e. When
    * set, every LOG frame flush_logs() and send_log_direct() put on the radio
    * is sealed with it or NOT SENT (fail-closed, no cleartext fallback). The
    * SD flush path (flush_logs_to) is never sealed: those files are local
    * diagnostics and must stay readable without the key.
    */
    void configure_btp(BtpEndpoint& endpoint, LogRadioSeal seal = nullptr,
                       void* seal_context = nullptr);

    /**
     * @brief Configure how many packets flush_logs() sends per call
     * (max_chunks_per_flush * block_size). Defaults match the historic
     * MAX_CHUNKS_PER_FLUSH/BLOCK_SIZE compile-time values; call this once
     * after loading RobotSettings to apply the runtime-configured ones.
     */
    void set_flush_limits(uint32_t max_chunks_per_flush, uint32_t block_size);

    /*
    * @brief Insert a log message into the logger's buffer. 
    * This method is thread-safe and can be called from multiple tasks concurrently.
    * 
    * @param msg The log message to be inserted. This is a char*object that can contain any text.
    * @param type The type of the log message, using the logType enum to categorize
    */
    void insert_log(logType type, const char* msg);
    void insert_logf(logType type, const char* format, ...);

    /**
     * @brief Send text directly without retaining it in the PSRAM ring.
     *
     * Used for shell responses while the SD card belongs to the USB host.
     */
    bool send_log_direct(logType type, const char* msg);

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
     * @brief Set the system clock used to name new SD log files, validating
     * the calendar date instead of silently normalizing an invalid one.
     *
     * @param posix_tz TZ string applied before interpreting the given time
     * as local (see RobotSettings::data().timezone).
     */
    bool set_datetime(uint16_t year, uint8_t month, uint8_t day,
                      uint8_t hour, uint8_t minute, uint8_t second,
                      const char* posix_tz);

    /**
     * @brief Save every retained PSRAM log to the SD card: either start a
     * new, uniquely-named dated file or append to the most recently written
     * one (falling back to the newest matching file already on the card).
     *
     * @param out_filename Optional; receives the file name written to.
     * @return true when every retained message was written and the file
     * closed cleanly.
     */
    bool flush_to_sd(SDCard& card, bool append,
                     char* out_filename = nullptr, size_t out_capacity = 0);

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

    /**
     * @brief Register the "logger" shell module (test_packet/psram_usage/
     * set_datetime/flush_new/flush_append/print_log), backed by this
     * instance.
     * @param settings Read live at call time (data().timezone) by
     * "set_datetime", not captured.
     * @param mark_direct_output Called after "flush_new"/"flush_append" send
     * their reply, so it is not itself retained in the PSRAM log (see
     * ROBOT::sendNextShellOutputDirect).
     */
    void register_shell_commands(TinyShell& shell, SDCard& sd_card, RobotSettings& settings,
                                 std::function<void()> mark_direct_output);

private:
    // private members for the logger
    struct StoredLogHeader {
        uint64_t timestamp_us;
        uint32_t sequence;
        uint16_t length;
        logType type;
        uint8_t reserved;
    };

    static_assert(sizeof(StoredLogHeader) == 16, "Unexpected stored log header size");

    uint8_t* storage_ = nullptr; // variable-length byte ring in external PSRAM
    const uint32_t storage_capacity_ = LOGGER_PSRAM_CAPACITY_BYTES;
    uint32_t max_chunks_per_flush_ = 10; // RobotSettings default; see set_flush_limits
    uint32_t block_size_ = 16;           // RobotSettings default; see set_flush_limits
    uint32_t write_offset_ = 0;
    uint32_t read_offset_ = 0;
    uint32_t used_bytes_ = 0;

    // ESP-NOW has its own cursor. Sending does not release retained PSRAM.
    uint32_t send_offset_ = 0;
    uint32_t pending_send_bytes_ = 0;
    bool send_record_active_ = false;
    StoredLogHeader send_header_{};
    // Octets of the current record already put on the radio. This was a
    // uint8_t fragment index; it became a byte count once each radio chunk
    // started going out as its own single-fragment LOG message rather than
    // fragment N of one -- an AEAD tag covers the whole logical payload and
    // cannot cover a slice of one, so a sealed LOG record over one ESP-NOW
    // frame is sent as several independent sealed messages (each a separate
    // line at the desktop). The SD flush path (flush_logs_to) still writes
    // one multi-fragment record and is never sealed.
    uint32_t send_record_bytes_done_ = 0;

    // Only one consumer (ESP-NOW or SD) may walk the ring at a time.
    bool consumer_busy_ = false;

    SemaphoreHandle_t mutex_ = NULL;
    BtpEndpoint* endpoint_ = nullptr;
    // Channel-B sealer for LOG on the radio (RadioSeal::seal_e in real
    // firmware); nullptr means send LOG in the clear, the pre-topico state
    // and what env:native would exercise if it built Logger. Guarded by
    // mutex_ like endpoint_.
    LogRadioSeal endpoint_seal_ = nullptr;
    void* endpoint_seal_context_ = nullptr;
    bool initialized_ = false;

    // SD log file naming/appending state (flush_to_sd/set_datetime).
    // Mirrors SDFileInfo::MAX_NAME_LENGTH (lib/SDCard/SDCard.h) without
    // requiring Logger.h to include the full SDCard header.
    static constexpr size_t kLogFilenameCapacity = 128;
    bool clock_synchronized_ = false;
    char last_log_file_[kLogFilenameCapacity] = {};

    bool make_log_filename(SDCard& card, char* filename, size_t capacity) const;
    bool find_latest_log_file(SDCard& card, char* filename, size_t capacity) const;

    // private methods for the logger
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
    * @param timestamp_us Monotonic microseconds at logical-message creation.
    */
    void insert_log_impl(const uint8_t* data, size_t len, logType type,
                         uint64_t timestamp_us);

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

};

#endif // LOGGER_H
