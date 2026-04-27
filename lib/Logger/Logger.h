#ifndef LOGGER_H
#define LOGGER_H

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <SharedMessageTypes.h>
#include <esp_err.h>
#include <esp_now.h>

#define MAX_MESSAGE_SIZE 230
#define MAX_MESSAGES 1024
#define LOGGER_MUTEX_TIMEOUT_MS 100

typedef struct {
    uint32_t timer;
    char msg[MAX_MESSAGE_SIZE + 1];
    logType type;
    uint8_t packetInfo;
} message;

class Logger {
public:
    // Define a type for the send callback function
    using SendCallback = esp_err_t (*)(const uint8_t *peer_addr, const uint8_t *data, size_t len);

    Logger();

    void setSendCallback(SendCallback callback);
    void insert_log(const String& msg, logType type);
    void send_logger(logType type);
    void clear_logger();
    void send_logger_live();

private:
    message messages[MAX_MESSAGES] = {};
    uint32_t message_count = 0;
    uint32_t last_index = 0;
    SemaphoreHandle_t mutex_ = NULL;
    SendCallback send_callback_;

    void insert_log_impl(const String& msg, logType type, uint32_t ts);

    static esp_err_t defaultSendCallback(const uint8_t *peer_addr, const uint8_t *data, size_t len);
    static void defaultCommandCallback(const String& cmd);
};

#endif // LOGGER_H
