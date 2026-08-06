#ifndef SHARED_MESSAGE_TYPES_H
#define SHARED_MESSAGE_TYPES_H

#include <stdint.h>
#include <cstring>
#include <settings.h>

// enum to define log types, categorizing sent messages
// Shared, byte-for-byte, with the T-Dongle receiver firmware
// (github.com/AlisonTristao/t_dongle_develop) — keep both copies of this
// enum in the same order, new values only ever appended at the end.
enum class logType : uint8_t {
    NONE = 0,       // default type, should not be used for actual log messages
    INFO,           // informational messages that indicate normal operation and important events
    WARN,           // warning messages that indicate potential issues or important notices that are not errors
    ERRO,           // error messages
    DEBG,           // debug messages
    CMDO,           // terminal commands received
    PING,           // heartbeat probe: only the ESP-NOW delivery ACK matters, receivers ignore the payload
};

// creates a struct union to convert the message text or sound data to a byte array, 
// this will be used to send the messages over the transport protocol
struct messageContent_t {
    size_t size; 
    union { 
        char    text[MAX_CONTENT_SIZE + 1]; // +1 for null terminator
        uint8_t byte[MAX_CONTENT_SIZE + 1];              
    }; 

    // contrutctor to initialize the size and clear the content
    messageContent_t() : size(0) {
        memset(byte, 0, MAX_CONTENT_SIZE + 1);
        // set the null terminator for the text content to ensure it is always a valid string
        text[MAX_CONTENT_SIZE] = '\0';
    }
};

// struct to store logger messages, including timestamp, type, and content
// packet number and total packets are used for messages that need fragmentation due to protocol size limits
typedef struct {
    // message overhead for control and integrity verification
    uint32_t    timer;          // timestamp in ms to indicate when the message was created
    logType     type;           // message type, using the logtype enum to categorize the message
    uint16_t    packet_number;  // packet number for fragmented messages
    uint16_t    total_packets;  // total packets for fragmented messages
    uint8_t     checksum;       // simple checksum to verify message integrity

    // actual message content, limited to max_message_size to ensure the full packet stays within limits
    messageContent_t content;
} message;

inline constexpr const char* logTypeToString(logType type) {
    switch (type) {
        case logType::INFO:         return "INFO";
        case logType::CMDO:         return "CMDO";
        case logType::WARN:         return "WARN";
        case logType::ERRO:         return "ERRO";
        case logType::DEBG:         return "DEBG";
        case logType::NONE:         return "NONE";
        case logType::PING:         return "PING";
        // capotamo o corsa
        default:                    return "UNKN";
    }
};

#endif // SHARED_MESSAGE_TYPES_H
