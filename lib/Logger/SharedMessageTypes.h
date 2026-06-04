#ifndef SHARED_MESSAGE_TYPES_H
#define SHARED_MESSAGE_TYPES_H

#include <stdint.h>
#include "esp_timer.h"
#include <cstring>

// here we define the data structures and definitions for the log messages, 
// that will be shared between the logger and the transport protocol (e.g., esp-now)

// max espnow packet size -> 250bytes

// protocol overhead: 
//   4 bytes for timestamp 
// + 1 byte for type 
// + 2 byte for packet number 
// + 2 byte for total packets 
// + 1 byte for checksum 
// + 4 byte para o length
// = 14 bytes
    
// therefore, max message size is 250 - 14 = 236 bytes

// but, to use a size that is a multiple of 4 for better memory alignment, we will use 230 bytes for the content

// the N16R8 chip has 8mb of external psram, which is 8 * 1024 * 1024 = 8388608 bytes
// if we use 250 bytes per message (including overhead), we can store up to 8388608 / 250 = 33554
// but to be safe, we use 90% of the available memory for the logger, which gives us a limit of 30.200 
// max packets in psram = 0.9 * 33554 = 30.200 -> round to 30000

#define MAX_PACKET_SIZE         250   // if we change the transport protocol, we can increase this value
#define PROTOCOL_OVERHEAD_SIZE  20    // overhead for the protocol, including timestamp, type, packet number, total packets and checksum
#define MAX_CONTENT_SIZE        229   // -1 to ensure we have space for the null terminator
#define MAX_PACKETS_IN_PSRAM    30000 // limit for messages in memory - watch out for available ram limits
#define LOGGER_MUTEX_TIMEOUT_MS 100   // time in ms to wait for the logger to be available - used to avoid deleting messages during printing

// to empty the array and free the mutex to other tasks, 
// the code flush the array in chuncks, and to free the core to other tasks,
// we limit a max chucks per flush
#define MAX_CHUNKS_PER_FLUSH    10 // 10 chucks per flush 
#define BLOCK_SIZE              16 // the chunck has 16 messages 

// enum to define log types, categorizing sent messages
enum class logType : uint8_t {
    NONE = 0,       // default type, should not be used for actual log messages
    INFO,           // informational messages that indicate normal operation and important events
    WARN,           // warning messages that indicate potential issues or important notices that are not errors
    ERRO,           // error messages
    DEBG,           // debug messages
    CMDO,           // terminal commands received
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
        // capotamo o corsa
        default:                    return "UNKN"; 
    }
};

#endif // SHARED_MESSAGE_TYPES_H