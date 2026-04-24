#ifndef WRAPPER_H
#define WRAPPER_H

// ==================== SYSTEM ====================
#include <Arduino.h>

// ==================== PROJECT HEADERS ====================
#include <EspNow.h>

// ==================== UTILITIES ====================
#include <Flags.h>
#include <Logger.h>
#include <StateMachine.h>
#include <StaticObjects.h>

// ==================== EXTERNAL LIBRARIES ====================
#include <TinyShell.h>


// wrapper functions for shell commands
void run_command(const String& cmd) {
    // run the command using TinyShell
    String result = ROBOT::shell.run_command_line(cmd.c_str()).c_str();

    // log the command result
    #if defined(LOG_ALL) || defined(LOG_CMD)
        ROBOT::logger.insert_log(result, logType::CMD);
    #endif
}

/**
 * @brief Virtually triggers a button, keeping the flag active for time_ms milliseconds
 * @param button Button index
 * @param time_ms Time in milliseconds
 */

uint8_t triggerVirtualButton(uint8_t button) {
    if (button > 2) {
        ROBOT::logger.insert_log(String("invalid button index {") + String(button) + "}. expected: 0..2", logType::ERROR);
        return RESULT_ERROR;
    }

    // Set the button flag
    ROBOT::buttons.setFlag(button);
    // Log the action
    ROBOT::logger.insert_log(String("button {") + String(button) + "} virtually triggered", logType::INFO);
    // The duration is controlled by the flags system (checkFlagsDuration)
    // If you want to force the time, you can implement a timer/task to clear the flag after time_ms

    return RESULT_OK;
}

static bool normalizeSideSensorIndex(uint8_t sensor, uint8_t& normalized) {
    if (sensor <= 1) {
        normalized = sensor;
        return true;
    }

    if (sensor == LEFT) {
        normalized = 0;
        return true;
    }

    if (sensor == RIGHT) {
        normalized = 1;
        return true;
    }

    return false;
}

uint8_t triggerVirtualSideSensor(uint8_t sensor, uint32_t time_ms) {
    uint8_t sensorIndex = 0;
    if (!normalizeSideSensorIndex(sensor, sensorIndex)) {
        ROBOT::logger.insert_log(String("invalid side sensor {") + String(sensor) + "}. expected: 0/1 or LEFT/RIGHT pin", logType::ERROR);
        return RESULT_ERROR;
    }

    if (time_ms > 0)
        ROBOT::sideSensors.setTimeLimit(sensorIndex, time_ms);

    // Set the side sensor flag
    ROBOT::sideSensors.setFlag(sensorIndex);
    // Log the action
    ROBOT::logger.insert_log(String("side sensor {") + String(sensorIndex) + "} virtually triggered", logType::INFO);
    // The duration is controlled by the flags system (checkFlagsDuration)
    // If you want to force the time, you can implement a timer/task to clear the flag after time_ms

    return RESULT_OK;
}

/**
 * @brief Envia dados genéricos via ESP-NOW
 * @param data Dados a enviar
 * @param len Comprimento dos dados
 * @return true se enviado com sucesso
 */

 // ================== ESP-NOW SEND WRAPPERS ==================

bool send_data(const uint8_t *data, size_t len) {
    if (data == nullptr || len == 0) {
        return false;
    }

    EspNowManager::message outgoing = {};
    outgoing.timer = millis();
    outgoing.type = logType::INFO;
    outgoing.packetInfo = makePacketInfo(0, true);

    // Logger sends packed `message`; normalize it to EspNow wire format.
    if (len == sizeof(::message)) {
        const ::message* logger_msg = reinterpret_cast<const ::message*>(data);
        outgoing.timer = logger_msg->timer;
        outgoing.type = logger_msg->type;
        outgoing.packetInfo = logger_msg->packetInfo;
        strncpy(outgoing.msg, logger_msg->msg, EspNowManager::MESSAGE_TEXT_SIZE - 1);
        outgoing.msg[EspNowManager::MESSAGE_TEXT_SIZE - 1] = '\0';
    } else {
        const size_t copy_size = (len < (EspNowManager::MESSAGE_TEXT_SIZE - 1)) ? len : (EspNowManager::MESSAGE_TEXT_SIZE - 1);
        memcpy(outgoing.msg, data, copy_size);
        outgoing.msg[copy_size] = '\0';
    }

    uint8_t peer_mac[] = MAC_ADDR;
    return espNowManager.sendToMac(peer_mac, outgoing);
}

static bool send_text_message(const char* text, logType type) {
    if (text == nullptr) {
        return false;
    }

    uint8_t peer_mac[] = MAC_ADDR;

    EspNowManager::message msg = {};
    msg.timer = millis();
    msg.type = type;
    msg.packetInfo = makePacketInfo(0, true);

    const size_t len = strlen(text);
    const size_t copy_size = (len < (EspNowManager::MESSAGE_TEXT_SIZE - 1)) ? len : (EspNowManager::MESSAGE_TEXT_SIZE - 1);
    memcpy(msg.msg, text, copy_size);
    msg.msg[copy_size] = '\0';

    return espNowManager.sendToMac(peer_mac, msg);
}

/**
 * @brief Envia mensagem de informação via ESP-NOW
 * @param message Mensagem a enviar
 * @return true se enviado com sucesso
 */
bool send_info(const char* message) {
    return send_text_message(message, logType::INFO);
}

/**
 * @brief Envia mensagem de telemetria via ESP-NOW
 * @param telemetry Dados de telemetria
 * @return true se enviado com sucesso
 */
bool send_telemetry(const char* telemetry) {
    return send_text_message(telemetry, logType::TELEMETRY);
}

/**
 * @brief Envia mensagem de erro via ESP-NOW
 * @param error Mensagem de erro
 * @return true se enviado com sucesso
 */
bool send_error(const char* error) {
    return send_text_message(error, logType::ERROR);
}

/**
 * @brief Envia mensagem de debug via ESP-NOW
 * @param debug Mensagem de debug
 * @return true se enviado com sucesso
 */
bool send_debug(const char* debug) {
    return send_text_message(debug, logType::DEBUG);
}

// ================== RGB LED WRAPPERS ==================

// Example wrapper functions for RGB LED control
uint8_t set_led(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
    ROBOT::rgb_led.set(r, g, b, brightness);
    return RESULT_OK;
}

// led off wrapper
uint8_t led_off() {
    ROBOT::rgb_led.off();
    return RESULT_OK;
}

uint8_t wrapper_h() {
    // get help
    String result = ROBOT::shell.get_help("").c_str();
    #if defined(LOG_ALL) || defined(LOG_CMD)
        ROBOT::logger.insert_log(result, logType::CMD);
    #endif
    return RESULT_OK;  // return 0 to indicate success
}

uint8_t wrapper_types_help() {
    #if defined(LOG_ALL) || defined(LOG_CMD)
        ROBOT::logger.insert_log("Type aliases used by shell:", logType::CMD);
        ROBOT::logger.insert_log("u8  => uint8_t", logType::CMD);
        ROBOT::logger.insert_log("u16 => uint16_t", logType::CMD);
        ROBOT::logger.insert_log("u32 => uint32_t", logType::CMD);
        ROBOT::logger.insert_log("u64 => uint64_t", logType::CMD);
        ROBOT::logger.insert_log("i8  => int8_t", logType::CMD);
        ROBOT::logger.insert_log("i16 => int16_t", logType::CMD);
        ROBOT::logger.insert_log("i32 => int32_t", logType::CMD);
        ROBOT::logger.insert_log("i64 => int64_t", logType::CMD);
        ROBOT::logger.insert_log("f32 => float", logType::CMD);
        ROBOT::logger.insert_log("f64 => double", logType::CMD);
        ROBOT::logger.insert_log("b   => bool", logType::CMD);
        ROBOT::logger.insert_log("str => string", logType::CMD);
    #endif
    return RESULT_OK;
}

bool start_shell_wrappers() {
    ROBOT::shell.create_module("help", "Module for help and information");
    ROBOT::shell.add(wrapper_h, "h", "List all modules", "help");
    ROBOT::shell.add(wrapper_types_help, "types", "Show type aliases (u8, i32, str...)", "help");

    ROBOT::shell.create_module("robot", "Module for robot control commands");
    ROBOT::shell.add(set_led, "set_led", "Set LED color and brightness", "robot");
    ROBOT::shell.add(led_off, "led_off", "Turn off the LED", "robot");
    ROBOT::shell.add(triggerVirtualButton, "btn", "Virtually trigger a button", "robot");
    ROBOT::shell.add(triggerVirtualSideSensor, "ssr", "Virtually trigger a side sensor", "robot");

    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("Shell started", logType::CMD);
    #endif
    return true;
}

#endif // WRAPPER_H