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
    String result = ROBOT::shell.run_line_command(cmd.c_str()).c_str();

    // log the command result
    #if defined(LOG_ALL) || defined(LOG_CMD)
        ROBOT::logger.insert_log(result, logType::CMD);
    #endif
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

    // Logger sends packed `message`; normalize it to EspNow wire format.
    if (len == sizeof(::message)) {
        const ::message* logger_msg = reinterpret_cast<const ::message*>(data);
        outgoing.timer = logger_msg->timer;
        outgoing.type = logger_msg->type;
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

bool start_shell_wrappers() {
    ROBOT::shell.create_module("help", "Module for help and information");
    ROBOT::shell.add(wrapper_h, "h", "List all modules", "help");

    ROBOT::shell.create_module("robot", "Module for robot control commands");
    ROBOT::shell.add(set_led, "set_led", "Set LED color and brightness", "robot");
    ROBOT::shell.add(led_off, "led_off", "Turn off the LED", "robot");

    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("Shell started", logType::CMD);
    #endif
    return true;
}

#endif // WRAPPER_H