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
    String result = ROBOT::runLineCommand(cmd);

    // log the command result
    #if defined(LOG_ALL) || defined(LOG_CMD)
        Logger::insert_log(result, logType::CMD);
    #endif
}

// ================== ESP-NOW SEND WRAPPERS ==================

static EspNowManager::logType map_logger_type_to_espnow(::logType type) {
    switch (type) {
        case logType::INFO:
            return EspNowManager::logType::INFO;
        case logType::TELEMETRY:
            return EspNowManager::logType::TELEMETRY;
        case logType::ERROR:
            return EspNowManager::logType::ERROR;
        case logType::DEBUG:
            return EspNowManager::logType::DEBUG;
        case logType::CMD:
            return EspNowManager::logType::INFO;
        case logType::NONE:
        default:
            return EspNowManager::logType::NONE;
    }
}

static bool send_text_message(const char* text, EspNowManager::logType type) {
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
 * @brief Envia dados genéricos via ESP-NOW
 * @param data Dados a enviar
 * @param len Comprimento dos dados
 * @return true se enviado com sucesso
 */
bool send_data(const uint8_t *data, size_t len) {
    if (data == nullptr || len == 0) {
        return false;
    }

    EspNowManager::message outgoing = {};
    outgoing.timer = millis();
    outgoing.type = EspNowManager::logType::INFO;

    // Logger sends packed `message`; normalize it to EspNow wire format.
    if (len == sizeof(::message)) {
        const ::message* logger_msg = reinterpret_cast<const ::message*>(data);
        outgoing.timer = logger_msg->timer;
        outgoing.type = map_logger_type_to_espnow(logger_msg->type);
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

/**
 * @brief Envia mensagem de informação via ESP-NOW
 * @param message Mensagem a enviar
 * @return true se enviado com sucesso
 */
bool send_info(const char* message) {
    return send_text_message(message, EspNowManager::logType::INFO);
}

/**
 * @brief Envia mensagem de telemetria via ESP-NOW
 * @param telemetry Dados de telemetria
 * @return true se enviado com sucesso
 */
bool send_telemetry(const char* telemetry) {
    return send_text_message(telemetry, EspNowManager::logType::TELEMETRY);
}

/**
 * @brief Envia mensagem de erro via ESP-NOW
 * @param error Mensagem de erro
 * @return true se enviado com sucesso
 */
bool send_error(const char* error) {
    return send_text_message(error, EspNowManager::logType::ERROR);
}

/**
 * @brief Envia mensagem de debug via ESP-NOW
 * @param debug Mensagem de debug
 * @return true se enviado com sucesso
 */
bool send_debug(const char* debug) {
    return send_text_message(debug, EspNowManager::logType::DEBUG);
}

// ================== RGB LED WRAPPERS ==================

/**
 * @brief Wrapper para ligar LED RGB verde (sucesso)
 */
uint8_t led_set_green() {
    rgbLed.setGreen();
    Logger::insert_log("LED RGB set to GREEN", logType::INFO);
    return RESULT_OK;
}

/**
 * @brief Wrapper para ligar LED RGB vermelho (erro)
 */
uint8_t led_set_red() {
    rgbLed.setRed();
    Logger::insert_log("LED RGB set to RED", logType::INFO);
    return RESULT_OK;
}

/**
 * @brief Wrapper para ligar LED RGB azul (wait/idle)
 */
uint8_t led_set_blue() {
    rgbLed.setBlue();
    Logger::insert_log("LED RGB set to BLUE", logType::INFO);
    return RESULT_OK;
}

/**
 * @brief Wrapper para ligar LED RGB amarelo (aviso)
 */
uint8_t led_set_yellow() {
    rgbLed.setYellow();
    Logger::insert_log("LED RGB set to YELLOW", logType::INFO);
    return RESULT_OK;
}

/**
 * @brief Wrapper para desligar LED RGB
 */
uint8_t led_off() {
    rgbLed.off();
    Logger::insert_log("LED RGB turned OFF", logType::INFO);
    return RESULT_OK;
}

/**
 * @brief Wrapper para piscar LED RGB
 * @param duration Duração do piscar em ms (padrão 100ms)
 */
uint8_t led_blink(uint32_t duration = 100) {
    rgbLed.blink(duration);
    Logger::insert_log("LED RGB blinking for " + String(duration) + "ms", logType::INFO);
    return RESULT_OK;
}

/**
 * @brief Wrapper para definir cor customizada do LED RGB
 * @param r Vermelho (0-255)
 * @param g Verde (0-255)
 * @param b Azul (0-255)
 */
uint8_t led_set_color(uint8_t r, uint8_t g, uint8_t b) {
    rgbLed.setColor(r, g, b);
    Logger::insert_log("LED RGB set to RGB(" + String(r) + "," + String(g) + "," + String(b) + ")", logType::INFO);
    return RESULT_OK;
}

/**
 * @brief Wrapper para ajustar brilho do LED RGB
 * @param brightness Brilho (0-31)
 */
uint8_t led_set_brightness(uint8_t brightness) {
    rgbLed.setBrightness(brightness);
    Logger::insert_log("LED RGB brightness set to " + String(brightness), logType::INFO);
    return RESULT_OK;
}

static IRAM_ATTR void sampleISR(void* arg) {
    #if defined(LOG_ALL) || defined(LOG_TELEMETRY)
        if(!(StateMachine::current_state == RUN)) return;
        /*Logger::insert_log(
                            String(ROBOT::encoder_left.getCount())    + ";" +
                            String(ROBOT::encoder_right.getCount()),
                            logType::TELEMETRY);*/
    #endif
}

uint8_t wrapper_h() {
    // get help
    String result = ROBOT::shell().get_help("").c_str();
    #if defined(LOG_ALL) || defined(LOG_CMD)
        Logger::insert_log(result, logType::CMD);
    #endif
    return RESULT_OK;  // return 0 to indicate success
}

uint8_t wrapper_test() {
    // test command
    String result = "Test command executed";
    #if defined(LOG_ALL) || defined(LOG_CMD)
        Logger::insert_log(result, logType::CMD);
    #endif
    return RESULT_OK;  // return 0 to indicate success
}

uint8_t apply_pwm(int32_t pwm_right, int32_t pwm_left) {
    // apply PWM to motors
    ROBOT::applyMotorPWM(pwm_left, pwm_right);

    String result = "PWM applied: Left = " + String(pwm_left) + ", Right = " + String(pwm_right);
    #if defined(LOG_ALL) || defined(LOG_CMD)
        Logger::insert_log(result, logType::CMD);
    #endif
    return RESULT_OK;  // return 0 to indicate success
}

uint8_t set_state_control(uint8_t state) {
    StateMachine::current_state = state;
    ROBOT::setState(state);

    // log the state change
    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::insert_log("State machine changed to state: " + String(state), logType::INFO);
    #endif

    return RESULT_OK;  // return 0 to indicate success
}

bool start_shell() {
    Logger::setSendCallback(send_data);
    Logger::setCommandCallback(run_command);

    ROBOT::shell().create_module("help", "Module for help and information");
    ROBOT::shell().add(wrapper_h, "h", "List all modules", "help");

    ROBOT::shell().create_module("HB", "Module for H-Bridge control");
    ROBOT::shell().add(apply_pwm, "apply_pwm", "Apply PWM to motors", "HB");

    ROBOT::shell().create_module("state_machine", "Module for state control");
    ROBOT::shell().add(set_state_control, "set_state", "Set the current state of the state machine", "state_machine");

    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::insert_log("Shell started", logType::CMD);
    #endif
    return true;
}

#endif // WRAPPER_H