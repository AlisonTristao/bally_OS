#ifndef WRAPPER_H
#define WRAPPER_H

//   SYSTEM  
#include <Arduino.h>

//   UTILITIES  
#include <Flags.h>
#include <Logger.h>
#include <StateMachine.h>
#include <StaticObjects.h>

//   EXTERNAL LIBRARIES  
#include <TinyShell.h>

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
        ROBOT::logger.insert_log("Type aliases used by shell:" + String("\n") +
                                "u8  => uint8_t" + String("\n") +
                                "u16 => uint16_t" + String("\n") +
                                "u32 => uint32_t" + String("\n") +
                                "u64 => uint64_t" + String("\n") +
                                "i8  => int8_t" + String("\n") +
                                "i16 => int16_t" + String("\n") +
                                "i32 => int32_t" + String("\n") +
                                "i64 => int64_t" + String("\n") +
                                "f32 => float" + String("\n") +
                                "f64 => double" + String("\n") +
                                "b   => bool" + String("\n") +
                                "str => string", logType::CMD);
    #endif
    return RESULT_OK;
}

uint8_t testPacket() {
    // envia um texto grande para testar o envio de varios pacotes
    // o texto é uma citacao de "Dom Casmurro", de Machado de Assis
    const char* long_text =
        "Uma noite, ao chegar a casa,\n"
        "encontrei um bilhete de minha mãe, dizendo que ela e meu pai\n"
        "haviam saído para jantar, e que eu deveria me comportar.\n"
        "Fiquei sozinho em casa, e a solidão me envolveu como um manto.\n"
        "Sentei-me à janela, olhando para as estrelas,\n"
        "e pensei em tudo o que havia acontecido em minha vida até então.\n"
        "As lembranças de minha infância, de meus pais, de minha escola,\n"
        "de meus amigos, tudo isso passou diante de meus olhos como um filme.\n"
        "E então, percebi que a vida era como um rio, que corria sem parar,\n"
        "levando-nos para lugares desconhecidos, e que nós éramos como folhas,\n"
        "flutuando na correnteza, sem saber onde iríamos parar. Foi uma\n"
        "noite de reflexão profunda, e eu me senti mais maduro, mais consciente\n"
        "de mim mesmo e do mundo ao meu redor. E assim, adormeci, com a cabeça\n"
        "cheia de pensamentos e o coração cheio de emoções, sabendo que a vida\n"
        "continuaria a me surpreender, a me desafiar, e que eu teria que enfrentar\n"
        "tudo isso com coragem e determinação.";

    // When info logs are enabled, push to logger to validate its packetization path.
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log(long_text, logType::INFO);
        return RESULT_OK;
    #endif
}

// reset the robot using the shell
uint8_t reset_robot() {
    #if defined(LOG_ALL) || defined(LOG_CMD)
        ROBOT::logger.insert_log("Resetting robot in 3s...", logType::CMD);
    #endif

    // Wait for 3 seconds to allow the log message to be sent before restarting
    vTaskDelay(3000/portTICK_PERIOD_MS);

    ESP.restart();
    return RESULT_OK; // this line will never be reached, but it's here to satisfy the return type
}

bool start_shell_wrappers() {
    ROBOT::shell.create_module("help", "Module for help and information");
    ROBOT::shell.add(wrapper_h, "h", "List all modules", "help");
    ROBOT::shell.add(wrapper_types_help, "types", "Show type aliases (u8, i32, str...)", "help");

    ROBOT::shell.create_module("robot", "Module for robot control commands");
    ROBOT::shell.add(testPacket, "test_packet", "Send a long test packet to evaluate multi-packet handling", "robot");
    ROBOT::shell.add(set_led, "set_led", "Set LED color and brightness", "robot");
    ROBOT::shell.add(led_off, "led_off", "Turn off the LED", "robot");
    ROBOT::shell.add(triggerVirtualButton, "btn", "Virtually trigger a button", "robot");
    ROBOT::shell.add(triggerVirtualSideSensor, "ssr", "Virtually trigger a side sensor", "robot");
    ROBOT::shell.add(reset_robot, "reset", "Reset the robot", "robot");

    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("Shell started", logType::CMD);
    #endif
    return true;
}

#endif // WRAPPER_H