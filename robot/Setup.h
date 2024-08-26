#include <Arduino.h>
#include <Config.h>
#include <Interrupt.h>

bool setup_function() {
    try {
        Serial.begin(921600);

        // init pins direction, settings, i2c communication...
        init_structure();

        // init interruptions in parallel processing
        init_interruptions();

    } catch (const std::exception& e) {
        // LOGGER ERROR
        return false;
    }

    return true;
}