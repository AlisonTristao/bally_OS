#ifndef ARRAYSENSOR_H
#define ARRAYSENSOR_H

#include <cstdint>
#include <string>

#include "esp_adc/adc_oneshot.h"

class TinyShell;
class Logger;
class RobotSettings;

class ArraySensor {
public:
    // Hardware ceiling: the analog multiplexer has 8 channels (S0..S2
    // select 1 of 8). len (runtime sensor count) is clamped to this range.
    static constexpr uint8_t MAX_LEN = 8;

    ArraySensor(
        uint8_t s0,
        uint8_t s1,
        uint8_t s2,
        uint8_t signal,
        uint8_t len = MAX_LEN
    );

    virtual ~ArraySensor();

    bool calibration_ok();

    bool calibrate(uint8_t n_samples, uint8_t delay_ms);

    std::string calibrate_status();

    float get_line_position();

    std::string debug();

    std::string raw();

    /**
     * @brief Register this sensor's "sensor" shell module commands
     * (calibrate/calibrate_status/position/raw). Owned here instead of the
     * ROBOT composition root so the calibration workflow lives next to the
     * class it operates on.
     * @param settings Read live at call time (cfg.samples/delay_sample), not
     * captured, so a "settings set" takes effect on the next calibrate
     * without re-registering.
     */
    void register_shell_commands(TinyShell& shell, Logger& logger, RobotSettings& settings);

private:
    // Pinos de seleção do multiplexador
    const uint8_t s0;
    const uint8_t s1;
    const uint8_t s2;

    // Saída comum do multiplexador conectada ao ADC
    const uint8_t signal;

    // Número de sensores ativos (1..MAX_LEN), configurável em runtime.
    const uint8_t len_;

    adc_unit_t unit = ADC_UNIT_1;
    adc_channel_t channel = ADC_CHANNEL_0;

    adc_oneshot_unit_handle_t adc_handle = nullptr;

    float lastPosition = 0;

    uint16_t min[MAX_LEN];
    uint16_t max[MAX_LEN];

    void select_channel(uint8_t channel);

    uint16_t read(uint8_t index);

    int16_t normalize(uint16_t value, uint8_t index);
};

#endif // ARRAYSENSOR_H
