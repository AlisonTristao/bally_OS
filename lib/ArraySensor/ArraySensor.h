#ifndef ARRAYSENSOR_H
#define ARRAYSENSOR_H

#include <cstdint>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"

template <uint8_t LEN>
class ArraySensor {
private:
    static_assert(LEN > 0, "ArraySensor precisa ter pelo menos um sensor");
    static_assert(LEN <= 8, "O multiplexador possui no maximo 8 canais");

    // Pinos de seleção do multiplexador
    const uint8_t s0;
    const uint8_t s1;
    const uint8_t s2;

    // Saída comum do multiplexador conectada ao ADC
    const uint8_t signal;

    adc_unit_t unit = ADC_UNIT_1;
    adc_channel_t channel = ADC_CHANNEL_0;

    adc_oneshot_unit_handle_t adc_handle = nullptr;

    float lastPosition = 0;

    uint16_t min[LEN];
    uint16_t max[LEN];

    void select_channel(uint8_t channel);

    uint16_t read(uint8_t index);

    int16_t normalize(uint16_t value, uint8_t index);

public:
    ArraySensor(
        uint8_t s0,
        uint8_t s1,
        uint8_t s2,
        uint8_t signal
    );

    virtual ~ArraySensor();

    bool calibration_ok();

    bool calibrate(uint8_t n_samples, uint8_t delay_ms);

    std::string calibrate_status();

    float get_line_position();

    std::string debug();

    std::string raw();
};

// ============================================================================
// Constructor
// ============================================================================

template <uint8_t LEN>
ArraySensor<LEN>::ArraySensor(
    uint8_t s0,
    uint8_t s1,
    uint8_t s2,
    uint8_t signal
)
    : s0(s0),
      s1(s1),
      s2(s2),
      signal(signal)
{
    // Configura os pinos de seleção do multiplexador
    gpio_reset_pin(static_cast<gpio_num_t>(s0));
    gpio_set_direction(
        static_cast<gpio_num_t>(s0),
        GPIO_MODE_OUTPUT
    );
    gpio_set_level(static_cast<gpio_num_t>(s0), 0);

    gpio_reset_pin(static_cast<gpio_num_t>(s1));
    gpio_set_direction(
        static_cast<gpio_num_t>(s1),
        GPIO_MODE_OUTPUT
    );
    gpio_set_level(static_cast<gpio_num_t>(s1), 0);

    gpio_reset_pin(static_cast<gpio_num_t>(s2));
    gpio_set_direction(
        static_cast<gpio_num_t>(s2),
        GPIO_MODE_OUTPUT
    );
    gpio_set_level(static_cast<gpio_num_t>(s2), 0);

    /*
     * Existe somente uma entrada ADC.
     * Todos os sensores chegam ao ESP32 pelo pino SIG.
     */
    if (
        adc_oneshot_io_to_channel(
            signal,
            &unit,
            &channel
        ) == ESP_OK
    ) {
        adc_oneshot_unit_init_cfg_t adc_config = {};

        adc_config.unit_id = unit;
        adc_config.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
        adc_config.ulp_mode = ADC_ULP_MODE_DISABLE;

        if (
            adc_oneshot_new_unit(
                &adc_config,
                &adc_handle
            ) == ESP_OK
        ) {
            adc_oneshot_chan_cfg_t channel_config = {};

            channel_config.atten = ADC_ATTEN_DB_12;
            channel_config.bitwidth = ADC_BITWIDTH_12;

            if (
                adc_oneshot_config_channel(
                    adc_handle,
                    channel,
                    &channel_config
                ) != ESP_OK
            ) {
                adc_oneshot_del_unit(adc_handle);
                adc_handle = nullptr;
            }
        }
    }

    for (uint8_t i = 0; i < LEN; i++) {
        min[i] = 4095;
        max[i] = 0;
    }
}

// ============================================================================
// Destructor
// ============================================================================

template <uint8_t LEN>
ArraySensor<LEN>::~ArraySensor()
{
    if (adc_handle != nullptr) {
        adc_oneshot_del_unit(adc_handle);
        adc_handle = nullptr;
    }
}

// ============================================================================
// Seleção do canal do multiplexador
// ============================================================================

template <uint8_t LEN>
void ArraySensor<LEN>::select_channel(uint8_t mux_channel)
{
    /*
     * Seleção dos canais X0 até X7:
     *
     * Canal   S2 S1 S0
     * X0       0  0  0
     * X1       0  0  1
     * X2       0  1  0
     * X3       0  1  1
     * X4       1  0  0
     * X5       1  0  1
     * X6       1  1  0
     * X7       1  1  1
     */

    gpio_set_level(
        static_cast<gpio_num_t>(s0),
        mux_channel & 0x01
    );

    gpio_set_level(
        static_cast<gpio_num_t>(s1),
        (mux_channel >> 1) & 0x01
    );

    gpio_set_level(
        static_cast<gpio_num_t>(s2),
        (mux_channel >> 2) & 0x01
    );

    // Tempo para a saída analógica do MUX estabilizar
    esp_rom_delay_us(5);
}

// ============================================================================
// Leitura
// ============================================================================

template <uint8_t LEN>
uint16_t ArraySensor<LEN>::read(uint8_t index)
{
    if (index >= LEN || adc_handle == nullptr) {
        return 0;
    }

    /*
     * O índice já representa diretamente o canal:
     *
     * index 0 -> sensor 1 -> X0
     * index 1 -> sensor 2 -> X1
     * ...
     * index 7 -> sensor 8 -> X7
     */
    select_channel(index);

    int raw_val = 0;

    esp_err_t err = adc_oneshot_read(
        adc_handle,
        channel,
        &raw_val
    );

    if (err != ESP_OK) {
        return 0;
    }

    return 4095 - raw_val;
}

// ============================================================================
// Normalização
// ============================================================================

template <uint8_t LEN>
int16_t ArraySensor<LEN>::normalize(
    uint16_t value,
    uint8_t index
) {
    int32_t range = max[index] - min[index];

    if (range <= 0) return 0;

    int32_t norm =
        (
            static_cast<int32_t>(value) -
            static_cast<int32_t>(min[index])
        ) * 1000 / range;

    if (norm < 0) return 0;
    if (norm > 1000) return 1000;

    return norm;
}

// ============================================================================
// Verificação da calibração
// ============================================================================

template <uint8_t LEN>
bool ArraySensor<LEN>::calibration_ok()
{
    for (uint8_t i = 0; i < LEN; i++) {
        if ((max[i] - min[i]) <= 100) {
            return false;
        }
    }

    return true;
}

// ============================================================================
// Calibração
// ============================================================================

template <uint8_t LEN>
bool ArraySensor<LEN>::calibrate(
    uint8_t n_samples,
    uint8_t delay_ms
) {
    uint16_t value = 0;

    for (uint8_t i = 0; i < n_samples; i++) {
        for (uint8_t j = 0; j < LEN; j++) {
            value = read(j);

            if (value < min[j]) {
                min[j] = value;
            }

            if (value > max[j]) {
                max[j] = value;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    return calibration_ok();
}

// ============================================================================
// Estado da calibração
// ============================================================================

template <uint8_t LEN>
std::string ArraySensor<LEN>::calibrate_status()
{
    std::string status;

    for (uint8_t i = 0; i < LEN; i++) {
        status += std::to_string(min[i]) +
                  "-" +
                  std::to_string(max[i]) +
                  "\n";
    }

    return status;
}

// ============================================================================
// Posição da linha
// ============================================================================

template <uint8_t LEN>
float ArraySensor<LEN>::get_line_position()
{
    uint32_t value = 0;
    uint32_t measure = 0;

    bool line = false;

    uint16_t val = 0;

    for (uint8_t i = 0; i < LEN; i++) {
        val = normalize(read(i), i);

        value += val * (i + 1) * 1000;
        measure += val;

        if (val > (LEN * 100)) {
            line = true;
        }
    }

    if (line && measure > 0) {
        lastPosition =
            static_cast<float>(value) /
            static_cast<float>(measure);
    } else {
        lastPosition =
            lastPosition < (LEN * 1000) / 2
                ? 1000
                : LEN * 1000;
    }

    return lastPosition;
}

// ============================================================================
// Debug
// ============================================================================

template <uint8_t LEN>
std::string ArraySensor<LEN>::debug()
{
    std::string status;

    for (uint8_t i = 0; i < LEN; i++) {
        status += std::to_string(read(i)) + "\t";
    }

    return status;
}

// ============================================================================
// Leituras brutas
// ============================================================================

template <uint8_t LEN>
std::string ArraySensor<LEN>::raw()
{
    std::string status;

    for (uint8_t i = 0; i < LEN; i++) {
        status += std::to_string(read(i)) + "\t";
    }

    return status;
}

#endif // ARRAYSENSOR_H