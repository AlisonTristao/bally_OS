#ifndef ARRAYSENSOR_H
#define ARRAYSENSOR_H

#include <cstdint>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"

template <uint8_t LEN>
class ArraySensor {
private:
    uint8_t arr[LEN];

    adc_unit_t units[LEN];
    adc_channel_t channels[LEN];

    float lastPosition = 0;

    adc_oneshot_unit_handle_t adc1_handle = nullptr;
    adc_oneshot_unit_handle_t adc2_handle = nullptr;

    uint16_t read(uint8_t index);

    int16_t normalize(uint16_t value, uint8_t index);

    uint16_t min[LEN], max[LEN];

public:
    ArraySensor(const uint8_t* arr);
    virtual ~ArraySensor();

    bool calibration_ok();

    bool calibrate(uint8_t n_samples, uint8_t delay_ms);

    std::string calibrate_status();

    float get_line_position();

    std::string debug();

    std::string raw();
};

template <uint8_t LEN>
ArraySensor<LEN>::ArraySensor(const uint8_t* arr) {
    adc_oneshot_unit_init_cfg_t adc1_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };

    adc_oneshot_unit_init_cfg_t adc2_config = {
        .unit_id = ADC_UNIT_2,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };

    adc_oneshot_new_unit(&adc1_config, &adc1_handle);
    adc_oneshot_new_unit(&adc2_config, &adc2_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    for(uint8_t i = 0; i < LEN; i++) {
        this->arr[i] = arr[i];

        adc_unit_t unit;
        adc_channel_t channel;

        if (adc_oneshot_io_to_channel(this->arr[i], &unit, &channel) == ESP_OK) {
            units[i] = unit;
            channels[i] = channel;

            if (unit == ADC_UNIT_1 && adc1_handle != nullptr) {
                adc_oneshot_config_channel(adc1_handle, channel, &config);
            } else if (unit == ADC_UNIT_2 && adc2_handle != nullptr) {
                adc_oneshot_config_channel(adc2_handle, channel, &config);
            }
        } else {
            units[i] = ADC_UNIT_1;
            channels[i] = ADC_CHANNEL_0;
        }

        min[i] = 4095;
        max[i] = 0;
    }
}

template <uint8_t LEN>
ArraySensor<LEN>::~ArraySensor() {
    if (adc1_handle != nullptr) {
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = nullptr;
    }

    if (adc2_handle != nullptr) {
        adc_oneshot_del_unit(adc2_handle);
        adc2_handle = nullptr;
    }
}

template <uint8_t LEN>
uint16_t ArraySensor<LEN>::read(uint8_t index) {
    if (index >= LEN) return 0;

    int raw_val = 0;
    esp_err_t err = ESP_FAIL;

    if (units[index] == ADC_UNIT_1 && adc1_handle != nullptr) {
        err = adc_oneshot_read(adc1_handle, channels[index], &raw_val);
    } else if (units[index] == ADC_UNIT_2 && adc2_handle != nullptr) {
        err = adc_oneshot_read(adc2_handle, channels[index], &raw_val);
    }

    if (err != ESP_OK) return 0;

    return 4095 - raw_val;
}

template <uint8_t LEN>
int16_t ArraySensor<LEN>::normalize(uint16_t value, uint8_t index) {
    int32_t range = max[index] - min[index];

    if (range <= 0) return 0;

    int32_t norm = ((int32_t)value - (int32_t)min[index]) * 1000 / range;

    if(norm < 0)    return 0;
    if(norm > 1000) return 1000;

    return norm;
}

template <uint8_t LEN>
bool ArraySensor<LEN>::calibration_ok() {
    for(uint8_t i = 0; i < LEN; i++) {
        if((max[i] - min[i]) <= 100) return false;
    }

    return true;
}

template <uint8_t LEN>
bool ArraySensor<LEN>::calibrate(uint8_t n_samples, uint8_t delay_ms) {
    uint16_t value = 0;

    for(uint8_t i = 0; i < n_samples; i++) {
        for(uint8_t j = 0; j < LEN; j++) {
            value = read(j);

            if (value < min[j]) min[j] = value;
            if (value > max[j]) max[j] = value;
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    return calibration_ok();
}

template <uint8_t LEN>
std::string ArraySensor<LEN>::calibrate_status() {
    std::string status;

    for(uint8_t i = 0; i < LEN; i++) {
        status += std::to_string(min[i]) +
                  "-" +
                  std::to_string(max[i]) +
                  "\n";
    }

    return status;
}

template <uint8_t LEN>
float ArraySensor<LEN>::get_line_position() {
    uint32_t value = 0;
    uint32_t measure = 0;
    bool line = false;
    uint16_t val = 0;

    for (uint8_t i = 0; i < LEN; i++) {
        val = normalize(read(i), i);

        value += val * (i + 1) * 1000;
        measure += val;

        if(val > (LEN * 100)) line = true;
    }

    if(line && measure > 0) {
        lastPosition = (float)value / (float)measure;
    } else {
        lastPosition = lastPosition < (LEN * 1000) / 2 ? 1000 : LEN * 1000;
    }

    return lastPosition;
}

template <uint8_t LEN>
std::string ArraySensor<LEN>::debug() {
    std::string status;

    for(uint8_t i = 0; i < LEN; i++) {
        status += std::to_string(read(i)) + "\t";
    }

    return status;
}

template <uint8_t LEN>
std::string ArraySensor<LEN>::raw() {
    std::string status;

    for(uint8_t i = 0; i < LEN; i++) {
        status += std::to_string(read(i)) + "\t";
    }

    return status;
}

#endif