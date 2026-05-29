#include "ArraySensor.h"
#include <Compat.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <cstdio>

static const char* TAG = "ArraySensor";

ArraySensor::ArraySensor(uint8_t* arr, uint8_t len){
    this->len = len;
    this->arr = new uint8_t[len];
    for(uint8_t i = 0; i < len; i++)
        this->arr[i] = arr[i];
    // init arrays
    min = new uint16_t[len];
    max = new uint16_t[len];
    for(uint8_t i = 0; i < len; i++){
        min[i] = 4095;
        max[i] = 0;
    }

    adc_channels_.reserve(len);
    adc_unit_t detected_unit = ADC_UNIT_1;
    bool unit_set = false;

    for (uint8_t i = 0; i < len; i++) {
        adc_unit_t unit;
        adc_channel_t channel;
        esp_err_t err = adc_oneshot_io_to_channel(static_cast<gpio_num_t>(this->arr[i]), &unit, &channel);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ADC map failed for GPIO %u: %s", this->arr[i], esp_err_to_name(err));
            continue;
        }

        if (!unit_set) {
            detected_unit = unit;
            unit_set = true;
        }

        if (unit != detected_unit) {
            ESP_LOGE(TAG, "Mixed ADC units are not supported (GPIO %u)", this->arr[i]);
            continue;
        }

        adc_channels_.push_back(channel);
    }

    if (unit_set && adc_channels_.size() == len) {
        adc_unit_ = detected_unit;
        adc_oneshot_unit_init_cfg_t init_cfg = {};
        init_cfg.unit_id = adc_unit_;
        init_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;

        if (adc_oneshot_new_unit(&init_cfg, &adc_handle_) == ESP_OK) {
            adc_oneshot_chan_cfg_t chan_cfg = {};
            chan_cfg.bitwidth = ADC_BITWIDTH_12;
            chan_cfg.atten = ADC_ATTEN_DB_11;

            bool ok = true;
            for (auto channel : adc_channels_) {
                if (adc_oneshot_config_channel(adc_handle_, channel, &chan_cfg) != ESP_OK) {
                    ok = false;
                    break;
                }
            }
            adc_ready_ = ok;
        }
    }

    if (!adc_ready_) {
        ESP_LOGW(TAG, "ADC oneshot not ready; sensor reads will return 0");
    }
}

ArraySensor::~ArraySensor(){
    if (adc_handle_ != nullptr) {
        adc_oneshot_del_unit(adc_handle_);
        adc_handle_ = nullptr;
    }

    delete[] arr;
    delete[] min;
    delete[] max;
}

uint16_t ArraySensor::read(uint8_t index){
    if (!adc_ready_ || index >= adc_channels_.size())
        return 0;

    int raw = 0;
    if (adc_oneshot_read(adc_handle_, adc_channels_[index], &raw) != ESP_OK)
        return 0;

    if (raw < 0) raw = 0;
    if (raw > 4095) raw = 4095;

    // if the line is black, invert the value
    return static_cast<uint16_t>(4095 - raw);
}

int16_t ArraySensor::normalize(uint16_t value, uint8_t index){
    // normalize the value
    int16_t norm = map(value, min[index], max[index], 0, 1000);

    // sature the value
    if(norm < 0)        return 0;
    if(norm > 1000)     return 1000;
    return norm;
}

bool ArraySensor::calibration_ok(){
    // check if the calibration is ok
    for(uint8_t i = 0; i < len; i++)
        if((max[i] - min[i]) <= 100) return false;
    return true;
}

bool ArraySensor::calibrate(uint8_t n_samples, uint8_t delay_ms){
    // calibrate the sensors
    uint16_t value = 0;
    for(uint8_t i = 0; i < n_samples; i++){
        for(uint8_t j = 0; j < len; j++){
            value = read(j);
            // line white or black
            if (value < min[j]) min[j] = value;
            if (value > max[j]) max[j] = value;
        }
        delay(delay_ms);
    }

    // check if the calibration is ok
    return calibration_ok();
}

std::string ArraySensor::calibrate_status(){
    // return the calibration status
    std::string status;
    for(uint8_t i = 0; i < len; i++){
        status += std::to_string(min[i]);
        status += "-";
        status += std::to_string(max[i]);
        status += "\n";
    }

    return status;
}

double ArraySensor::read_line(){
    double value = 0, measure = 0;
    bool line = false;
    uint16_t val = 0;
    // read the sensor and normalize the value
    for (uint8_t i = 0; i < len; i++){
        val = normalize(read(i), i);
        value += val * (i+1) * 1000;
        measure += val;
        // check if the line is detected
        if(val > (len*100)) line = true;
    }

    // atualize last position
    if(line)    lastPosition = value/measure;
    // saturate last position
    else        lastPosition = lastPosition < (len*1000)/2 ? 1000 : len*1000;
    
    // return the last position
    return lastPosition;
}

std::string ArraySensor::debug(){
    std::string status;
    for(uint8_t i = 0; i < len; i++) {
        status += std::to_string(read(i));
        status += "\t";
    }
    return status;
}

std::string ArraySensor::raw(){
    std::string status;
    for(uint8_t i = 0; i < len; i++) {
        status += std::to_string(read(i));
        status += "\t";
    }
    return status;
}

void ArraySensor::saveCalibration() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("calibration", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return;
    }

    char key[10]; // Buffer to store the keys
    for (uint8_t i = 0; i < len; i++) {
        snprintf(key, sizeof(key), "min%u", i);
        nvs_set_u16(handle, key, min[i]);

        snprintf(key, sizeof(key), "max%u", i);
        nvs_set_u16(handle, key, max[i]);
    }

    nvs_commit(handle);
    nvs_close(handle);
}

bool ArraySensor::loadCalibration() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("calibration", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return false;
    }

    char key[10]; // Buffer to store the keys
    bool ok = true;
    for (uint8_t i = 0; i < len; i++) {
        uint16_t min_val = 0xFFFF;
        uint16_t max_val = 0x0000;

        snprintf(key, sizeof(key), "min%u", i);
        if (nvs_get_u16(handle, key, &min_val) != ESP_OK)
            ok = false;

        snprintf(key, sizeof(key), "max%u", i);
        if (nvs_get_u16(handle, key, &max_val) != ESP_OK)
            ok = false;

        min[i] = min_val;
        max[i] = max_val;
    }

    nvs_close(handle);

    if (!ok)
        return false;

    // check if the calibration is valid
    for (uint8_t i = 0; i < len; i++) {
        if (min[i] == 0xFFFF || max[i] == 0x0000) {
            return false; // calibration is not valid
        }
    }

    return true;
}