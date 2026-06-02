#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "driver/temperature_sensor.h"
#include <string>
#include <functional>

// defines the callback type: receives a constant c++ string by reference
using MonitorCallback = std::function<void(const std::string&)>;

class SystemMonitor {
private:
    temperature_sensor_handle_t temp_sensor;
    bool is_initialized;
    MonitorCallback output_cb;

    // internal function to dispatch the string to the callback
    void dispatch(const std::string& data);

public:
    SystemMonitor();
    
    // initializes hardware (like the temperature sensor)
    void begin();
    
    // registers the function that will consume and send the data
    void setCallback(MonitorCallback cb);

    // returns core temperature in celsius
    float getCoreTemperature();
    
    // returns formatted memory statistics in kilobytes
    std::string getMemoryStats();
    
    // returns formatted task status table in kilobytes
    std::string getTaskStats();
    
    // returns the entire consolidated report
    std::string getFullReport();

    // dispatches the full report to the registered callback
    void report();
};