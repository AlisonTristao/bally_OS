#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/temperature_sensor.h"
#include <string>
#include <functional>
#include <map>

// defines the callback type: receives a constant c++ string by reference
using MonitorCallback = std::function<void(const std::string&)>;

class SystemMonitor {
private:
    temperature_sensor_handle_t temp_sensor;
    bool is_initialized;
    MonitorCallback output_cb;

    // variables for cpu delta calculation
    uint32_t last_total_runtime;
    std::map<TaskHandle_t, uint32_t> last_task_runtimes;

    // stores the global load per core
    float core0_load;
    float core1_load;

    // internal function to dispatch the string to the callback
    void dispatch(const std::string& data);

public:
    SystemMonitor();
    
    // initializes hardware (like the temperature sensor)
    void begin();
    
    // registers the function that will consume and send the data
    void setCallback(MonitorCallback cb);

    // system metrics
    float getCoreTemperature();
    std::string getUptime();
    std::string getMemoryStats();
    
    // updates the core loads internally and returns the formatted table
    std::string getTaskStats();
    
    // returns the entire consolidated report
    std::string getFullReport();

    // dispatches the full report to the registered callback
    void report();
};