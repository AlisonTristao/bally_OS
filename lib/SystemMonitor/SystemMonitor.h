#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/temperature_sensor.h"
#include <string>
#include <functional>
#include <vector>

using MonitorCallback = std::function<void(const std::string&)>;
using GetLoggerIndexCallback = float (*)();

struct TaskRecord {
    TaskHandle_t handle;
    char name[16];
    uint32_t last_runtime;
    float current_cpu_load;
    UBaseType_t priority;
    BaseType_t core_id;
    uint32_t high_water_mark;
    bool active_this_cycle; // (Garbage Collection)
};

class SystemMonitor {
private:
    temperature_sensor_handle_t temp_sensor;
    bool is_initialized;
    MonitorCallback output_cb;
    GetLoggerIndexCallback logger_index_cb;

    uint32_t last_total_runtime;
    std::vector<TaskRecord> task_records; 

    float core0_load;
    float core1_load;

    void dispatch(const std::string& data);

public:
    SystemMonitor();
    
    void begin();
    
    void setOutputCallback(MonitorCallback cb);
    void setLoggerCallback(GetLoggerIndexCallback cb);
    void update(); 

    float getCoreTemperature();
    std::string getUptime();
    std::string getMemoryStats();
    
    std::string getTaskStats();
    
    std::string getFullReport();

    void report();
};