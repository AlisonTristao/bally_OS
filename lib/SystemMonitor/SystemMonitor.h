#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/temperature_sensor.h"
#include <string>
#include <cstddef>
#include <functional>
#include <vector>

using MonitorCallback = std::function<void(const std::string&)>;
using GetLoggerIndexCallback = float (*)();

class TinyShell;
class Logger;

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

    // update() runs from the normal monitor task and can also be requested by
    // the telemetry/shell tasks. Protect the vector and its delta counters so
    // those low-rate readers never race a realloc/sort.
    StaticSemaphore_t state_mutex_buffer;
    SemaphoreHandle_t state_mutex;

    void dispatch(const std::string& data);
    std::string getTaskStatsUnlocked();
    // Body of getFullReport(); caller must already hold state_mutex.
    std::string getFullReportUnlocked();

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

    // The `sys -health` report (getFullReport) for the BTP UTF8
    // system.monitor telemetry topic. Returns at most max_bytes: an
    // oversized task table is cut on a line boundary with a trailing
    // "... (truncated)" marker.
    std::string getTelemetryReport(std::size_t max_bytes);

    void report();

    /**
     * @brief Register the "sysmon" shell module (temp/uptime/memory/tasks/
     * report), backed by this instance.
     */
    void register_shell_commands(TinyShell& shell, Logger& logger);
};
