#include "SystemMonitor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

SystemMonitor::SystemMonitor() {
    temp_sensor = NULL;
    is_initialized = false;
    output_cb = nullptr;
    last_total_runtime = 0;
    core0_load = 0.0f;
    core1_load = 0.0f;
}

void SystemMonitor::begin() {
    if (is_initialized) return;

    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
    temperature_sensor_install(&temp_sensor_config, &temp_sensor);
    temperature_sensor_enable(temp_sensor);

    is_initialized = true;
}

void SystemMonitor::setCallback(MonitorCallback cb) {
    output_cb = cb;
}

void SystemMonitor::dispatch(const std::string& data) {
    if (output_cb) {
        output_cb(data);
    }
}

float SystemMonitor::getCoreTemperature() {
    if (!is_initialized) return 0.0f;
    float tsens_value;
    if (temperature_sensor_get_celsius(temp_sensor, &tsens_value) == ESP_OK) {
        return tsens_value;
    }
    return 0.0f;
}

std::string SystemMonitor::getUptime() {
    char buffer[64];
    
    // gets microseconds since boot and converts to seconds
    uint64_t uptime_us = esp_timer_get_time();
    uint32_t uptime_s = uptime_us / 1000000;
    
    // calculates hours, minutes and seconds
    uint32_t h = uptime_s / 3600;
    uint32_t m = (uptime_s % 3600) / 60;
    uint32_t s = uptime_s % 60;
    
    snprintf(buffer, sizeof(buffer), "uptime      : %02uh %02um %02us\n", (unsigned int)h, (unsigned int)m, (unsigned int)s);
    return std::string(buffer);
}

std::string SystemMonitor::getMemoryStats() {
    char buffer[128];
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    uint32_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    // clean alignment without the min free variable
    snprintf(buffer, sizeof(buffer), 
             "free ram    : %7.2f kb\n"
             "internal ram: %7.2f kb\n"
             "psram       : %7.2f kb\n", 
             free_heap / 1024.0f, 
             internal_free / 1024.0f, psram_free / 1024.0f);
    
    return std::string(buffer);
}

std::string SystemMonitor::getTaskStats() {
    uint32_t total_runtime;
    UBaseType_t array_size = uxTaskGetNumberOfTasks();
    TaskStatus_t *status_array = (TaskStatus_t *)malloc(array_size * sizeof(TaskStatus_t));
    
    if (status_array == nullptr) {
        return "error: failed to allocate memory for tasks\n";
    }

    array_size = uxTaskGetSystemState(status_array, array_size, &total_runtime);

    uint32_t delta_total = total_runtime - last_total_runtime;
    last_total_runtime = total_runtime;

    // clean header with exactly 50 characters
    std::string output = "task name        prio core      cpu   free stack\n";
    output +=            "--------------------------------------------------\n";
    
    for (UBaseType_t i = 0; i < array_size; i++) {
        TaskHandle_t handle = status_array[i].xHandle;
        uint32_t current_runtime = status_array[i].ulRunTimeCounter;

        if (last_task_runtimes.find(handle) == last_task_runtimes.end()) {
            last_task_runtimes[handle] = current_runtime;
        }

        uint32_t delta_task = current_runtime - last_task_runtimes[handle];
        last_task_runtimes[handle] = current_runtime;

        float percent = 0.0f;
        if (delta_total > 0) {
            percent = (static_cast<float>(delta_task) * 100.0f) / delta_total;
        }
        if (percent > 100.0f) percent = 100.0f; 

        // saves global load by looking at the idle tasks
        if (strcmp(status_array[i].pcTaskName, "IDLE0") == 0) {
            core0_load = 100.0f - percent;
            if (core0_load < 0.0f) core0_load = 0.0f;
        } else if (strcmp(status_array[i].pcTaskName, "IDLE1") == 0) {
            core1_load = 100.0f - percent;
            if (core1_load < 0.0f) core1_load = 0.0f;
        }

        // checks core affinity using esp-idf v5+ api
        char core_str[8];
        BaseType_t core_id = xTaskGetCoreID(handle);
        
        if (core_id == 0) {
            snprintf(core_str, sizeof(core_str), "0");
        } else if (core_id == 1) {
            snprintf(core_str, sizeof(core_str), "1");
        } else {
            snprintf(core_str, sizeof(core_str), "Any"); // tasks with no fixed core
        }

        char line[96];
        // clean print: name(16), priority(4), core(6), cpu(6) and memory(8)
        snprintf(line, sizeof(line), "%-16s %-4u %-6s %6.2f%% %8.2f kb\n", 
                 status_array[i].pcTaskName, 
                 (unsigned int)status_array[i].uxCurrentPriority,
                 core_str, 
                 percent,
                 status_array[i].usStackHighWaterMark / 1024.0f);
        output += line;
    }

    free(status_array);
    return output;
}

std::string SystemMonitor::getFullReport() {
    // we run the task stats first so it calculates the core0 and core1 loads
    std::string tasks_str = getTaskStats(); 
    
    std::string report = "";
    char temp_buffer[192];
    
    // aligned to match perfectly with the 50 characters of the table
    snprintf(temp_buffer, sizeof(temp_buffer), 
             "==================================================\n"
             "%s"
             "core temp   : %7.2f °C\n"
             "core 0 load : %7.2f %%\n"
             "core 1 load : %7.2f %%\n", 
             getUptime().c_str(), getCoreTemperature(), core0_load, core1_load);
    
    report += temp_buffer;
    report += getMemoryStats();
    report += tasks_str;
    report += "==================================================\n";
    
    return report;
}

void SystemMonitor::report() {
    dispatch(getFullReport());
}