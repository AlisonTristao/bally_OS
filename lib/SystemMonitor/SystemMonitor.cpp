#include "SystemMonitor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>

SystemMonitor::SystemMonitor() {
    temp_sensor = NULL;
    is_initialized = false;
    output_cb = nullptr;
    logger_index_cb = nullptr;
    last_total_runtime = 0;
    core0_load = 0.0f;
    core1_load = 0.0f;
    
    task_records.reserve(30); 
}

void SystemMonitor::begin() {
    if (is_initialized) return;

    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(25, 55);
    temperature_sensor_install(&temp_sensor_config, &temp_sensor);
    temperature_sensor_enable(temp_sensor);

    is_initialized = true;
}

void SystemMonitor::setOutputCallback(MonitorCallback cb) {
    output_cb = cb;
}

void SystemMonitor::setLoggerCallback(GetLoggerIndexCallback cb) {
    logger_index_cb = cb;
}

void SystemMonitor::dispatch(const std::string& data) {
    if (output_cb) {
        output_cb(data);
    }
}

void SystemMonitor::update() {
    uint32_t total_runtime;
    UBaseType_t array_size = uxTaskGetNumberOfTasks();
    TaskStatus_t *status_array = (TaskStatus_t *)malloc(array_size * sizeof(TaskStatus_t));
    
    if (status_array == nullptr) return;

    array_size = uxTaskGetSystemState(status_array, array_size, &total_runtime);

    uint32_t delta_total = total_runtime - last_total_runtime;
    last_total_runtime = total_runtime;

    for (auto& record : task_records) {
        record.active_this_cycle = false;
    }

    core0_load = 100.0f;
    core1_load = 100.0f; 

    for (UBaseType_t i = 0; i < array_size; i++) {
        TaskHandle_t handle = status_array[i].xHandle;
        uint32_t current_runtime = status_array[i].ulRunTimeCounter;

        auto it = std::find_if(task_records.begin(), task_records.end(),
                               [handle](const TaskRecord& r) { return r.handle == handle; });

        if (it == task_records.end()) {
            TaskRecord new_record;
            new_record.handle = handle;
            strncpy(new_record.name, status_array[i].pcTaskName, sizeof(new_record.name) - 1);
            new_record.name[sizeof(new_record.name) - 1] = '\0';
            new_record.last_runtime = current_runtime;
            new_record.current_cpu_load = 0.0f;
            new_record.active_this_cycle = true;
            task_records.push_back(new_record);
            it = task_records.end() - 1; 
        } else {
            it->active_this_cycle = true;
        }

        it->priority = status_array[i].uxCurrentPriority;
        it->high_water_mark = status_array[i].usStackHighWaterMark;
        it->core_id = xTaskGetCoreID(handle);

        // Calcula a CPU Load desta task
        uint32_t delta_task = current_runtime - it->last_runtime;
        it->last_runtime = current_runtime;

        float percent = 0.0f;
        if (delta_total > 0) {
            percent = (static_cast<float>(delta_task) * 100.0f) / delta_total;
        }
        if (percent > 100.0f) percent = 100.0f;
        it->current_cpu_load = percent;

        if (strcmp(it->name, "IDLE0") == 0) {
            core0_load = 100.0f - percent;
            if (core0_load < 0.0f) core0_load = 0.0f;
        } else if (strcmp(it->name, "IDLE1") == 0) {
            core1_load = 100.0f - percent;
            if (core1_load < 0.0f) core1_load = 0.0f;
        }
    }

    // Garbage Collection:
    task_records.erase(
        std::remove_if(task_records.begin(), task_records.end(),
                       [](const TaskRecord& r) { return !r.active_this_cycle; }),
        task_records.end()
    );

    free(status_array);
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
    uint64_t uptime_us = esp_timer_get_time();
    uint32_t uptime_s = uptime_us / 1000000;
    
    uint32_t h = uptime_s / 3600;
    uint32_t m = (uptime_s % 3600) / 60;
    uint32_t s = uptime_s % 60;
    
    snprintf(buffer, sizeof(buffer), "uptime      : %02uh %02um %02us\n", (unsigned int)h, (unsigned int)m, (unsigned int)s);
    return std::string(buffer);
}

std::string SystemMonitor::getMemoryStats() {
    char buffer[256];
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    uint32_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    auto format_mem = [](uint32_t bytes) -> std::string {
        float kb = bytes / 1024.0f;
        char b[32];
        if (kb >= 1024.0f) {
            snprintf(b, sizeof(b), "%7.2f mb", kb / 1024.0f);
        } else {
            snprintf(b, sizeof(b), "%7.2f kb", kb);
        }
        return std::string(b);
    };

    snprintf(buffer, sizeof(buffer), 
             "total ram   : %s\n"
             "sram        : %s\n"
             "psram       : %s\n", 
             format_mem(free_heap).c_str(), 
             format_mem(internal_free).c_str(), 
             format_mem(psram_free).c_str());
    
    return std::string(buffer);
}

std::string SystemMonitor::getTaskStats() {
    std::string output;
    output.reserve(task_records.size() * 100 + 150); 

    output += "--------------------- tasks ---------------------\n";
    output += "task name        prio core      cpu     free heap\n";
    output += "-\n";
    
    for (const auto& record : task_records) {
        char core_str[8];
        if (record.core_id == 0) {
            snprintf(core_str, sizeof(core_str), "pro");
        } else if (record.core_id == 1) {
            snprintf(core_str, sizeof(core_str), "app");
        } else {
            snprintf(core_str, sizeof(core_str), "any");
        }

        char line[96];
        snprintf(line, sizeof(line), "%-16s %-4u %-6s %6.2f%% %8.2f kb\n", 
                 record.name, 
                 (unsigned int)record.priority,
                 core_str, 
                 record.current_cpu_load,
                 record.high_water_mark / 1024.0f);
        output += line;
    }

    return output;
}

std::string SystemMonitor::getFullReport() {
    std::string report = "";
    report.reserve(1024);
    
    char core_buffer[512];

    float write_pct = 0.0f;
    if (logger_index_cb) {
        write_pct = logger_index_cb();
    }
    
    snprintf(core_buffer, sizeof(core_buffer), 
             "================= Monitor Stats =================\n"
             "%s"
             "---------------------- CPU ----------------------\n"
             "tick rate   : %7d Hz\n"
             "core rate   : %7d MHz\n"
             "core temp   : %7.2f °C\n"
             "core PRO    : %7.2f %%\n"
             "core APP    : %7.2f %%\n"
             "-------------------- memory --------------------\n"
             "logger      : %7.2f %%\n", 
             getUptime().c_str(), 
             configTICK_RATE_HZ, 
             CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, 
             getCoreTemperature(), 
             core0_load, 
             core1_load,
             write_pct);
    
    report += core_buffer;
    report += getMemoryStats();
    report += getTaskStats();
    report += "=================================================\n";
    
    return report;
}

void SystemMonitor::report() {
    dispatch(getFullReport());
}