#include "SystemMonitor.h"
#include <stdio.h>
#include <stdlib.h>

SystemMonitor::SystemMonitor() {
    temp_sensor = NULL;
    is_initialized = false;
    output_cb = nullptr;
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

std::string SystemMonitor::getMemoryStats() {
    char buffer[256];
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_free_heap = esp_get_minimum_free_heap_size();
    uint32_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    uint32_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    // separating into 3 aligned lines and converting bytes to kb
    snprintf(buffer, sizeof(buffer), 
             "ram total   : %7.2f kb (min free: %.2f kb)\n"
             "ram internal: %7.2f kb\n"
             "ram psram   : %7.2f kb\n", 
             free_heap / 1024.0f, min_free_heap / 1024.0f, 
             internal_free / 1024.0f, psram_free / 1024.0f);
    
    return std::string(buffer);
}

std::string SystemMonitor::getTaskStats() {
    uint32_t dummy_runtime;
    UBaseType_t array_size = uxTaskGetNumberOfTasks();
    TaskStatus_t *status_array = (TaskStatus_t *)malloc(array_size * sizeof(TaskStatus_t));
    
    if (status_array == nullptr) {
        return "error: failed to allocate memory for tasks\n";
    }

    // gets the raw state of all freertos tasks
    array_size = uxTaskGetSystemState(status_array, array_size, &dummy_runtime);

    std::string output = "task name            state    prio      min free \n";
    output +=            "------------------------------------------------\n";
    
    for (UBaseType_t i = 0; i < array_size; i++) {
        char state_char;
        switch (status_array[i].eCurrentState) {
            case eRunning:   state_char = 'R'; break; // running
            case eReady:     state_char = 'r'; break; // ready
            case eBlocked:   state_char = 'B'; break; // blocked
            case eSuspended: state_char = 'S'; break; // suspended
            case eDeleted:   state_char = 'D'; break; // deleted
            default:         state_char = '?'; break;
        }

        char line[96];
        // formatting as %7.2f to align the floating point numbers
        snprintf(line, sizeof(line), "%-20s %-8c %-7u %7.2f kb\n", 
                 status_array[i].pcTaskName, state_char, 
                 (unsigned int)status_array[i].uxCurrentPriority,
                 status_array[i].usStackHighWaterMark / 1024.0f);
        output += line;
    }

    free(status_array);
    return output;
}

std::string SystemMonitor::getFullReport() {
    std::string report = "";
    
    char temp_buffer[128];
    // 52 '=' characters to exactly match the tasks table border
    snprintf(temp_buffer, sizeof(temp_buffer), 
             "------------------------------------------------\n"
             "core temp   : %7.2f °C\n", 
             getCoreTemperature());
    
    report += temp_buffer;
    report += getMemoryStats();
    report += getTaskStats();
    report += "------------------------------------------------\n";
    
    return report;
}

void SystemMonitor::report() {
    // dispatches the full string to the callback
    dispatch(getFullReport());
}