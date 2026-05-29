#ifndef COMPAT_H
#define COMPAT_H

#include <stdint.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

inline uint32_t millis() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

inline void delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif // COMPAT_H
