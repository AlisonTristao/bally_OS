#ifndef FORMAT_H
#define FORMAT_H

#include <cstddef>
#include <cstdint>
#include <cstdio>

/**
 * @brief Human-readable byte size ("12.34 MB"). Shared by every module that
 * reports storage/memory usage over the shell (SDCard, USBMassStorage,
 * Logger) so the formatting logic and its output stay identical across all
 * of them instead of being copy-pasted into each one.
 */
inline void formatBytes(uint64_t bytes, char* output, size_t capacity) {
    const char* unit = "B";
    double value = static_cast<double>(bytes);

    if (bytes >= (1024ULL * 1024ULL * 1024ULL)) {
        value /= 1024.0 * 1024.0 * 1024.0;
        unit = "GB";
    } else if (bytes >= (1024ULL * 1024ULL)) {
        value /= 1024.0 * 1024.0;
        unit = "MB";
    } else if (bytes >= 1024ULL) {
        value /= 1024.0;
        unit = "kB";
    }

    snprintf(output, capacity, "%.2f %s", value, unit);
}

#endif // FORMAT_H
