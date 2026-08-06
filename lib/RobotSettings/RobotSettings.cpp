#include <RobotSettings.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "driver/gpio.h"

#include <SDCard.h>

namespace {
constexpr size_t kMaxModuleNameLength = 32;
}

// ============================================================================
// Settings table — one row per SettingsData field. This is the single
// source of truth consumed by load/save/get/set/list/reset.
// ============================================================================

#define ENTRY(mod, field, type) \
    { mod, #field, SettingType::type, offsetof(SettingsData, field), sizeof(SettingsData::field) }

const SettingEntry RobotSettings::kTable[] = {
    ENTRY("timers", sample_micros,  U32),
    ENTRY("timers", sysmon_freq_ms, U32),
    ENTRY("timers", delay_flags,    U32),
    ENTRY("timers", timezone,       STRING),

    ENTRY("sensor", len_sensor,   U8),
    ENTRY("sensor", samples,      U8),
    ENTRY("sensor", delay_sample, U8),

    ENTRY("ekf_noise", v_noise,     FLOAT),
    ENTRY("ekf_noise", w_noise,     FLOAT),
    ENTRY("ekf_noise", b_noise,     FLOAT),
    ENTRY("ekf_noise", initial_p,   FLOAT),
    ENTRY("ekf_noise", enc_noise,   FLOAT),
    ENTRY("ekf_noise", accel_noise, FLOAT),
    ENTRY("ekf_noise", gyro_noise,  FLOAT),

    ENTRY("kinematics", encoder_ppr,  FLOAT),
    ENTRY("kinematics", wheel_radius, FLOAT),

    ENTRY("logger", logger_mutex_timeout_ms, U32),
    ENTRY("logger", max_chunks_per_flush,    U32),
    ENTRY("logger", block_size,              U32),

    ENTRY("pins_led", led0, PIN),
    ENTRY("pins_led", led1, PIN),
    ENTRY("pins_led", led2, PIN),
    ENTRY("pins_led", led3, PIN),

    ENTRY("pins_hbridge", ain1,      PIN),
    ENTRY("pins_hbridge", ain2,      PIN),
    ENTRY("pins_hbridge", bin1,      PIN),
    ENTRY("pins_hbridge", bin2,      PIN),
    ENTRY("pins_hbridge", current_a, PIN),
    ENTRY("pins_hbridge", current_b, PIN),

    ENTRY("pins_encoder", enc_a0, PIN),
    ENTRY("pins_encoder", enc_a1, PIN),
    ENTRY("pins_encoder", enc_b0, PIN),
    ENTRY("pins_encoder", enc_b1, PIN),

    ENTRY("pins_button", btn0, PIN),
    ENTRY("pins_button", btn1, PIN),
    ENTRY("pins_button", btn2, PIN),

    ENTRY("pins_side_sensor", left,  PIN),
    ENTRY("pins_side_sensor", right, PIN),

    ENTRY("pins_buzzer", bzr, PIN),

    ENTRY("pins_mux", s0,  PIN),
    ENTRY("pins_mux", s1,  PIN),
    ENTRY("pins_mux", s2,  PIN),
    ENTRY("pins_mux", sig, PIN),

    ENTRY("pins_i2c", sda_pin, PIN),
    ENTRY("pins_i2c", scl_pin, PIN),

    ENTRY("ota", ota_led_step_ms,        U32),
    ENTRY("ota", ota_led_hold_ms,        U32),
    ENTRY("ota", ota_led_fail_hold_ms,   U32),
    ENTRY("ota", ota_connect_timeout_ms, U32),
    ENTRY("ota", ota_retry_scan_ms,      U32),
    ENTRY("ota", espnow_channel,         U8),
    ENTRY("ota", ota_hostname,           STRING),
    ENTRY("ota", ota_instance_name,      STRING),
    ENTRY("ota", ota_password,           STRING),

    ENTRY("error", error_blink_ms, U32),
};

#undef ENTRY

const size_t RobotSettings::kTableCount =
    sizeof(RobotSettings::kTable) / sizeof(RobotSettings::kTable[0]);

// ============================================================================
// Per-type format / parse — the only place that touches raw field bytes.
// ============================================================================

bool RobotSettings::format_entry(const SettingsData& d, const SettingEntry& e,
                                 std::string& out) {
    const uint8_t* base = reinterpret_cast<const uint8_t*>(&d) + e.offset;
    char buf[32];

    switch (e.type) {
        case SettingType::U32:
            snprintf(buf, sizeof(buf), "%lu",
                     static_cast<unsigned long>(*reinterpret_cast<const uint32_t*>(base)));
            break;
        case SettingType::U8:
            snprintf(buf, sizeof(buf), "%u",
                     static_cast<unsigned int>(*reinterpret_cast<const uint8_t*>(base)));
            break;
        case SettingType::FLOAT:
            snprintf(buf, sizeof(buf), "%.6f",
                     static_cast<double>(*reinterpret_cast<const float*>(base)));
            break;
        case SettingType::PIN:
            snprintf(buf, sizeof(buf), "%ld",
                     static_cast<long>(*reinterpret_cast<const int32_t*>(base)));
            break;
        case SettingType::STRING:
            out.assign(reinterpret_cast<const char*>(base));
            return true;
        default:
            return false;
    }

    out.assign(buf);
    return true;
}

bool RobotSettings::parse_into(SettingsData& d, const SettingEntry& e,
                               const char* value) {
    if (value == nullptr || value[0] == '\0') return false;

    uint8_t* base = reinterpret_cast<uint8_t*>(&d) + e.offset;
    char* end = nullptr;

    switch (e.type) {
        case SettingType::U32: {
            unsigned long v = strtoul(value, &end, 0);
            if (end == value || *end != '\0') return false;
            *reinterpret_cast<uint32_t*>(base) = static_cast<uint32_t>(v);
            return true;
        }
        case SettingType::U8: {
            unsigned long v = strtoul(value, &end, 0);
            if (end == value || *end != '\0' || v > 0xFFUL) return false;
            *reinterpret_cast<uint8_t*>(base) = static_cast<uint8_t>(v);
            return true;
        }
        case SettingType::FLOAT: {
            float v = strtof(value, &end);
            if (end == value || *end != '\0') return false;
            *reinterpret_cast<float*>(base) = v;
            return true;
        }
        case SettingType::PIN: {
            long v = strtol(value, &end, 0);
            if (end == value || *end != '\0' ||
                !GPIO_IS_VALID_GPIO(static_cast<gpio_num_t>(v))) {
                return false;
            }
            *reinterpret_cast<int32_t*>(base) = static_cast<int32_t>(v);
            return true;
        }
        case SettingType::STRING: {
            if (strlen(value) >= e.size) return false;
            snprintf(reinterpret_cast<char*>(base), e.size, "%s", value);
            return true;
        }
    }

    return false;
}

const SettingEntry* RobotSettings::find(const char* module, const char* key) {
    if (module == nullptr || key == nullptr) return nullptr;

    for (size_t i = 0; i < kTableCount; ++i) {
        if (strcmp(kTable[i].module, module) == 0 &&
            strcmp(kTable[i].key, key) == 0) {
            return &kTable[i];
        }
    }
    return nullptr;
}

// ============================================================================
// Public API
// ============================================================================

bool RobotSettings::get(const char* module, const char* key,
                        std::string& value_out) const {
    const SettingEntry* entry = find(module, key);
    if (entry == nullptr) return false;
    return format_entry(data_, *entry, value_out);
}

bool RobotSettings::set(const char* module, const char* key, const char* value) {
    const SettingEntry* entry = find(module, key);
    if (entry == nullptr) return false;
    return parse_into(data_, *entry, value);
}

void RobotSettings::list(const char* module, std::string& out) const {
    out.clear();
    const bool all = module == nullptr || module[0] == '\0';

    for (size_t i = 0; i < kTableCount; ++i) {
        const SettingEntry& entry = kTable[i];
        if (!all && strcmp(entry.module, module) != 0) continue;

        std::string value;
        format_entry(data_, entry, value);

        out += entry.module;
        out += '.';
        out += entry.key;
        out += '=';
        out += value;
        out += '\n';
    }
}

void RobotSettings::list_all(std::string& out) const {
    list(nullptr, out);
}

void RobotSettings::reset_all() {
    data_ = SettingsData{};
}

bool RobotSettings::reset_module(const char* module) {
    if (module == nullptr || module[0] == '\0') return false;

    static const SettingsData kDefaults{};
    bool found = false;

    for (size_t i = 0; i < kTableCount; ++i) {
        const SettingEntry& entry = kTable[i];
        if (strcmp(entry.module, module) != 0) continue;

        found = true;
        memcpy(reinterpret_cast<uint8_t*>(&data_) + entry.offset,
               reinterpret_cast<const uint8_t*>(&kDefaults) + entry.offset,
               entry.size);
    }

    return found;
}

float RobotSettings::freq_ekf() const {
    return 1.0f / (static_cast<float>(data_.sample_micros) * 0.000001f);
}

// ============================================================================
// SD persistence — same hand-rolled line-parsing style as
// OTAUpdater::parse_file (lib/OTAUpdater/OTAUpdater.cpp), applied to a
// "[module]\nkey=value" text file instead of comma-separated lines.
// ============================================================================

bool RobotSettings::save(SDCard& card) const {
    if (!card.is_mounted()) return false;

    std::string buffer;
    buffer.reserve(kTableCount * 24);

    const char* current_module = "";
    for (size_t i = 0; i < kTableCount; ++i) {
        const SettingEntry& entry = kTable[i];

        if (strcmp(current_module, entry.module) != 0) {
            if (i != 0) buffer += '\n';
            buffer += '[';
            buffer += entry.module;
            buffer += "]\n";
            current_module = entry.module;
        }

        std::string value;
        format_entry(data_, entry, value);

        buffer += entry.key;
        buffer += '=';
        buffer += value;
        buffer += '\n';
    }

    return card.write_file(ROBOT_SETTINGS_FILE, buffer.data(), buffer.size());
}

bool RobotSettings::load(SDCard& card, uint16_t* skipped_lines) {
    if (skipped_lines != nullptr) *skipped_lines = 0;
    if (!card.is_mounted()) return false;

    std::string buffer(ROBOT_SETTINGS_FILE_MAX_BYTES, '\0');
    size_t bytes_read = 0;

    if (!card.read_file(ROBOT_SETTINGS_FILE, &buffer[0], buffer.size() - 1,
                        &bytes_read)) {
        // Missing (or unreadable) file: keep the compiled-in defaults
        // already in data_ and persist them, so the SD ends up with a
        // complete, valid file that matches what is actually running.
        return save(card);
    }
    buffer.resize(bytes_read);
    buffer.push_back('\0');

    char current_module[kMaxModuleNameLength] = "";
    char* line_ctx = nullptr;
    char* line = strtok_r(&buffer[0], "\r\n", &line_ctx);

    while (line != nullptr) {
        while (*line == ' ' || *line == '\t') ++line;

        if (line[0] == '[') {
            const char* close = strchr(line, ']');
            if (close != nullptr && close > line + 1) {
                size_t len = static_cast<size_t>(close - line - 1);
                if (len >= sizeof(current_module)) len = sizeof(current_module) - 1;
                memcpy(current_module, line + 1, len);
                current_module[len] = '\0';
            }
        } else if (line[0] != '\0' && line[0] != '#' && line[0] != ';') {
            char* equals = strchr(line, '=');
            if (equals != nullptr) {
                *equals = '\0';
                const SettingEntry* entry = find(current_module, line);
                if (entry == nullptr || !parse_into(data_, *entry, equals + 1)) {
                    if (skipped_lines != nullptr) ++*skipped_lines;
                }
            } else if (skipped_lines != nullptr) {
                ++*skipped_lines;
            }
        }

        line = strtok_r(nullptr, "\r\n", &line_ctx);
    }

    return true;
}
