#include <RobotSettings.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "driver/gpio.h"

#include <SDCard.h>
#include <TinyShell.h>
#include <Logger.h>

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
    ENTRY("identity", name,        STRING),
    ENTRY("identity", description, STRING),

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

    const bool written = card.write_file(ROBOT_SETTINGS_FILE, buffer.data(), buffer.size());
    if (written) ++save_revision_;
    return written;
}

bool RobotSettings::load(SDCard& card, uint16_t* skipped_lines) {
    if (skipped_lines != nullptr) *skipped_lines = 0;
    if (!card.is_mounted()) return false;

    if (!parse_file(card, data_, skipped_lines)) {
        // Missing (or unreadable) file: keep the compiled-in defaults
        // already in data_ and persist them, so the SD ends up with a
        // complete, valid file that matches what is actually running.
        return save(card);
    }
    return true;
}

// The read-and-parse half of load(), against any destination. diff() uses it
// to see what the FILE says without disturbing the live values.
bool RobotSettings::parse_file(SDCard& card, SettingsData& out,
                               uint16_t* skipped_lines) const {
    if (!card.is_mounted()) return false;

    std::string buffer(ROBOT_SETTINGS_FILE_MAX_BYTES, '\0');
    size_t bytes_read = 0;

    if (!card.read_file(ROBOT_SETTINGS_FILE, &buffer[0], buffer.size() - 1,
                        &bytes_read)) {
        return false;
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
                if (entry == nullptr || !parse_into(out, *entry, equals + 1)) {
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

// ============================================================================
// Applying changes without a reboot
// ============================================================================

bool RobotSettings::module_exists(const char* module) {
    if (module == nullptr || module[0] == '\0') return false;
    for (size_t i = 0; i < kTableCount; ++i) {
        if (std::strcmp(kTable[i].module, module) == 0) return true;
    }
    return false;
}

bool RobotSettings::register_applier(const char* module, ApplyFn apply) {
    if (!module_exists(module) || !apply) return false;
    if (applier_count_ >= kMaxAppliers) return false;

    // Re-registering a module replaces its applier rather than adding a
    // second one, so a caller that runs twice cannot end up applying twice.
    for (size_t i = 0; i < applier_count_; ++i) {
        if (std::strcmp(appliers_[i].module, module) == 0) {
            appliers_[i].apply = std::move(apply);
            return true;
        }
    }

    appliers_[applier_count_].module = module;
    appliers_[applier_count_].apply = std::move(apply);
    ++applier_count_;
    return true;
}

RobotSettings::ApplyResult RobotSettings::apply(const char* module) {
    if (!module_exists(module)) return ApplyResult::UnknownModule;

    for (size_t i = 0; i < applier_count_; ++i) {
        if (std::strcmp(appliers_[i].module, module) != 0) continue;
        return appliers_[i].apply(data_) ? ApplyResult::Applied
                                         : ApplyResult::Failed;
    }

    // A real module that simply cannot be changed while running (every
    // pins_* group, and sensor -- see ROBOT::registerSettingsAppliers). Not
    // an error: the caller turns this into "requires a reboot".
    return ApplyResult::NoApplier;
}

void RobotSettings::apply_all(size_t* applied, size_t* failed) {
    size_t ok = 0;
    size_t bad = 0;

    for (size_t i = 0; i < applier_count_; ++i) {
        if (appliers_[i].apply(data_)) ++ok;
        else                           ++bad;
    }

    if (applied != nullptr) *applied = ok;
    if (failed != nullptr)  *failed = bad;
}

const char* RobotSettings::apply_result_text(ApplyResult result) {
    switch (result) {
        case ApplyResult::Applied:       return "applied";
        case ApplyResult::Failed:        return "failed";
        case ApplyResult::NoApplier:     return "requires a reboot";
        case ApplyResult::UnknownModule: return "unknown module";
    }
    return "unknown";
}

bool RobotSettings::diff(SDCard& card, std::string& out) const {
    // Parsed into a scratch copy that starts from the compiled-in defaults,
    // exactly like a fresh boot would: a key missing from the file therefore
    // compares as its default, which is what the next boot would actually
    // use.
    SettingsData on_card;
    if (!parse_file(card, on_card, nullptr)) return false;

    size_t differences = 0;
    for (size_t i = 0; i < kTableCount; ++i) {
        std::string memory_value;
        std::string file_value;
        if (!format_entry(data_, kTable[i], memory_value)) continue;
        if (!format_entry(on_card, kTable[i], file_value)) continue;
        if (memory_value == file_value) continue;

        out += kTable[i].module;
        out += '.';
        out += kTable[i].key;
        out += " memory=";
        out += memory_value;
        out += " file=";
        out += file_value;
        out += '\n';
        ++differences;
    }

    out += "differences=";
    out += std::to_string(differences);
    return true;
}

// ============================================================================
// Shell commands
// ============================================================================

void RobotSettings::register_shell_commands(TinyShell& shell, Logger& logger, SDCard& sd_card,
                                            std::function<void()> mark_direct_output) {
    shell.create_module("settings", "Runtime robot settings backed by settings.conf");

    shell.add([this, &logger](std::string module) -> uint8_t {
        const ApplyResult result = apply(module.c_str());
        if (result == ApplyResult::Applied) {
            logger.insert_logf(logType::INFO, "module=%s applied=1", module.c_str());
            return RESULT_OK;
        }
        // NoApplier is the honest answer for every pins_* group and for
        // "sensor": say which, instead of the silence this command replaces.
        logger.insert_logf(logType::ERRO, "module=%s applied=0 reason=%s",
                           module.c_str(), apply_result_text(result));
        return RESULT_ERROR;
    }, "apply", "Push one module's in-memory values into the running system", "settings");

    shell.add([this, &logger]() -> uint8_t {
        size_t applied = 0;
        size_t failed = 0;
        apply_all(&applied, &failed);
        logger.insert_logf(logType::INFO, "applied=%u failed=%u",
                           static_cast<unsigned>(applied),
                           static_cast<unsigned>(failed));
        return (failed == 0) ? RESULT_OK : RESULT_ERROR;
    }, "apply_all", "Run every registered applier", "settings");

    shell.add([this, &logger, &sd_card]() -> uint8_t {
        std::string report;
        if (!diff(sd_card, report)) {
            logger.insert_log(logType::ERRO,
                              "cannot read settings.conf (card not mounted or file missing)");
            return RESULT_ERROR;
        }
        logger.insert_log(logType::INFO, report.c_str());
        return RESULT_OK;
    }, "diff", "Show settings that differ between memory and settings.conf", "settings");

    shell.add([this, &logger](std::string module, std::string key) -> uint8_t {
        std::string value;
        if (!get(module.c_str(), key.c_str(), value)) {
            logger.insert_logf(logType::ERRO, "Unknown setting: %s.%s",
                               module.c_str(), key.c_str());
            return RESULT_ERROR;
        }

        logger.insert_logf(logType::INFO, "%s.%s=%s", module.c_str(),
                           key.c_str(), value.c_str());
        return RESULT_OK;
    }, "get", "Read one setting: module,key", "settings");

    shell.add([this, &logger](std::string module, std::string key, std::string value) -> uint8_t {
        if (!set(module.c_str(), key.c_str(), value.c_str())) {
            logger.insert_logf(
                logType::ERRO,
                "Failed to set %s.%s: unknown setting or invalid value",
                module.c_str(), key.c_str());
            return RESULT_ERROR;
        }

        logger.insert_logf(
            logType::INFO,
            "%s.%s set in memory; run 'settings save' to persist, "
            "'settings apply %s' to take effect now (some modules require a reboot)",
            module.c_str(), key.c_str(), module.c_str());
        return RESULT_OK;
    }, "set", "Change one setting in memory: module,key,value", "settings");

    shell.add([this, &logger](std::string module) -> uint8_t {
        std::string out;
        list(module.c_str(), out);

        if (out.empty()) {
            logger.insert_logf(logType::ERRO, "Unknown settings module: %s",
                               module.c_str());
            return RESULT_ERROR;
        }

        logger.insert_log(logType::INFO, out.c_str());
        return RESULT_OK;
    }, "list", "List every setting in one module", "settings");

    shell.add([this, &logger]() -> uint8_t {
        std::string out;
        list_all(out);
        logger.insert_log(logType::INFO, out.c_str());
        return RESULT_OK;
    }, "list_all", "List every setting in every module", "settings");

    shell.add([this, &logger]() -> uint8_t {
        // A cheap "has anything changed" check for a remote client -- see
        // save_revision()'s comment for why this exists instead of extending
        // MANIFEST_DATA's config_revision.
        logger.insert_logf(logType::INFO, "revision=%lu",
                           static_cast<unsigned long>(save_revision()));
        return RESULT_OK;
    }, "revision", "How many times settings were saved since boot", "settings");

    shell.add([this, &logger, &sd_card, mark_direct_output]() -> uint8_t {
        if (!save(sd_card)) {
            logger.insert_log(
                logType::ERRO, "Failed to save settings.conf: SD card not mounted");
            return RESULT_ERROR;
        }

        logger.send_log_direct(logType::INFO, "Settings saved to settings.conf");
        mark_direct_output();
        return RESULT_OK;
    }, "save", "Persist current settings to settings.conf", "settings");

    shell.add([this, &logger, &sd_card]() -> uint8_t {
        uint16_t skipped = 0;
        if (!load(sd_card, &skipped)) {
            logger.insert_log(
                logType::ERRO, "Failed to load settings.conf: SD card not mounted");
            return RESULT_ERROR;
        }

        logger.insert_logf(
            logType::INFO,
            "Settings reloaded from settings.conf (%u line(s) ignored); reboot to apply",
            skipped);
        return RESULT_OK;
    }, "load", "Reload settings.conf from SD, discarding unsaved edits", "settings");

    shell.add([this, &logger](std::string module) -> uint8_t {
        if (!reset_module(module.c_str())) {
            logger.insert_logf(logType::ERRO, "Unknown settings module: %s",
                               module.c_str());
            return RESULT_ERROR;
        }

        logger.insert_logf(
            logType::INFO,
            "%s reset to defaults in memory; run 'settings save' to persist",
            module.c_str());
        return RESULT_OK;
    }, "reset", "Reset one module to compiled-in defaults, in memory", "settings");

    shell.add([this, &logger]() -> uint8_t {
        reset_all();
        logger.insert_log(
            logType::INFO,
            "All settings reset to defaults in memory; run 'settings save' to persist");
        return RESULT_OK;
    }, "reset_all", "Reset every setting to compiled-in defaults, in memory", "settings");
}
