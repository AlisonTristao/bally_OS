// Shell registration for every module the ROBOT composition root owns.
//
// Split out of BallyRobot.cpp, which had grown past 1900 lines with the
// command registrations mixed into the boot/task/radio code. Same class,
// same members, different translation unit -- nothing here is a new
// abstraction, it is the same functions moved verbatim.
//
// THIS FILE IS THE ESP-IDF-ONLY SIDE OF THE SHELL. That matters beyond
// tidiness: ten libraries (BtpTransport, CommandProcessor, Format, KeyStore,
// ManifestResponder, RxRouter, StatusReporter, SubscriptionResponder,
// TelemetryPublisher, TxScheduler) are compiled by env:native for their unit
// tests, where TinyShell does not exist. A register_shell_commands() inside
// any of them would break `pio test -e native` for every suite at once, as a
// link error that does not name the cause. Commands that operate on those
// libraries are therefore registered HERE, from the composition root, which
// only the ESP-IDF build ever compiles -- the same reasoning RadioSeal.h
// documents for keeping btp::aead out of BtpTransport.
//
// Modules owned by a subsystem still live in that subsystem's own .cpp (see
// CONTRIBUTING.md, "Nova biblioteca" item 5); startWrappers() below is only
// the composition list that calls them.

#include <BallyRobot.h>
#include <Settings.h>

#include <cstdint>
#include <cstdio>
#include <string>

#include "esp_app_desc.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"

// For bally::kChannelContractVersion, reported by "sec -channels". This is the
// one header that must stay byte-identical across bally_OS, bally_dongle and
// TraceView -- read it, never edit it from here.
#include <bally_channels.h>

namespace {

// verify_e/verify_l and protocol_uuid_ are public-by-construction values, so
// this is only ever pointed at those. Never at key_e()/key_l() -- see the
// SECURITY RULE on the KeyStore class.
void hexEncode(const std::uint8_t* data, std::size_t size, char* out) {
    static const char kDigits[] = "0123456789abcdef";
    for (std::size_t i = 0; i < size; ++i) {
        out[i * 2U]      = kDigits[(data[i] >> 4) & 0x0FU];
        out[i * 2U + 1U] = kDigits[data[i] & 0x0FU];
    }
    out[size * 2U] = '\0';
}

const char* resetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:  return "poweron";
        case ESP_RST_EXT:      return "external_pin";
        case ESP_RST_SW:       return "software";
        case ESP_RST_PANIC:    return "panic";
        case ESP_RST_INT_WDT:  return "interrupt_watchdog";
        case ESP_RST_TASK_WDT: return "task_watchdog";
        case ESP_RST_WDT:      return "other_watchdog";
        case ESP_RST_DEEPSLEEP:return "deepsleep_wake";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO:     return "sdio";
        case ESP_RST_UNKNOWN:
        default:               return "unknown";
    }
}

const char* otaStateName(esp_ota_img_states_t state) {
    switch (state) {
        case ESP_OTA_IMG_NEW:             return "new";
        case ESP_OTA_IMG_PENDING_VERIFY:  return "pending_verify";
        case ESP_OTA_IMG_VALID:           return "valid";
        case ESP_OTA_IMG_INVALID:         return "invalid";
        case ESP_OTA_IMG_ABORTED:         return "aborted";
        case ESP_OTA_IMG_UNDEFINED:
        default:                          return "undefined";
    }
}

// Deferred restart, so the RESULT_OK and the "Rebooting..." log actually get
// out over the radio before the chip goes down. Same 500 ms one-shot the
// "ota reboot" command has always used -- this is that implementation, moved.
void scheduleRestart() {
    esp_timer_handle_t reboot_timer = nullptr;
    const esp_timer_create_args_t timer_args = {
        .callback = [](void*) { esp_restart(); },
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "shell_reboot",
        .skip_unhandled_events = false,
    };
    esp_timer_create(&timer_args, &reboot_timer);
    esp_timer_start_once(reboot_timer, 500000);
}

}  // namespace


void ROBOT::startWrappers() {
    // Modules owned by ROBOT itself — see the declarations in BallyRobot.h
    // for why each one stays here instead of moving to a subsystem lib.
    registerSystemCommands();
    registerLinkCommands();
    registerTelemetryCommands();
    registerSecurityCommands();
    registerJobCommands();
    registerRobotIOCommands();
    registerKalmanCommands();
    registerDebugCommands();

    // Every other module is owned by the subsystem it operates on. "sensor"
    // and "storage" are each split across two owners below (array sensor vs.
    // encoders; SD file management vs. USB ownership) — both register into
    // the same TinyShell module name, which TinyShell allows.
    array_sensor->register_shell_commands(shell, logger, settings);
    Encoder::register_shell_commands(shell, logger, *encoder_left, *encoder_right);
    junkebox->register_shell_commands(shell, logger);

    usb_storage.register_shell_commands(
        shell, logger, sd_card,
        [this]() { return anyDebugTestActive(); },
        [this]() { sendNextShellOutputDirect(); });
    sd_card.register_shell_commands(shell, logger);

    logger.register_shell_commands(
        shell, sd_card, settings,
        [this]() { sendNextShellOutputDirect(); });

    ota.register_shell_commands(
        shell, logger, sd_card, usb_storage,
        [this]() { return anyDebugTestActive(); },
        [this]() { sendNextShellOutputDirect(); });

    // "set"/"reset"/"reset_all" only change the in-memory copy; "save"
    // persists it, and every change needs a reboot to actually take effect
    // (pins, timers and EKF init are all applied once, at boot).
    settings.register_shell_commands(
        shell, logger, sd_card,
        [this]() { sendNextShellOutputDirect(); });

    sysmon.register_shell_commands(shell, logger);
}

void ROBOT::registerLinkCommands() {
    // "link": the radio and the protocol seen from the outside. Everything
    // here is a read of a counter that already exists and, until now, only
    // left the robot inside the binary CONTROL/STATUS message once a second.
    //
    // Registered here and not in TxScheduler/RxRouter/CommandProcessor because
    // all three are compiled by env:native for their unit suites, where
    // TinyShell does not exist -- see this file's header comment.
    shell.create_module("link", "Radio and protocol counters");

    shell.add([]() -> uint8_t {
        const RxRouter::Stats rx = instance_->rx_router_.stats();
        // Same aggregation publishStatus() does for the wire message, so the
        // two can be compared directly when something looks wrong.
        ROBOT::logger.insert_logf(
            logType::INFO,
            "frames_rx=%llu crc_errors=%llu decode_errors=%llu "
            "reassembly_completed=%llu reassembly_rejected=%llu reassembly_timeouts=%lu "
            "routed=%lu",
            static_cast<unsigned long long>(instance_->link_frames_rx_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(instance_->link_crc_errors_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(instance_->link_decode_errors_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(instance_->link_reassembly_completed_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(instance_->link_reassembly_rejected_.load(std::memory_order_relaxed)),
            static_cast<unsigned long>(rx.reassembly_timeouts),
            static_cast<unsigned long>(rx.routed));
        return RESULT_OK;
    }, "stats", "Receive-side link counters (same numbers CONTROL/STATUS carries)", "link");

    shell.add([]() -> uint8_t {
        const TxScheduler::Stats tx = instance_->tx_scheduler.stats();
        // Priority order is COMMAND_RESULT > STATUS > LOG > TELEMETRY > DEBUG
        // (see TxScheduler). Printed per class because "which class is being
        // dropped" is the whole diagnostic value here.
        ROBOT::logger.insert_logf(
            logType::INFO,
            "accepted=%lu delivered=%lu timeouts=%lu dropped=%lu delivery_failed=%lu "
            "queued=%lu/%lu/%lu/%lu/%lu dropped_by_class=%lu/%lu/%lu/%lu/%lu",
            static_cast<unsigned long>(tx.accepted),
            static_cast<unsigned long>(tx.delivered),
            static_cast<unsigned long>(tx.timeouts),
            static_cast<unsigned long>(tx.dropped),
            static_cast<unsigned long>(tx.delivery_failed),
            static_cast<unsigned long>(tx.queued_by_priority[0]),
            static_cast<unsigned long>(tx.queued_by_priority[1]),
            static_cast<unsigned long>(tx.queued_by_priority[2]),
            static_cast<unsigned long>(tx.queued_by_priority[3]),
            static_cast<unsigned long>(tx.queued_by_priority[4]),
            static_cast<unsigned long>(tx.dropped_by_priority[0]),
            static_cast<unsigned long>(tx.dropped_by_priority[1]),
            static_cast<unsigned long>(tx.dropped_by_priority[2]),
            static_cast<unsigned long>(tx.dropped_by_priority[3]),
            static_cast<unsigned long>(tx.dropped_by_priority[4]));
        return RESULT_OK;
    }, "tx", "Transmit scheduler counters, per priority class", "link");

    shell.add([]() -> uint8_t {
        const RxRouter::Stats rx = instance_->rx_router_.stats();
        ROBOT::logger.insert_logf(
            logType::INFO,
            "routed=%lu fragments_accepted=%lu duplicate_fragments=%lu "
            "dropped_decode=%lu dropped_crc=%lu dropped_reassembly=%lu "
            "dropped_invalid_argument=%lu reassembly_timeouts=%lu slots=%u",
            static_cast<unsigned long>(rx.routed),
            static_cast<unsigned long>(rx.fragments_accepted),
            static_cast<unsigned long>(rx.duplicate_fragments),
            static_cast<unsigned long>(rx.dropped_decode),
            static_cast<unsigned long>(rx.dropped_crc),
            static_cast<unsigned long>(rx.dropped_reassembly),
            static_cast<unsigned long>(rx.dropped_invalid_argument),
            static_cast<unsigned long>(rx.reassembly_timeouts),
            static_cast<unsigned>(RxRouter::kSlotCount));
        return RESULT_OK;
    }, "rx", "Decode and reassembly counters", "link");

    shell.add([]() -> uint8_t {
        const CommandProcessor::Stats cmd = instance_->command_processor.stats();
        // "unauthorized" counts both a frame from a MAC that is not the peer
        // and one whose AEAD tag did not verify -- the two rejections
        // handleReceiveStatic makes before a command can reach the shell.
        ROBOT::logger.insert_logf(
            logType::INFO,
            "accepted=%lu executed=%lu duplicates=%lu replayed=%lu "
            "rejected=%lu dropped=%lu unauthorized=%lu cache_slots=%u",
            static_cast<unsigned long>(cmd.accepted),
            static_cast<unsigned long>(cmd.executed),
            static_cast<unsigned long>(cmd.duplicates),
            static_cast<unsigned long>(cmd.replayed),
            static_cast<unsigned long>(cmd.rejected),
            static_cast<unsigned long>(cmd.dropped),
            static_cast<unsigned long>(cmd.unauthorized),
            static_cast<unsigned>(CommandProcessor::kCacheCapacity));
        return RESULT_OK;
    }, "cmd_stats", "Command intake: dedup, replay and rejection counters", "link");

    shell.add([]() -> uint8_t {
        uint8_t channel = 0;
        wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
        const bool read_ok = esp_wifi_get_channel(&channel, &second) == ESP_OK;

        // The configured value and the one the radio is actually parked on can
        // differ: OTA moves the radio to the access point's channel and only
        // gives it back on cancel(). Printing both is what makes "the dongle
        // stopped hearing me" diagnosable.
        ROBOT::logger.insert_logf(
            logType::INFO, "configured_channel=%u radio_channel=%s ota_active=%d",
            static_cast<unsigned>(instance_->settings.data().espnow_channel),
            read_ok ? std::to_string(channel).c_str() : "unknown",
            instance_->ota.is_active() ? 1 : 0);
        return RESULT_OK;
    }, "channel", "Configured vs. actual ESP-NOW radio channel", "link");

    shell.add([]() -> uint8_t {
        instance_->captureLinkBaseline();
        ROBOT::logger.insert_log(
            logType::INFO,
            "baseline=set counters unchanged; use 'link -delta' for the difference");
        return RESULT_OK;
    }, "reset_stats", "Mark a zero point for 'link -delta' (counters are not zeroed)", "link");

    shell.add([]() -> uint8_t {
        if (!instance_->link_baseline_.set) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "no baseline: run 'link -reset_stats' first");
            return RESULT_ERROR;
        }

        const LinkStatsBaseline& base = instance_->link_baseline_;
        const RxRouter::Stats rx = instance_->rx_router_.stats();
        const TxScheduler::Stats tx = instance_->tx_scheduler.stats();
        const CommandProcessor::Stats cmd = instance_->command_processor.stats();
        const uint32_t now_ms =
            static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);

        ROBOT::logger.insert_logf(
            logType::INFO,
            "since_reset_ms=%lu frames_rx=%llu crc_errors=%llu decode_errors=%llu "
            "reassembly_timeouts=%lu tx_accepted=%lu tx_dropped=%lu "
            "cmd_executed=%lu cmd_duplicates=%lu cmd_unauthorized=%lu",
            static_cast<unsigned long>(now_ms - base.uptime_ms),
            static_cast<unsigned long long>(
                instance_->link_frames_rx_.load(std::memory_order_relaxed) - base.frames_rx),
            static_cast<unsigned long long>(
                instance_->link_crc_errors_.load(std::memory_order_relaxed) - base.crc_errors),
            static_cast<unsigned long long>(
                instance_->link_decode_errors_.load(std::memory_order_relaxed) - base.decode_errors),
            static_cast<unsigned long>(rx.reassembly_timeouts - base.rx.reassembly_timeouts),
            static_cast<unsigned long>(tx.accepted - base.tx.accepted),
            static_cast<unsigned long>(tx.dropped - base.tx.dropped),
            static_cast<unsigned long>(cmd.executed - base.command.executed),
            static_cast<unsigned long>(cmd.duplicates - base.command.duplicates),
            static_cast<unsigned long>(cmd.unauthorized - base.command.unauthorized));
        return RESULT_OK;
    }, "delta", "Counter difference since the last 'link -reset_stats'", "link");
}

void ROBOT::registerTelemetryCommands() {
    // "telemetry": what the robot publishes, to whom, and how fast. Same
    // env:native reason as "link" for living here.
    shell.create_module("telemetry", "Topics, subscriptions and publish rates");

    shell.add([]() -> uint8_t {
        TelemetryPublisher::TopicStats topics[StatusReporter::kMaxTopicRecords];
        const size_t count =
            instance_->telemetry.topic_stats(topics, StatusReporter::kMaxTopicRecords);

        std::string out;
        char line[176];
        for (size_t i = 0; i < count; ++i) {
            const uint64_t period_us =
                instance_->telemetry.topic_period_us(topics[i].topic_id);
            std::snprintf(
                line, sizeof(line),
                "topic_id=0x%04X subscribers=%u rate_millihz=%lu period_us=%llu "
                "bytes=%llu dropped=%llu\n",
                static_cast<unsigned>(topics[i].topic_id),
                static_cast<unsigned>(topics[i].subscriber_count),
                static_cast<unsigned long>(topics[i].effective_rate_millihz),
                static_cast<unsigned long long>(period_us),
                static_cast<unsigned long long>(topics[i].bytes_sent_total),
                static_cast<unsigned long long>(topics[i].samples_dropped_total));
            out += line;
        }
        // period_us == 0 means "nobody is subscribed, so it is not being
        // published" -- that is the definition sampleTelemetry() gates on, not
        // an error.
        std::snprintf(line, sizeof(line), "topics=%u subscriptions=%u queued=%u",
                      static_cast<unsigned>(count),
                      static_cast<unsigned>(instance_->telemetry.active_subscription_count()),
                      static_cast<unsigned>(instance_->telemetry.queued_count()));
        out += line;

        ROBOT::logger.insert_log(logType::INFO, out.c_str());
        return RESULT_OK;
    }, "topics", "Per-topic subscribers, granted rate, bytes and drops", "telemetry");

    shell.add([](uint16_t topic_id, uint32_t rate_millihz,
                 uint32_t lease_ms) -> uint8_t {
        // Subscriptions are keyed by (session, topic), and the session comes
        // from the BTP envelope. A shell subscription has no envelope, so it
        // uses the reserved local identity -- otherwise the dongle's own
        // drop_session() would take this one down with its own.
        const TelemetryPublisher::SubscribeOutcome outcome =
            instance_->telemetry.subscribe(
                topic_id,
                TelemetryPublisher::kLocalSubscriberSourceId,
                TelemetryPublisher::kLocalSubscriberBootId,
                rate_millihz, lease_ms,
                static_cast<uint64_t>(esp_timer_get_time()));

        if (!outcome.topic_known) {
            ROBOT::logger.insert_logf(logType::ERRO, "unknown topic_id=0x%04X",
                                      static_cast<unsigned>(topic_id));
            return RESULT_ERROR;
        }
        if (outcome.rate_below_minimum) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "rate below the schema minimum; the robot may not publish slower than asked");
            return RESULT_ERROR;
        }
        if (outcome.capacity_exhausted) {
            ROBOT::logger.insert_log(logType::ERRO, "subscription table full");
            return RESULT_ERROR;
        }

        // The lease is clamped by the publisher, so report what was granted
        // rather than what was asked: this subscription DOES expire, and a
        // shell caller has no renewal loop behind it.
        ROBOT::logger.insert_logf(
            logType::INFO,
            "subscription_id=%lu topic_id=0x%04X rate_millihz=%lu lease_ms=%lu",
            static_cast<unsigned long>(outcome.subscription_id),
            static_cast<unsigned>(topic_id),
            static_cast<unsigned long>(outcome.effective_rate_millihz),
            static_cast<unsigned long>(outcome.granted_lease_ms));
        return RESULT_OK;
    }, "sub", "Subscribe locally: topic_id,rate_millihz,lease_ms", "telemetry");

    shell.add([](uint32_t subscription_id) -> uint8_t {
        const TelemetryPublisher::UnsubscribeOutcome outcome =
            instance_->telemetry.unsubscribe(subscription_id);
        if (outcome != TelemetryPublisher::UnsubscribeOutcome::Removed) {
            ROBOT::logger.insert_logf(
                logType::ERRO, "no live subscription with id=%lu",
                static_cast<unsigned long>(subscription_id));
            return RESULT_ERROR;
        }
        ROBOT::logger.insert_logf(logType::INFO, "unsubscribed id=%lu",
                                  static_cast<unsigned long>(subscription_id));
        return RESULT_OK;
    }, "unsub", "Cancel a subscription by id (see topics/sub)", "telemetry");

    shell.add([]() -> uint8_t {
        // Reads the same table ManifestResponder serializes into
        // MANIFEST_DATA, so the text here can never disagree with the wire.
        size_t schema_count = 0;
        const TelemetryPublisher::TopicSchema* schemas =
            TelemetryPublisher::schemas(&schema_count);

        std::string out;
        char line[176];
        for (size_t i = 0; i < schema_count; ++i) {
            const TelemetryPublisher::TopicSchema& schema = schemas[i];
            std::snprintf(
                line, sizeof(line),
                "topic_id=0x%04X name=%s version=%u fields=%u payload_bytes=%u "
                "max_rate_millihz=%lu\n",
                static_cast<unsigned>(schema.topic_id), schema.name,
                static_cast<unsigned>(schema.schema_version),
                static_cast<unsigned>(schema.field_count),
                static_cast<unsigned>(schema.payload_size),
                static_cast<unsigned long>(schema.max_rate_millihz));
            out += line;
        }
        // max_rate_millihz == 0 is commands.md section 3.3's "not periodic",
        // which is what robot.state is: published on transitions only.
        out += "note: max_rate_millihz=0 means event-driven, not periodic";

        ROBOT::logger.insert_log(logType::INFO, out.c_str());
        return RESULT_OK;
    }, "manifest", "The static topic schemas, as MANIFEST_DATA reports them", "telemetry");

    shell.add([]() -> uint8_t {
        const TelemetryPublisher::Stats stats = instance_->telemetry.stats();
        ROBOT::logger.insert_logf(
            logType::INFO,
            "queued=%lu sent=%lu dropped_full=%lu dropped_invalid=%lu "
            "send_failed=%lu pending=%u",
            static_cast<unsigned long>(stats.queued),
            static_cast<unsigned long>(stats.sent),
            static_cast<unsigned long>(stats.dropped_full),
            static_cast<unsigned long>(stats.dropped_invalid),
            static_cast<unsigned long>(stats.send_failed),
            static_cast<unsigned>(instance_->telemetry.queued_count()));
        return RESULT_OK;
    }, "stats", "Publisher queue counters", "telemetry");
}

void ROBOT::registerSecurityCommands() {
    // "sec": which keys are loaded and which channel they actually protect.
    //
    // SECURITY RULE, do not weaken it: nothing in this module may print
    // key_e() or key_l(), not partially and not behind a debug flag. Only the
    // verify tags, which are the images of the keys under a one-way function
    // and are public by construction -- and are exactly what tells a bench
    // WHICH key is wrong. See the KeyStore class comment.
    shell.create_module("sec", "Channel keys and their status");

    shell.add([]() -> uint8_t {
        if (!instance_->key_store.loaded()) {
            ROBOT::logger.insert_logf(
                logType::WARN, "loaded=0 failed_field=%s file=%s",
                KeyStore::error_field(instance_->key_store.last_error()),
                KEY_STORE_FILE);
            return RESULT_OK;
        }

        char verify_e_hex[KeyStore::kVerifyLength * 2U + 1U];
        char verify_l_hex[KeyStore::kVerifyLength * 2U + 1U];
        hexEncode(instance_->key_store.verify_e(), KeyStore::kVerifyLength, verify_e_hex);
        hexEncode(instance_->key_store.verify_l(), KeyStore::kVerifyLength, verify_l_hex);

        ROBOT::logger.insert_logf(
            logType::INFO,
            "loaded=1 verify_e=%s verify_l=%s kdf_iterations=%lu cipher=aes-128-gcm",
            verify_e_hex, verify_l_hex,
            static_cast<unsigned long>(instance_->key_store.iterations()));
        return RESULT_OK;
    }, "keys", "Public verify tags of the two channel keys (never the keys)", "sec");

    shell.add([]() -> uint8_t {
        // Re-reads bally.key without a reboot, so a re-provisioned card can be
        // picked up in place. A failed reload wipes the previously held keys
        // (KeyStore::load), which means channel C stops opening frames rather
        // than silently keeping stale material -- say so plainly.
        if (!instance_->key_store.load_from_card(instance_->sd_card)) {
            ROBOT::logger.insert_logf(
                logType::ERRO,
                "reload failed at field=%s; previously held keys were wiped, "
                "sealed traffic will now be refused until a valid %s is loaded",
                KeyStore::error_field(instance_->key_store.last_error()),
                KEY_STORE_FILE);
            return RESULT_ERROR;
        }

        char verify_e_hex[KeyStore::kVerifyLength * 2U + 1U];
        char verify_l_hex[KeyStore::kVerifyLength * 2U + 1U];
        hexEncode(instance_->key_store.verify_e(), KeyStore::kVerifyLength, verify_e_hex);
        hexEncode(instance_->key_store.verify_l(), KeyStore::kVerifyLength, verify_l_hex);
        ROBOT::logger.insert_logf(logType::INFO,
                                  "reloaded=1 verify_e=%s verify_l=%s",
                                  verify_e_hex, verify_l_hex);
        return RESULT_OK;
    }, "reload", "Re-read bally.key from the SD card without rebooting", "sec");

    shell.add([]() -> uint8_t {
        // Reports the gap as much as the state: key E is loaded and unused,
        // because channel B has no path into this firmware yet. That is the
        // single most important thing an operator can know about this build's
        // security posture, so it is printed, not left to documentation.
        const bool loaded = instance_->key_store.loaded();
        ROBOT::logger.insert_logf(
            logType::INFO,
            "contract_version=%u channel_a=console_cleartext "
            "channel_b=endpoint_key_e sealed=0 key_e_loaded=%d "
            "channel_c=link_key_l sealed=1 key_l_loaded=%d",
            static_cast<unsigned>(bally::kChannelContractVersion),
            loaded ? 1 : 0, loaded ? 1 : 0);
        ROBOT::logger.insert_log(
            logType::WARN,
            "channel B is not implemented in this firmware: any peer holding "
            "the fleet key L can run every command, motors included");
        return RESULT_OK;
    }, "channels", "Which channel each key protects, and which are sealed today", "sec");
}

void ROBOT::registerJobCommands() {
    // "job": the robot running its own policy instead of waiting to be told.
    //
    // Registered here and not in lib/JobScheduler because that library is
    // deliberately TinyShell-free so it can be unit tested under env:native
    // (test/test_job_scheduler) -- the same rule the ten protocol libraries
    // follow, see this file's header comment.
    //
    // Argument note, because it bites: TinyShell splits arguments on unescaped
    // commas, so the scheduled command travels as one string argument and may
    // contain spaces freely. A command that itself needs commas escapes them:
    //   job -every 1000, robot -set_pwm_pair 40\, 40\, 500
    shell.create_module("job", "Scheduled shell commands and SD scripts");

    shell.add([](uint32_t interval_ms, std::string command) -> uint8_t {
        uint8_t id = 0;
        const JobScheduler::Error error = instance_->jobs.schedule_interval(
            interval_ms, 0U, command.c_str(), &id);
        if (error != JobScheduler::Error::Ok) {
            ROBOT::logger.insert_logf(logType::ERRO, "job refused: %s",
                                      JobScheduler::error_text(error));
            return RESULT_ERROR;
        }
        ROBOT::logger.insert_logf(logType::INFO,
                                  "job_id=%u kind=every interval_ms=%lu repeat=0",
                                  static_cast<unsigned>(id),
                                  static_cast<unsigned long>(interval_ms));
        return RESULT_OK;
    }, "every", "Run a command forever on an interval: interval_ms,command", "job");

    shell.add([](uint32_t interval_ms, uint32_t times,
                 std::string command) -> uint8_t {
        if (times == 0U) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "job refused: times must be at least 1 (use 'every' for no limit)");
            return RESULT_ERROR;
        }

        uint8_t id = 0;
        const JobScheduler::Error error = instance_->jobs.schedule_interval(
            interval_ms, times, command.c_str(), &id);
        if (error != JobScheduler::Error::Ok) {
            ROBOT::logger.insert_logf(logType::ERRO, "job refused: %s",
                                      JobScheduler::error_text(error));
            return RESULT_ERROR;
        }
        ROBOT::logger.insert_logf(logType::INFO,
                                  "job_id=%u kind=every interval_ms=%lu repeat=%lu",
                                  static_cast<unsigned>(id),
                                  static_cast<unsigned long>(interval_ms),
                                  static_cast<unsigned long>(times));
        return RESULT_OK;
    }, "repeat", "Run a command a fixed number of times: interval_ms,times,command", "job");

    shell.add([](uint32_t delay_ms, std::string command) -> uint8_t {
        uint8_t id = 0;
        const JobScheduler::Error error =
            instance_->jobs.schedule_once(delay_ms, command.c_str(), &id);
        if (error != JobScheduler::Error::Ok) {
            ROBOT::logger.insert_logf(logType::ERRO, "job refused: %s",
                                      JobScheduler::error_text(error));
            return RESULT_ERROR;
        }
        ROBOT::logger.insert_logf(logType::INFO,
                                  "job_id=%u kind=once delay_ms=%lu",
                                  static_cast<unsigned>(id),
                                  static_cast<unsigned long>(delay_ms));
        return RESULT_OK;
    }, "once", "Run a command a single time after a delay: delay_ms,command", "job");

    shell.add([](std::string state_name, std::string command) -> uint8_t {
        const uint8_t state = StateMachine::stateFromString(state_name.c_str());
        if (state == NONE) {
            ROBOT::logger.insert_logf(
                logType::ERRO,
                "unknown state '%s' (SETUP|WAIT|CALIBRATE|DEBUG|RUN|FINISH|TELEMETRY|ERROR)",
                state_name.c_str());
            return RESULT_ERROR;
        }

        uint8_t id = 0;
        const JobScheduler::Error error =
            instance_->jobs.schedule_on_state(state, command.c_str(), &id);
        if (error != JobScheduler::Error::Ok) {
            ROBOT::logger.insert_logf(logType::ERRO, "job refused: %s",
                                      JobScheduler::error_text(error));
            return RESULT_ERROR;
        }
        // "on entering", not "while in": a job on the state the robot is
        // already sitting in waits for the next entry.
        ROBOT::logger.insert_logf(logType::INFO, "job_id=%u kind=at state=%s",
                                  static_cast<unsigned>(id),
                                  StateMachine::stateToString(state));
        return RESULT_OK;
    }, "at", "Run a command every time a state is entered: state,command", "job");

    shell.add([]() -> uint8_t {
        JobScheduler::JobView views[JobScheduler::kMaxJobs];
        const size_t count = instance_->jobs.list(views, JobScheduler::kMaxJobs);
        if (count == 0U) {
            ROBOT::logger.insert_log(logType::INFO, "jobs=0");
            return RESULT_OK;
        }

        std::string out;
        char line[192];
        for (size_t i = 0; i < count; ++i) {
            if (views[i].kind == JobScheduler::Kind::OnStateEnter) {
                std::snprintf(line, sizeof(line),
                              "%u kind=at state=%s fired=%lu cmd=%s\n",
                              static_cast<unsigned>(views[i].id),
                              StateMachine::stateToString(views[i].state),
                              static_cast<unsigned long>(views[i].fired),
                              views[i].command);
            } else {
                std::snprintf(line, sizeof(line),
                              "%u kind=%s interval_ms=%lu remaining=%lu fired=%lu cmd=%s\n",
                              static_cast<unsigned>(views[i].id),
                              JobScheduler::kind_text(views[i].kind),
                              static_cast<unsigned long>(views[i].interval_ms),
                              static_cast<unsigned long>(views[i].remaining),
                              static_cast<unsigned long>(views[i].fired),
                              views[i].command);
            }
            out += line;
        }
        ROBOT::logger.insert_log(logType::INFO, out.c_str());
        return RESULT_OK;
    }, "list", "List the scheduled jobs", "job");

    shell.add([](uint8_t job_id) -> uint8_t {
        if (!instance_->jobs.cancel(job_id)) {
            ROBOT::logger.insert_logf(logType::ERRO, "no active job with id=%u",
                                      static_cast<unsigned>(job_id));
            return RESULT_ERROR;
        }
        ROBOT::logger.insert_logf(logType::INFO, "cancelled job_id=%u",
                                  static_cast<unsigned>(job_id));
        return RESULT_OK;
    }, "cancel", "Cancel one job by id (see list)", "job");

    shell.add([]() -> uint8_t {
        const size_t cancelled = instance_->jobs.cancel_all();
        ROBOT::logger.insert_logf(logType::INFO, "cancelled=%u",
                                  static_cast<unsigned>(cancelled));
        return RESULT_OK;
    }, "cancel_all", "Cancel every scheduled job", "job");

    shell.add([](std::string path) -> uint8_t {
        if (instance_->script_active_) {
            ROBOT::logger.insert_log(logType::ERRO,
                                     "a script is already running; wait for it to finish");
            return RESULT_ERROR;
        }
        if (!instance_->startScript(path.c_str())) {
            ROBOT::logger.insert_logf(
                logType::ERRO,
                "cannot run %s: card not mounted, file missing, or unreadable",
                path.c_str());
            return RESULT_ERROR;
        }
        // Fed one line per routine() pass, so a long script cannot overflow
        // the 10-deep command queue. It runs after this reply, not during it.
        ROBOT::logger.insert_logf(logType::INFO, "script=%s started", path.c_str());
        return RESULT_OK;
    }, "run_file", "Run a file of shell commands from the SD card, one line per pass", "job");

    shell.add([]() -> uint8_t {
        const JobScheduler::Stats stats = instance_->jobs.stats();
        ROBOT::logger.insert_logf(
            logType::INFO,
            "jobs=%u fired=%lu submit_failed=%lu rejected=%lu skipped_late=%lu "
            "script_active=%d script_line=%u",
            static_cast<unsigned>(instance_->jobs.active_count()),
            static_cast<unsigned long>(stats.fired),
            static_cast<unsigned long>(stats.submit_failed),
            static_cast<unsigned long>(stats.rejected),
            static_cast<unsigned long>(stats.skipped_late),
            instance_->script_active_ ? 1 : 0,
            static_cast<unsigned>(instance_->script_line_));
        return RESULT_OK;
    }, "stats", "Job counters and script progress", "job");
}

void ROBOT::registerSystemCommands() {
    // "sys": who this machine is, how it is doing, and its lifecycle. Stays
    // in the composition root because it crosses esp_system, esp_ota_ops, the
    // BTP endpoint's identity, SystemMonitor and StateMachine at once -- no
    // single lib can answer "who am I and how am I".
    //
    // Output convention (CONTRIBUTING.md): one line of space-separated
    // key=value pairs, so the console can parse a reply without a per-command
    // rule. The delegating commands below (health/memory/tasks/uptime) are the
    // deliberate exception: they forward SystemMonitor's existing multi-line
    // report verbatim rather than reformatting it.
    shell.create_module("sys", "Machine identity, health and lifecycle");

    shell.add([]() -> uint8_t {
        const esp_app_desc_t* app = esp_app_get_description();

        esp_chip_info_t chip{};
        esp_chip_info(&chip);

        uint32_t flash_size = 0;
        if (esp_flash_get_size(nullptr, &flash_size) != ESP_OK) flash_size = 0;

        const esp_partition_t* running = esp_ota_get_running_partition();
        esp_ota_img_states_t   ota_state = ESP_OTA_IMG_UNDEFINED;
        if (running != nullptr) {
            esp_ota_get_state_partition(running, &ota_state);
        }

        ROBOT::logger.insert_logf(
            logType::INFO,
            "firmware=%s built=%s_%s idf=%s chip=ESP32-S3 rev=%u cores=%u "
            "flash_bytes=%lu partition=%s ota_state=%s",
            app != nullptr ? app->version : "?",
            app != nullptr ? app->date : "?",
            app != nullptr ? app->time : "?",
            app != nullptr ? app->idf_ver : "?",
            static_cast<unsigned>(chip.revision),
            static_cast<unsigned>(chip.cores),
            static_cast<unsigned long>(flash_size),
            running != nullptr ? running->label : "?",
            otaStateName(ota_state));
        return RESULT_OK;
    }, "info", "Firmware version, chip, flash and running OTA partition", "sys");

    shell.add([]() -> uint8_t {
        uint8_t mac[6] = {};
        esp_read_mac(mac, ESP_MAC_WIFI_STA);

        char uuid_hex[sizeof(instance_->protocol_uuid_) * 2U + 1U];
        hexEncode(instance_->protocol_uuid_, sizeof(instance_->protocol_uuid_),
                  uuid_hex);

        static constexpr uint8_t peer[6] = {MAC_ADDR};

        ROBOT::logger.insert_logf(
            logType::INFO,
            "source_id=0x%08lX boot_id=0x%08lX mac=%02X:%02X:%02X:%02X:%02X:%02X "
            "uuid=%s espnow_channel=%u peer=%02X:%02X:%02X:%02X:%02X:%02X",
            static_cast<unsigned long>(instance_->protocol.source_id()),
            static_cast<unsigned long>(instance_->protocol.boot_id()),
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
            uuid_hex,
            static_cast<unsigned>(instance_->settings.data().espnow_channel),
            peer[0], peer[1], peer[2], peer[3], peer[4], peer[5]);
        return RESULT_OK;
    }, "identity", "BTP source_id/boot_id, MAC, uuid, radio channel and peer", "sys");

    shell.add([]() -> uint8_t {
        // update() before reading: with timers.sysmon_freq_ms == 0 nobody
        // else refreshes the task table, and even with the periodic report
        // running, an on-demand read should not show whatever the last cycle
        // happened to leave behind.
        instance_->sysmon.update();
        ROBOT::logger.insert_log(logType::INFO,
                                 instance_->sysmon.getFullReport().c_str());
        return RESULT_OK;
    }, "health", "Full system health report (temperature, memory, tasks)", "sys");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_log(logType::INFO,
                                 instance_->sysmon.getUptime().c_str());
        return RESULT_OK;
    }, "uptime", "Time since boot", "sys");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_log(logType::INFO,
                                 instance_->sysmon.getMemoryStats().c_str());
        return RESULT_OK;
    }, "memory", "Heap and PSRAM usage", "sys");

    shell.add([]() -> uint8_t {
        instance_->sysmon.update();  // same reason as "health" above
        ROBOT::logger.insert_log(logType::INFO,
                                 instance_->sysmon.getTaskStats().c_str());
        return RESULT_OK;
    }, "tasks", "Per-task CPU load, priority, core and stack high water mark", "sys");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_logf(logType::INFO, "core_temp_c=%.1f",
                                  instance_->sysmon.getCoreTemperature());
        return RESULT_OK;
    }, "temp", "SoC core temperature", "sys");

    shell.add([]() -> uint8_t {
        const esp_reset_reason_t reason = esp_reset_reason();
        ROBOT::logger.insert_logf(logType::INFO, "reset_reason=%d name=%s",
                                  static_cast<int>(reason),
                                  resetReasonName(reason));
        return RESULT_OK;
    }, "reset_reason", "Why the chip last restarted (panic, watchdog, ...)", "sys");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_logf(
            logType::INFO,
            "boot_state=%s current_state=%s ota_active=%d usb_exposed=%d",
            StateMachine::stateToString(
                static_cast<uint8_t>(instance_->bootState())),
            StateMachine::stateToString(
                StateMachine::current_state.load(std::memory_order_acquire)),
            instance_->ota.is_active() ? 1 : 0,
            instance_->usb_storage.is_exposed() ? 1 : 0);
        return RESULT_OK;
    }, "boot_mode", "Which state this boot started in, and which sub-mode is active", "sys");

    shell.add([]() -> uint8_t {
        // A successful OTA upload sets the new image as the boot partition but
        // deliberately does not reboot on its own (see
        // OTAUpdater::finish_update) -- this is the remote trigger for that,
        // and the general "restart it" the shell had been missing outside the
        // "ota" module. "ota reboot" is kept as an alias: the dongle and
        // TraceView already depend on that name.
        ROBOT::logger.send_log_direct(logType::INFO, "Rebooting...");
        instance_->sendNextShellOutputDirect();
        scheduleRestart();
        return RESULT_OK;
    }, "reboot", "Restart the robot now", "sys");

    shell.add([](std::string confirmation) -> uint8_t {
        // Destructive, and reachable by radio from anywhere in range, so it
        // takes a literal confirmation word rather than being one typo away.
        // Deliberately not a flag or a number: it has to be typed on purpose.
        if (confirmation != "CONFIRMA") {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "factory_reset refused: pass the literal word CONFIRMA "
                "(sys factory_reset CONFIRMA)");
            return RESULT_ERROR;
        }

        instance_->settings.reset_all();
        if (!instance_->settings.save(instance_->sd_card)) {
            // In-memory defaults are already restored, but they would not
            // survive a reboot -- say so instead of implying success.
            ROBOT::logger.insert_log(
                logType::ERRO,
                "factory_reset: defaults restored in memory but settings.conf "
                "could not be written; NOT rebooting");
            return RESULT_ERROR;
        }

        ROBOT::logger.send_log_direct(
            logType::WARN, "factory_reset: settings.conf rewritten, rebooting");
        instance_->sendNextShellOutputDirect();
        scheduleRestart();
        return RESULT_OK;
    }, "factory_reset", "Restore compiled-in defaults and reboot: pass CONFIRMA", "sys");
}

void ROBOT::registerRobotIOCommands() {
    // Raw actuator/virtual-input I/O: motors, LEDs, virtual button/side-sensor triggers.
    shell.create_module("robot", "Raw actuator and virtual-input commands");

    shell.add([](uint8_t btn_idx) -> uint8_t {
        // set the flag of the button with the given index
        if (btn_idx >= Flags_in::MAX_FLAGS)
            return RESULT_ERROR;
        instance_->buttons.setFlag(btn_idx);
        return RESULT_OK;
    }, "btn", "Virtually trigger a button", "robot");

    shell.add([](uint8_t ssr_idx) -> uint8_t {
        // set the flag of the side sensor with the given index
        if (ssr_idx >= Flags_in::MAX_FLAGS)
            return RESULT_ERROR;
        instance_->sideSensors.setFlag(ssr_idx);
        return RESULT_OK;
    }, "ssr", "Virtually trigger a side sensor", "robot");

    shell.add([](uint8_t led_idx, int8_t pwm_value, uint32_t time) -> uint8_t {
        // set the PWM value for the motor with the given index (-100..100)
        if (led_idx >= Flags_in::MAX_FLAGS)
            return RESULT_ERROR;
        instance_->motors.setValue(led_idx, pwm_value, time);
        return RESULT_OK;
    }, "set_pwm", "Set PWM value for a motor (0 for left, 1 for right)", "robot");

    shell.add([](int8_t left_pwm, int8_t right_pwm, uint32_t time) -> uint8_t {
        instance_->motors.setValue(MOTOR_LEFT_idx, left_pwm, time);
        instance_->motors.setValue(MOTOR_RIGHT_idx, right_pwm, time);
        return RESULT_OK;
    }, "set_pwm_pair", "Set PWM values for both motors at once", "robot");

    shell.add([](uint8_t pin, uint32_t time) -> uint8_t {
        instance_->leds.setFlag(pin, time);
        return RESULT_OK;
    }, "set_led", "turn on a LED", "robot");

    // Not a Flags_pwm command like the ones above: the buzzer needs a
    // frequency in Hz, not a -100..100 duty percentage, so it doesn't fit
    // that struct's semantics. Junkebox::play_tone() gives the same
    // "one-shot, auto-stops after `time`" safety directly instead.
    shell.add([](uint32_t freq_hz, uint32_t time) -> uint8_t {
        if (!instance_->junkebox->play_tone(static_cast<uint16_t>(freq_hz), time)) {
            ROBOT::logger.insert_log(logType::ERRO, "Junkebox is not initialized yet");
            return RESULT_ERROR;
        }
        return RESULT_OK;
    }, "test_buzzer", "Play a raw tone on the buzzer, bypassing note parsing: freq_hz,duration_ms", "robot");
}

void ROBOT::registerKalmanCommands() {
    // EKF (Kalman) state and periodic tuning log. Unlike the "debug" module
    // below, this is available in any state (including RUN) since tuning
    // the filter means watching it while the robot is actually driving.
    shell.create_module("kalman", "EKF state and periodic tuning log");

    shell.add([]() -> uint8_t {
        ROBOT::logger.insert_logf(logType::INFO, "EKF state: v=%.4f w=%.4f b=%.4f",
                                  instance_->EKF->get_state(0),
                                  instance_->EKF->get_state(1),
                                  instance_->EKF->get_state(2));
        return RESULT_OK;
    }, "state", "Read the current EKF state (v, w, gyro bias)", "kalman");

    shell.add([](uint32_t interval_ms) -> uint8_t {
        if (interval_ms == 0 ||
            !instance_->kalman_log_test_.schedule(UINT32_MAX, interval_ms)) {
            ROBOT::logger.insert_log(logType::ERRO,
                                     "Invalid interval; kalman log not started");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(logType::INFO,
                                  "Kalman log started: every %u ms", interval_ms);
        return RESULT_OK;
    }, "start_log", "Start periodic EKF state + measurement logging: interval_ms", "kalman");

    shell.add([]() -> uint8_t {
        instance_->kalman_log_test_.cancel();
        ROBOT::logger.insert_log(logType::INFO, "Kalman log stopped");
        return RESULT_OK;
    }, "stop_log", "Stop periodic EKF logging", "kalman");
}

void ROBOT::registerDebugCommands() {
    // DEBUG sensor tests: each schedules periodic, non-blocking sampling
    // (see ScheduledDebugTest/processDebug). Add one command per sensor
    // here — IMU and H-bridge current are planned next.
    shell.create_module("debug", "Safe non-blocking sensor tests");

    shell.add([](uint32_t samples, uint32_t interval_ms) -> uint8_t {
        if (!instance_->scheduleDebugTest(instance_->array_sensor_test_,
                                          samples, interval_ms)) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "Array test requires DEBUG state, USB inactive and samples > 0");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(
            logType::INFO,
            "Array sensor test scheduled: %u samples every %u ms",
            samples, interval_ms);
        return RESULT_OK;
    }, "test_arr_sensor", "Print sensor array: samples,interval_ms", "debug");

    shell.add([](uint32_t samples, uint32_t interval_ms) -> uint8_t {
        if (!instance_->scheduleDebugTest(instance_->encoder_test_,
                                          samples, interval_ms)) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "Encoder test requires DEBUG state, USB inactive and samples > 0");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(
            logType::INFO,
            "Encoder test scheduled: %u samples every %u ms",
            samples, interval_ms);
        return RESULT_OK;
    }, "test_encoder", "Print left/right encoder counts: samples,interval_ms", "debug");

    shell.add([](uint32_t samples, uint32_t interval_ms) -> uint8_t {
        // Same reasoning as sampleEKF()'s imu_ready_ guard: without it, a
        // sensor that never answered at boot would retry (and block on) an
        // I2C timeout every sample instead of failing once, up front.
        if (!instance_->imu_ready_) {
            ROBOT::logger.insert_log(logType::ERRO,
                                     "IMU not ready; nothing was detected at boot");
            return RESULT_ERROR;
        }

        if (!instance_->scheduleDebugTest(instance_->imu_test_,
                                          samples, interval_ms)) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "IMU test requires DEBUG state, USB inactive and samples > 0");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(
            logType::INFO,
            "IMU test scheduled: %u samples every %u ms",
            samples, interval_ms);
        return RESULT_OK;
    }, "test_imu", "Print IMU accel/gyro/temp readings: samples,interval_ms", "debug");

    shell.add([]() -> uint8_t {
        // Reuses the bus imu->begin() already opened on cfg.sda_pin/scl_pin
        // instead of opening a second i2c_master_bus on the same pins (see
        // ICM42688::scanBus()'s comment) -- so this works on demand, without
        // a reboot, even when imu_ready_ is false.
        if (!instance_->imu->busOpen()) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "IMU I2C bus never opened (begin() failed before that point); nothing to scan");
            return RESULT_ERROR;
        }

        std::string found;
        const bool  any_found = instance_->imu->scanBus(found);
        ROBOT::logger.insert_logf(
            logType::INFO,
            "I2C scan (IMU bus): %s | WHO_AM_I=0x%02X (expect 0x47)",
            any_found ? found.c_str() : "no devices found",
            instance_->imu->whoAmI());
        return RESULT_OK;
    }, "scan_i2c", "Probe the IMU I2C bus and read WHO_AM_I; works even if the IMU wasn't detected at boot", "debug");

    shell.add([](uint32_t samples, uint32_t interval_ms) -> uint8_t {
        // Same reuse-the-existing-bus reasoning as scan_i2c: no imu_ready_
        // gate here on purpose -- the whole point is to keep re-checking
        // even after a failed boot detection, since the bus can (and does,
        // on a noisy/pull-up-less setup) recover a moment later.
        if (!instance_->imu->busOpen()) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "IMU I2C bus never opened (begin() failed before that point); nothing to check");
            return RESULT_ERROR;
        }

        instance_->imu_i2c_ok_count_   = 0;
        instance_->imu_i2c_fail_count_ = 0;
        if (!instance_->scheduleDebugTest(instance_->imu_i2c_test_,
                                          samples, interval_ms)) {
            ROBOT::logger.insert_log(
                logType::ERRO,
                "I2C stability test requires DEBUG state, USB inactive and samples > 0");
            return RESULT_ERROR;
        }

        ROBOT::logger.insert_logf(
            logType::INFO,
            "I2C stability test scheduled: %u checks every %u ms",
            samples, interval_ms);
        return RESULT_OK;
    }, "test_i2c", "Re-check WHO_AM_I on a timer and tally ok/fail, to watch bus stability live: samples,interval_ms", "debug");
}
