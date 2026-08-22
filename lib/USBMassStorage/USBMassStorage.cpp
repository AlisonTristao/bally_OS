#include <USBMassStorage.h>

#include <SDCard.h>
#include "esp_mac.h"
#include "esp_timer.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"

#include <cstdio>
#include <Flags.h>
#include <TinyShell.h>
#include <Logger.h>
#include <Format.h>
#include <StateMachine.h>
// LED0_idx/LED1_idx/LED2_idx/LED3_idx only, for update_status_led()'s blink
// pattern (see CONTRIBUTING.md on why Settings.h isn't pulled into the
// header for this).
#include <Settings.h>

namespace {

constexpr char USB_HEX_DIGITS[] = "0123456789ABCDEF";
const char USB_LANGUAGE_DESCRIPTOR[] = {0x09, 0x04};

// Storage-mode LED pattern: LED0/LED2 and LED1/LED3 alternate every
// kStorageBlinkPeriodMs; kStorageLedHoldMs is the refresh window kept lit
// each pass, mirroring OtaDefaults::led_hold_ms, so the previous pair's
// Flags_out entry auto-expires (see Flags::checkFlagsDuration) shortly after
// each swap instead of staying lit.
constexpr uint32_t kStorageBlinkPeriodMs = 1000;
constexpr uint32_t kStorageLedHoldMs     = 200;

uint32_t now_ms() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

} // namespace

static void usb_device_event(tinyusb_event_t* event, void* context) {
    if (event == nullptr || context == nullptr) return;

    auto* storage = static_cast<USBMassStorage*>(context);
    if (event->id == TINYUSB_EVENT_ATTACHED) {
        storage->handle_host_connection(true);
    } else if (event->id == TINYUSB_EVENT_DETACHED) {
        storage->handle_host_connection(false);
    }
}

void usb_storage_event(tinyusb_msc_storage_handle_t handle,
                       tinyusb_msc_event_t* event,
                       void* context) {
    (void)handle;

    if (event == nullptr || context == nullptr) return;

    auto* storage = static_cast<USBMassStorage*>(context);
    storage->handle_storage_event(event);
}

USBMassStorage::~USBMassStorage() {
    if (storage_handle_ == nullptr) return;

    auto handle = static_cast<tinyusb_msc_storage_handle_t>(storage_handle_);

    if (card_ != nullptr) card_->close_stream();

    // The destructor runs before SDCard, so return ownership and unmount FAT
    // before the physical SDSPI device is released.
    tinyusb_msc_set_storage_mount_point(
        handle, TINYUSB_MSC_STORAGE_MOUNT_APP);
    sync_mount_state();

    if (usb_driver_installed_.load()) {
        tinyusb_driver_uninstall();
        usb_driver_installed_.store(false);
    }

    tinyusb_msc_delete_storage(handle);
    tinyusb_msc_uninstall_driver();
    storage_handle_ = nullptr;
    initialized_.store(false);
    app_has_access_.store(false);
    session_active_.store(false);
    host_attached_.store(false);

    if (card_ != nullptr) card_->set_app_mounted(false);
}

bool USBMassStorage::begin(SDCard& card, Flags_out& leds) {
    if (initialized_.load()) return true;
    if (card.card_handle() == nullptr || card.mount_point() == nullptr) {
        return false;
    }
    if (!prepare_usb_identity()) return false;

    leds_ = &leds;

    // Own every mount-point transition ourselves. Left at its default,
    // esp_tinyusb's MSC driver auto-returns the card to the app on
    // tud_umount_cb(), which fires not only on a real unplug but also when
    // the host issues SetConfiguration(0) -- something Windows can do
    // mid-enumeration without the cable ever moving. That silently yanks the
    // FAT filesystem away from the PC while it is still trying to read the
    // MBR/boot sector, which is why the drive shows the right capacity but
    // never gets a stable volume Windows can assign a letter to.
    tinyusb_msc_driver_config_t msc_driver_config{};
    msc_driver_config.user_flags.auto_mount_off = 1;
    msc_driver_config.callback = usb_storage_event;
    msc_driver_config.callback_arg = this;
    if (tinyusb_msc_install_driver(&msc_driver_config) != ESP_OK) return false;

    tinyusb_msc_storage_config_t config{};
    config.medium.card = card.card_handle();
    config.fat_fs.base_path = const_cast<char*>(card.mount_point());
    config.fat_fs.config.format_if_mount_failed = false;
    config.fat_fs.config.max_files = 5;
    // esp_tinyusb's tinyusb_msc_new_storage_sdmmc() never copies
    // config.fat_fs.config.allocation_unit_size into its internal state, and
    // its own format path (vfs_fat_format() in tinyusb_msc.c) derives the
    // cluster size from CONFIG_WL_SECTOR_SIZE instead, so this field has no
    // effect on a blank card's actual cluster size (~4 KB here) -- left unset
    // rather than implying a size we don't actually control.
    config.fat_fs.do_not_format = false;
    // FM_ANY | FM_SFD (0x07 | 0x08): format as a superfloppy (FAT32 boot
    // sector directly at LBA0, no MBR/partition table) instead of FatFs's
    // default of a partitioned disk -- the same layout virtually every
    // commercial USB flash drive uses.
    config.fat_fs.format_flags = 0x07 | 0x08;
    config.mount_point = TINYUSB_MSC_STORAGE_MOUNT_APP;

    tinyusb_msc_storage_handle_t handle = nullptr;
    if (tinyusb_msc_new_storage_sdmmc(&config, &handle) != ESP_OK) {
        tinyusb_msc_uninstall_driver();
        return false;
    }

    card_ = &card;
    storage_handle_ = handle;
    mount_transition_failed_.store(false);
    initialized_.store(true);

    uint64_t total_bytes = 0;
    uint64_t used_bytes = 0;
    uint64_t free_bytes = 0;
    // Re-check the MBR signature here too: a blank card only gets its FAT
    // filesystem (and FatFs's zero-signature MBR) created by
    // tinyusb_msc_new_storage_sdmmc() just above, after the earlier fix-up in
    // SDCard::begin() already ran and had nothing yet to repair.
    if (!sync_mount_state() || !app_has_access_.load() ||
        !card.get_storage_info(total_bytes, used_bytes, free_bytes) ||
        !card.ensure_mbr_signature()) {
        tinyusb_msc_delete_storage(handle);
        tinyusb_msc_uninstall_driver();
        storage_handle_ = nullptr;
        card_ = nullptr;
        initialized_.store(false);
        return false;
    }

    return true;
}

bool USBMassStorage::expose() {
    if (!initialized_.load() || storage_handle_ == nullptr ||
        session_active_.load() || !sync_mount_state() ||
        !app_has_access_.load() || card_ == nullptr ||
        card_->has_open_stream()) {
        return false;
    }

    auto handle = static_cast<tinyusb_msc_storage_handle_t>(storage_handle_);
    mount_transition_failed_.store(false);

    if (tinyusb_msc_set_storage_mount_point(
            handle, TINYUSB_MSC_STORAGE_MOUNT_USB) != ESP_OK ||
        mount_transition_failed_.load() || !sync_mount_state() ||
        app_has_access_.load()) {
        return false;
    }

    // Install the native USB task only for an explicitly requested session.
    // This prevents a connected PC from taking the card during normal states.
    host_attached_.store(false);

    tinyusb_config_t usb_config =
        TINYUSB_DEFAULT_CONFIG(usb_device_event, this);
    usb_config.descriptor.string = usb_string_descriptors_;
    usb_config.descriptor.string_count = 5;
    if (tinyusb_driver_install(&usb_config) != ESP_OK) {
        tinyusb_msc_set_storage_mount_point(
            handle, TINYUSB_MSC_STORAGE_MOUNT_APP);
        sync_mount_state();
        return false;
    }

    usb_driver_installed_.store(true);
    session_active_.store(true);
    // Always start the blink pattern from the same phase (LED0/LED2 first).
    blink_alt_ = false;
    blink_next_ms_ = now_ms() + kStorageBlinkPeriodMs;
    return true;
}

bool USBMassStorage::reclaim() {
    if (!session_active_.load() || storage_handle_ == nullptr) return false;

    auto handle = static_cast<tinyusb_msc_storage_handle_t>(storage_handle_);

    tinyusb_msc_set_storage_mount_point(handle, TINYUSB_MSC_STORAGE_MOUNT_APP);
    sync_mount_state();

    if (usb_driver_installed_.load()) {
        tinyusb_driver_uninstall();
        usb_driver_installed_.store(false);
    }

    host_attached_.store(false);
    session_active_.store(false);

    // Same "set for 1ms" trick as OTAUpdater::cancel(): lets Flags_out's own
    // auto-clear turn every LED off on the next pass instead of leaving
    // whichever pair was lit stuck on.
    if (leds_ != nullptr) {
        for (uint8_t i = 0; i < 4; ++i) leds_->setFlag(i, 1);
    }

    return app_has_access_.load();
}

void USBMassStorage::process(uint8_t button_flags) {
    if (!session_active_.load() || storage_handle_ == nullptr) return;

    // Do not auto-uninstall the native USB driver here: a DETACHED event can
    // also be raised by SetConfiguration(0) without the cable being removed.
    sync_mount_state();
    update_status_led();

    if ((button_flags & (1 << BIT_2)) != 0) {
        reclaim();
    }
}

void USBMassStorage::update_status_led() {
    if (leds_ == nullptr) return;

    const uint32_t now = now_ms();
    if (static_cast<int32_t>(now - blink_next_ms_) >= 0) {
        blink_alt_ = !blink_alt_;
        blink_next_ms_ = now + kStorageBlinkPeriodMs;
    }

    // "on - off - on - off" (blink_alt_ false) then, one second later,
    // "off - on - off - on" (blink_alt_ true): only the lit pair needs
    // refreshing each pass, the other expires on its own.
    if (blink_alt_) {
        leds_->setFlag(LED1_idx, kStorageLedHoldMs);
        leds_->setFlag(LED3_idx, kStorageLedHoldMs);
    } else {
        leds_->setFlag(LED0_idx, kStorageLedHoldMs);
        leds_->setFlag(LED2_idx, kStorageLedHoldMs);
    }
}

void USBMassStorage::handle_host_connection(bool attached) {
    host_attached_.store(attached);
}

void USBMassStorage::handle_storage_event(void* event_ptr) {
    if (event_ptr == nullptr) return;

    auto* event = static_cast<tinyusb_msc_event_t*>(event_ptr);

    switch (event->id) {
        case TINYUSB_MSC_EVENT_MOUNT_START:
            mount_transition_failed_.store(false);
            break;
        case TINYUSB_MSC_EVENT_MOUNT_FAILED:
            mount_transition_failed_.store(true);
            break;
        case TINYUSB_MSC_EVENT_MOUNT_COMPLETE:
            mount_transition_failed_.store(false);
            break;
        default:
            break;
    }
}

bool USBMassStorage::prepare_usb_identity() {
    uint8_t mac[6];
    if (esp_efuse_mac_get_default(mac) != ESP_OK) return false;

    // A stable per-robot serial prevents Windows from reusing the same
    // "123456" storage record for every ESP32-S3 and every firmware attempt.
    for (uint8_t index = 0; index < sizeof(mac); ++index) {
        usb_serial_[index * 2] = USB_HEX_DIGITS[mac[index] >> 4];
        usb_serial_[index * 2 + 1] = USB_HEX_DIGITS[mac[index] & 0x0F];
    }
    usb_serial_[12] = '\0';

    usb_string_descriptors_[0] = USB_LANGUAGE_DESCRIPTOR;
    usb_string_descriptors_[1] = CONFIG_TINYUSB_DESC_MANUFACTURER_STRING;
    usb_string_descriptors_[2] = CONFIG_TINYUSB_DESC_PRODUCT_STRING;
    usb_string_descriptors_[3] = usb_serial_;
    usb_string_descriptors_[4] = CONFIG_TINYUSB_DESC_MSC_STRING;
    return true;
}

uint64_t USBMassStorage::capacity_bytes() const {
    if (storage_handle_ == nullptr) return 0;

    uint32_t sector_count = 0;
    uint32_t sector_size = 0;
    auto handle = static_cast<tinyusb_msc_storage_handle_t>(storage_handle_);

    if (tinyusb_msc_get_storage_capacity(handle, &sector_count) != ESP_OK ||
        tinyusb_msc_get_storage_sector_size(handle, &sector_size) != ESP_OK) {
        return 0;
    }

    return static_cast<uint64_t>(sector_count) * sector_size;
}

bool USBMassStorage::sync_mount_state() {
    if (storage_handle_ == nullptr || card_ == nullptr) return false;

    tinyusb_msc_mount_point_t mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB;
    const bool success = tinyusb_msc_get_storage_mount_point(
        static_cast<tinyusb_msc_storage_handle_t>(storage_handle_),
        &mount_point) == ESP_OK;

    if (!success) return false;

    const bool app_access =
        mount_point == TINYUSB_MSC_STORAGE_MOUNT_APP;
    app_has_access_.store(app_access);
    card_->set_app_mounted(app_access);
    return true;
}

// ============================================================================
// Shell commands
// ============================================================================

void USBMassStorage::register_shell_commands(TinyShell& shell, Logger& logger, SDCard& sd_card,
                                             std::function<bool()> any_debug_test_active,
                                             std::function<void()> mark_direct_output) {
    shell.create_module("storage", "SD card file management and USB MSC ownership");

    shell.add([this, &logger, &sd_card, any_debug_test_active, mark_direct_output]() -> uint8_t {
        if (StateMachine::current_state.load(std::memory_order_acquire) !=
            DEBUG) {
            logger.insert_log(
                logType::ERRO,
                "USB storage requires DEBUG state; enter DEBUG first");
            return RESULT_ERROR;
        }

        if (is_exposed()) {
            logger.send_log_direct(logType::INFO,
                                   "USB storage is already enabled");
            return RESULT_OK;
        }

        if (!is_ready() ||
            !app_has_access() ||
            !sd_card.is_mounted()) {
            logger.insert_log(
                logType::ERRO,
                "USB storage unavailable: SD card is not mounted for robot");
            return RESULT_ERROR;
        }

        if (sd_card.has_open_stream()) {
            logger.insert_log(
                logType::ERRO,
                "USB storage blocked: close the active SD file first");
            return RESULT_ERROR;
        }

        if (any_debug_test_active()) {
            logger.insert_log(
                logType::ERRO,
                "USB storage blocked: wait for the DEBUG test to finish");
            return RESULT_ERROR;
        }

        if (!expose()) {
            logger.insert_log(
                logType::ERRO,
                "Failed to transfer SD card ownership to native USB");
            return RESULT_ERROR;
        }

        logger.send_log_direct(
            logType::INFO,
            "USB storage enabled; press button 2 on the robot to give the SD card back "
            "(the PC's own safe-eject no longer reclaims it automatically)");
        mark_direct_output();
        return RESULT_OK;
    }, "expose", "Expose the SD card through native USB MSC (DEBUG state)", "storage");

    shell.add([this, &logger, mark_direct_output]() -> uint8_t {
        if (!is_ready()) {
            logger.insert_log(logType::ERRO,
                              "USB storage is not initialized");
            return RESULT_ERROR;
        }

        const char* owner = nullptr;
        if (is_exposed()) {
            owner = "PC owns SD";
        } else if (is_active()) {
            owner = host_is_attached()
                ? "PC connected, waiting for media"
                : "waiting for PC connection";
        } else {
            owner = "robot owns SD";
        }

        char capacity_text[24];
        formatBytes(capacity_bytes(), capacity_text, sizeof(capacity_text));
        char status[96];
        snprintf(status, sizeof(status), "USB storage: %s; media=%s",
                 owner, capacity_text);

        logger.send_log_direct(logType::INFO, status);
        mark_direct_output();
        return RESULT_OK;
    }, "status", "Show current SD card owner", "storage");
}
