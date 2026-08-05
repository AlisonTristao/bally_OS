#include <USBMassStorage.h>

#include <SDCard.h>
#include "esp_mac.h"
#include "esp_timer.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"

namespace {

constexpr char USB_HEX_DIGITS[] = "0123456789ABCDEF";
const char USB_LANGUAGE_DESCRIPTOR[] = {0x09, 0x04};

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

bool USBMassStorage::begin(SDCard& card) {
    if (initialized_.load()) return true;
    if (card.card_handle() == nullptr || card.mount_point() == nullptr) {
        return false;
    }
    if (!prepare_usb_identity()) return false;

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
    config.fat_fs.config.allocation_unit_size = 16 * 1024;
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
    return true;
}

void USBMassStorage::process() {
    if (!session_active_.load() || storage_handle_ == nullptr) return;

    // Do not auto-uninstall the native USB driver here: a DETACHED event can
    // also be raised by SetConfiguration(0) without the cable being removed.
    sync_mount_state();
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
