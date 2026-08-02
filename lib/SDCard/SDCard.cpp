#include <SDCard.h>

#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_vfs_fat.h"

#include <cstdio>
#include <cstring>
#include <sys/stat.h>

// ============================================================================
// Lifecycle
// ============================================================================

SDCard::SDCard(gpio_num_t miso, gpio_num_t sck, gpio_num_t mosi, gpio_num_t cs,
               const char* mount_point)
    : miso_(miso),
      sck_(sck),
      mosi_(mosi),
      cs_(cs),
      mount_point_(mount_point) {}

SDCard::~SDCard() {
    end();
}

bool SDCard::begin() {
    // Avoid mounting the same card more than once.
    if (mounted_) return true;
    if (mount_point_ == nullptr || mount_point_[0] != '/') return false;

    // SDSPI_HOST_DEFAULT selects the default general-purpose SPI peripheral.
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    // Request the maximum High Speed clock supported by the SDSPI driver (40 MHz).
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    const spi_host_device_t host_slot =
        static_cast<spi_host_device_t>(host.slot);
    host_slot_ = host.slot;

    // Configure only the signals required by a standard SPI SD card module.
    spi_bus_config_t bus_config{};
    bus_config.mosi_io_num = mosi_;
    bus_config.miso_io_num = miso_;
    bus_config.sclk_io_num = sck_;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 4096;

    const esp_err_t bus_result =
        spi_bus_initialize(host_slot, &bus_config, SDSPI_DEFAULT_DMA);

    if (bus_result == ESP_OK) {
        bus_owned_ = true;
    } else if (bus_result != ESP_ERR_INVALID_STATE) {
        host_slot_ = -1;
        return false;
    }
    // ESP_ERR_INVALID_STATE means that this SPI bus was already initialized.
    // In that case the card can use the existing bus, but it must not free it.

    // Configure the SD device on the selected SPI bus.
    sdspi_device_config_t device_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    device_config.gpio_cs = cs_;
    device_config.host_id = host_slot;

    // Never format automatically: a mount error must not erase stored logs.
    esp_vfs_fat_mount_config_t mount_config{};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;

    const esp_err_t mount_result = esp_vfs_fat_sdspi_mount(
        mount_point_, &host, &device_config, &mount_config, &card_);

    if (mount_result != ESP_OK) {
        card_ = nullptr;

        // Release only a bus initialized by this object.
        if (bus_owned_) {
            spi_bus_free(host_slot);
            bus_owned_ = false;
        }

        host_slot_ = -1;
        return false;
    }

    mounted_ = true;
    return true;
}

void SDCard::end() {
    // Unregister the FAT filesystem before releasing the physical SPI bus.
    if (mounted_) {
        esp_vfs_fat_sdcard_unmount(mount_point_, card_);
        mounted_ = false;
        card_ = nullptr;
    }

    if (bus_owned_ && host_slot_ >= 0) {
        spi_bus_free(static_cast<spi_host_device_t>(host_slot_));
        bus_owned_ = false;
    }

    host_slot_ = -1;
}

// ============================================================================
// Path handling
// ============================================================================

bool SDCard::make_path(const char* path, char* full_path, size_t capacity) const {
    if (!mounted_ || path == nullptr || path[0] == '\0' ||
        full_path == nullptr || capacity == 0) {
        return false;
    }

    const size_t mount_length = strlen(mount_point_);
    int written = 0;

    // Accept "/sdcard/log.bin", "/log.bin" and "log.bin".
    if (strncmp(path, mount_point_, mount_length) == 0 &&
        (path[mount_length] == '/' || path[mount_length] == '\0')) {
        written = snprintf(full_path, capacity, "%s", path);
    } else if (path[0] == '/') {
        written = snprintf(full_path, capacity, "%s%s", mount_point_, path);
    } else {
        written = snprintf(full_path, capacity, "%s/%s", mount_point_, path);
    }

    return written > 0 && static_cast<size_t>(written) < capacity;
}

// ============================================================================
// File operations
// ============================================================================

bool SDCard::create_file(const char* path) {
    char full_path[MAX_PATH_LENGTH];
    if (!make_path(path, full_path, sizeof(full_path))) return false;

    // Append mode creates the file without truncating it when it already exists.
    FILE* file = fopen(full_path, "ab");
    if (file == nullptr) return false;

    return fclose(file) == 0;
}

bool SDCard::write_file(const char* path, const void* data, size_t length) {
    if (data == nullptr && length > 0) return false;

    char full_path[MAX_PATH_LENGTH];
    if (!make_path(path, full_path, sizeof(full_path))) return false;

    // Write mode creates the file or replaces all previous content.
    FILE* file = fopen(full_path, "wb");
    if (file == nullptr) return false;

    const bool written = length == 0 || fwrite(data, 1, length, file) == length;
    const bool flushed = written && fflush(file) == 0;
    const bool closed = fclose(file) == 0;

    return written && flushed && closed;
}

bool SDCard::append_file(const char* path, const void* data, size_t length) {
    if (data == nullptr && length > 0) return false;

    char full_path[MAX_PATH_LENGTH];
    if (!make_path(path, full_path, sizeof(full_path))) return false;

    // Append mode preserves previous logs and writes new bytes at the end.
    FILE* file = fopen(full_path, "ab");
    if (file == nullptr) return false;

    const bool written = length == 0 || fwrite(data, 1, length, file) == length;
    const bool flushed = written && fflush(file) == 0;
    const bool closed = fclose(file) == 0;

    return written && flushed && closed;
}

bool SDCard::read_file(const char* path, void* buffer, size_t capacity,
                       size_t* bytes_read) const {
    if (buffer == nullptr || capacity == 0) return false;
    if (bytes_read != nullptr) *bytes_read = 0;

    char full_path[MAX_PATH_LENGTH];
    if (!make_path(path, full_path, sizeof(full_path))) return false;

    FILE* file = fopen(full_path, "rb");
    if (file == nullptr) return false;

    // Read raw bytes; text callers are responsible for adding a null terminator.
    const size_t read = fread(buffer, 1, capacity, file);
    const bool success = ferror(file) == 0;
    const bool closed = fclose(file) == 0;

    if (bytes_read != nullptr) *bytes_read = read;
    return success && closed;
}

bool SDCard::remove_file(const char* path) {
    char full_path[MAX_PATH_LENGTH];
    if (!make_path(path, full_path, sizeof(full_path))) return false;

    return remove(full_path) == 0;
}

bool SDCard::rename_file(const char* current_path, const char* new_path) {
    char current_full_path[MAX_PATH_LENGTH];
    char new_full_path[MAX_PATH_LENGTH];

    if (!make_path(current_path, current_full_path, sizeof(current_full_path)) ||
        !make_path(new_path, new_full_path, sizeof(new_full_path))) {
        return false;
    }

    return rename(current_full_path, new_full_path) == 0;
}

bool SDCard::file_exists(const char* path) const {
    char full_path[MAX_PATH_LENGTH];
    if (!make_path(path, full_path, sizeof(full_path))) return false;

    struct stat info{};
    return stat(full_path, &info) == 0 && S_ISREG(info.st_mode);
}
