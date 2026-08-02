#ifndef SD_CARD_H
#define SD_CARD_H

// autor: Alison Tristao
// email: AlisonTristao@hotmail.com

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include "driver/gpio.h"
#include "sdmmc_cmd.h"

struct SDFileInfo {
    static constexpr size_t MAX_NAME_LENGTH = 128;

    char name[MAX_NAME_LENGTH];
    uint64_t size;
};

/**
 * @brief Manage an SD card connected through the SPI bus.
 *
 * The class mounts a FAT filesystem and provides basic binary file operations.
 * Paths can be relative to the mount point or include the mount point itself.
 */
class SDCard {
public:
    /**
     * @brief Create an SD card manager.
     *
     * @param miso SPI MISO pin connected to the card.
     * @param sck SPI clock pin connected to the card.
     * @param mosi SPI MOSI pin connected to the card.
     * @param cs SPI chip-select pin connected to the card.
     * @param mount_point FAT filesystem mount point.
     */
    SDCard(gpio_num_t miso, gpio_num_t sck, gpio_num_t mosi, gpio_num_t cs,
           const char* mount_point = "/sdcard");

    /**
     * @brief Unmount the card and release the SPI bus owned by this object.
     */
    ~SDCard();

    SDCard(const SDCard&) = delete;
    SDCard& operator=(const SDCard&) = delete;

    /**
     * @brief Initialize the SPI bus and the physical SD card.
     *
     * USBMassStorage mounts the initialized card for application access and
     * later transfers exclusive ownership between the robot and the USB host.
     *
     * @return true when the physical card is initialized.
     */
    bool begin();

    /**
     * @brief Unmount the filesystem and release the SPI bus when owned.
     */
    void end();

    /**
     * @brief Check whether the card is mounted.
     */
    bool is_mounted() const { return mounted_; }

    /**
     * @brief Check whether a sequential file is currently open.
     */
    bool has_open_stream() const { return stream_ != nullptr; }

    /**
     * @brief Get the filesystem mount point.
     */
    const char* mount_point() const { return mount_point_; }

    /**
     * @brief Create a file without erasing its existing content.
     *
     * @param path Relative path or complete path below the mount point.
     * @return true when the file exists and can be opened.
     */
    bool create_file(const char* path);

    /**
     * @brief Create or overwrite a file with binary data.
     *
     * @param path Relative path or complete path below the mount point.
     * @param data Bytes to write. It may be null when length is zero.
     * @param length Number of bytes to write.
     * @return true when all bytes are written, flushed and the file is closed.
     */
    bool write_file(const char* path, const void* data, size_t length);

    /**
     * @brief Append binary data to the end of a file.
     *
     * The file is created when it does not exist.
     *
     * @param path Relative path or complete path below the mount point.
     * @param data Bytes to append. It may be null when length is zero.
     * @param length Number of bytes to append.
     * @return true when all bytes are written, flushed and the file is closed.
     */
    bool append_file(const char* path, const void* data, size_t length);

    /**
     * @brief Read up to capacity bytes from the beginning of a file.
     *
     * This is a binary read and does not add a null terminator.
     *
     * @param path Relative path or complete path below the mount point.
     * @param buffer Destination buffer.
     * @param capacity Maximum number of bytes to read.
     * @param bytes_read Optional output with the number of bytes read.
     * @return true when the read completes without a file error.
     */
    bool read_file(const char* path, void* buffer, size_t capacity,
                   size_t* bytes_read = nullptr) const;

    /**
     * @brief Remove a file from the card.
     *
     * @return true when the file is removed.
     */
    bool remove_file(const char* path);

    /**
     * @brief Rename or move a file inside the mounted filesystem.
     *
     * @return true when the file is renamed.
     */
    bool rename_file(const char* current_path, const char* new_path);

    /**
     * @brief Check whether a regular file exists.
     */
    bool file_exists(const char* path) const;

    /**
     * @brief Read filesystem capacity information.
     */
    bool get_storage_info(uint64_t& total_bytes, uint64_t& used_bytes,
                          uint64_t& free_bytes) const;

    /**
     * @brief Count regular files stored at the root of the card.
     */
    uint16_t get_file_count() const;

    /**
     * @brief Get the name and size of a root file using its list index.
     */
    bool get_file_info(uint16_t index, SDFileInfo& info) const;

    /**
     * @brief Open one file for sequential access.
     *
     * @param append true to preserve content; false to create/overwrite.
     */
    bool open_write_stream(const char* path, bool append);
    bool open_read_stream(const char* path);
    bool write_stream(const void* data, size_t length);
    size_t read_stream(void* buffer, size_t capacity);
    bool close_stream();
    bool stream_has_error() const;

private:
    friend class USBMassStorage;

    // Maximum complete path accepted by the file helpers.
    static constexpr size_t MAX_PATH_LENGTH = 256;

    // SPI pins used by the SD card.
    gpio_num_t miso_;
    gpio_num_t sck_;
    gpio_num_t mosi_;
    gpio_num_t cs_;

    // VFS mount point. The pointed string must remain valid for this object's lifetime.
    const char* mount_point_;

    // Handles and lifecycle state owned by the ESP-IDF SD/FAT drivers.
    sdmmc_card_t card_storage_{};
    sdmmc_card_t* card_ = nullptr;
    int host_slot_ = -1;
    bool bus_owned_ = false;
    bool device_attached_ = false;
    bool mounted_ = false;
    FILE* stream_ = nullptr;
    bool stream_writable_ = false;

    /**
     * @brief Convert a relative path into a complete VFS path.
     *
     * @return false when the card is not mounted or the path is invalid/too long.
     */
    bool make_path(const char* path, char* full_path, size_t capacity) const;

    /**
     * @brief Give an MBR-formatted card a stable identity for USB hosts.
     *
     * FatFs can create a valid partition table with a zero disk signature.
     * Windows can read that volume, but its Mount Manager cannot persist a
     * Volume GUID or drive letter reliably. Existing non-zero signatures and
     * super-floppy formatted cards are left untouched.
     */
    bool ensure_mbr_signature();

    // USBMassStorage updates this flag whenever filesystem ownership changes.
    void set_app_mounted(bool mounted) { mounted_ = mounted; }
    sdmmc_card_t* card_handle() const { return card_; }
};

#endif // SD_CARD_H
