#ifndef USB_MASS_STORAGE_H
#define USB_MASS_STORAGE_H

// autor: Alison Tristao
// email: AlisonTristao@hotmail.com

#include <atomic>
#include <cstdint>
#include <functional>

#include "tinyusb_msc.h"

class SDCard;
class TinyShell;
class Logger;

/**
 * @brief Transfer exclusive SD card access between the robot and a USB host.
 *
 * The PC and the application are never allowed to mount the FAT filesystem at
 * the same time. The native ESP32-S3 USB peripheral uses GPIO19/20 internally.
 */
class USBMassStorage {
public:
    USBMassStorage() = default;
    ~USBMassStorage();

    USBMassStorage(const USBMassStorage&) = delete;
    USBMassStorage& operator=(const USBMassStorage&) = delete;

    /**
     * @brief Register an initialized SD card and mount it for the robot.
     */
    bool begin(SDCard& card);

    /**
     * @brief Unmount the card from the robot and expose it through USB MSC.
     */
    bool expose();

    /**
     * @brief Process USB eject/disconnect without blocking the state task.
     */
    void process();

    bool is_ready() const { return initialized_.load(); }
    bool is_exposed() const {
        return session_active_.load() && !app_has_access_.load();
    }
    bool is_active() const {
        return session_active_.load() || usb_driver_installed_.load();
    }
    bool app_has_access() const { return app_has_access_.load(); }
    bool host_is_attached() const { return host_attached_.load(); }
    uint64_t capacity_bytes() const;

    // Called by the TinyUSB device event bridge in the implementation file.
    void handle_host_connection(bool attached);

    /**
     * @brief Register this manager's "storage" shell module commands
     * (expose/status) — SD/USB ownership transfer, as opposed to SDCard's
     * plain file management (usage/list_logs/...), which share the same
     * "storage" module name.
     * @param any_debug_test_active Polled by "expose"; true while a DEBUG
     * sensor test (owned by ROBOT, not this class) is running.
     * @param mark_direct_output Called after "expose"/"status" send their
     * reply, so it is not itself retained in the PSRAM log (see
     * ROBOT::sendNextShellOutputDirect).
     */
    void register_shell_commands(TinyShell& shell, Logger& logger, SDCard& sd_card,
                                 std::function<bool()> any_debug_test_active,
                                 std::function<void()> mark_direct_output);

private:
    friend void usb_storage_event(tinyusb_msc_storage_handle_t handle,
                                  tinyusb_msc_event_t* event,
                                  void* context);

    SDCard* card_ = nullptr;
    void* storage_handle_ = nullptr;
    char usb_serial_[13]{};
    const char* usb_string_descriptors_[5]{};
    std::atomic<bool> initialized_{false};
    std::atomic<bool> app_has_access_{false};
    std::atomic<bool> usb_driver_installed_{false};
    std::atomic<bool> session_active_{false};
    std::atomic<bool> host_attached_{false};
    std::atomic<bool> mount_transition_failed_{false};

    bool prepare_usb_identity();
    bool sync_mount_state();
    void handle_storage_event(void* event);
};

#endif // USB_MASS_STORAGE_H
