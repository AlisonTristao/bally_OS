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
class Flags_out;

/**
 * @brief Transfer exclusive SD card access between the robot and a USB host.
 *
 * The PC and the application are never allowed to mount the FAT filesystem at
 * the same time. The native ESP32-S3 USB peripheral uses GPIO19/20 internally.
 *
 * While the card is exposed (see is_active()), the LEDs report it the same
 * way OTAUpdater reports its own sub-mode: LED0/LED2 and LED1/LED3 blink in
 * antiphase, one second per half — see update_status_led().
 */
class USBMassStorage {
public:
    USBMassStorage() = default;
    ~USBMassStorage();

    USBMassStorage(const USBMassStorage&) = delete;
    USBMassStorage& operator=(const USBMassStorage&) = delete;

    /**
     * @brief Register an initialized SD card and mount it for the robot.
     * @param leds LED outputs used to report storage-mode status while
     * exposed (see update_status_led()); mirrors OTAUpdater::begin().
     */
    bool begin(SDCard& card, Flags_out& leds);

    /**
     * @brief Unmount the card from the robot and expose it through USB MSC.
     */
    bool expose();

    /**
     * @brief Install the native USB device now and keep it installed for
     * the rest of this boot -- the direct-serial comm_mode (comm_mode==3)
     * needs its CDC interface up from boot, not only during a storage
     * session. Works with or without begin() having succeeded (no SD card
     * just means the MSC LUN reports no medium). expose()/reclaim() keep
     * working on top of it: they only move the FAT mount point, and
     * reclaim() never uninstalls a device installed this way.
     *
     * The TinyUSB task runs on PRO_CPU at priority 4 with an 8 KiB stack
     * here: the CDC RX callback runs the whole BTP receive path (COBS,
     * AEAD open, btp::Node replies) on it, and APP_CPU stays reserved for
     * the state machine (CONTRIBUTING.md). Idempotent.
     */
    bool install_persistent_usb_device();

    /**
     * @brief Report `name` (the robot's configured identity name) as the USB
     * product string and the CDC interface string, so a host lists the
     * serial port under the robot's name -- TraceView's port picker shows
     * "COM5 -- <name>" without opening the port (Windows reads the CDC
     * interface string, Linux/macOS/Android the product string). Same name
     * BleBtpServer advertises. Empty/null keeps the sdkconfig defaults.
     *
     * Only takes effect for a driver installed after this call -- the host
     * reads descriptors at enumeration -- so ROBOT::init() calls it right
     * after settings.load(), before anything installs the device. A rename
     * shows up from the next boot, like the BLE name. esp_tinyusb sends at
     * most 31 characters, one byte per character (ASCII).
     */
    void set_product_name(const char* name) noexcept;

    /**
     * @brief Hand the SD card back to the robot: return the FAT mount point
     * to the app side and tear down the native USB peripheral driver that
     * expose() installed. Mirrors OTAUpdater::cancel() — a firmware-side
     * "give it back" the operator triggers directly, since Windows' safe
     * eject no longer reclaims it on its own (auto_mount_off in begin()
     * disables the automatic mount-point handback tud_umount_cb() used to
     * do, which is also what was silently corrupting the volume mid-read —
     * see the class comment).
     * @return true once the card is confirmed back on the app side.
     */
    bool reclaim();

    /**
     * @brief Process USB eject/disconnect without blocking the state task.
     * @param button_flags Current button flags; BIT_2 reclaims the SD card
     * back to the robot (same physical button used to enter DEBUG in the
     * first place — see BIT_2 -> DEBUG in StatesManager.h), the same
     * button-driven exit gesture OTAUpdater::process() uses to cancel.
     */
    void process(uint8_t button_flags);

    bool is_ready() const { return initialized_.load(); }
    bool is_exposed() const {
        return session_active_.load() && !app_has_access_.load();
    }
    // A persistent device (install_persistent_usb_device()) alone does not
    // count: it is the serial link, not a storage session.
    bool is_active() const {
        return session_active_.load() ||
               (usb_driver_installed_.load() && !usb_driver_persistent_.load());
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
    Flags_out* leds_ = nullptr;
    void* storage_handle_ = nullptr;
    char usb_serial_[13]{};
    // set_product_name()'s copy; empty = sdkconfig's product/CDC strings.
    // 32 = esp_tinyusb's MAX_DESC_BUF_SIZE (31 characters on the wire).
    char usb_product_[32]{};
    // LANGID, manufacturer, product, serial, CDC interface, MSC interface --
    // the order esp_tinyusb's default descriptor (usb_descriptors.c) indexes
    // them in with both CFG_TUD_CDC and CFG_TUD_MSC enabled.
    static constexpr int kUsbStringCount = 6;
    const char* usb_string_descriptors_[kUsbStringCount]{};
    std::atomic<bool> initialized_{false};
    std::atomic<bool> app_has_access_{false};
    std::atomic<bool> usb_driver_installed_{false};
    std::atomic<bool> usb_driver_persistent_{false};
    std::atomic<bool> session_active_{false};
    std::atomic<bool> host_attached_{false};
    std::atomic<bool> mount_transition_failed_{false};

    // LED blink state while exposed; see update_status_led().
    uint32_t blink_next_ms_ = 0;
    bool blink_alt_ = false;

    bool prepare_usb_identity();
    bool install_usb_driver(bool persistent);
    bool sync_mount_state();
    void handle_storage_event(void* event);

    /**
     * @brief Non-blocking LED tick for storage mode, called every process()
     * pass while the card is exposed. Same on-time refresh trick as
     * OTAUpdater::update_status_led(): LED0/LED2 on for one second, then
     * LED1/LED3 on for the next second, repeating.
     */
    void update_status_led();
};

#endif // USB_MASS_STORAGE_H
