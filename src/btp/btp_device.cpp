// Name, link, radio, tasks and memory -- from settings.conf, read at boot.
#include "btp/btp_robot.h"

#include <cstdio>
#include <cstring>

#include "esp_chip_info.h"
#include "esp_ota_ops.h"

#include <BallyRobot.h>

namespace {

// DeviceConfig keeps pointers: everything it points at lives here.
char g_name[btpdev::DeviceConfig::kMaxNameLength + 1U] = {};
char g_chip_revision[8] = {};
char g_ota_endpoint[48] = {};

}  // namespace

void robot_device_config(const ROBOT& robot, btpdev::DeviceConfig& out) {
    const SettingsData& cfg = robot.settings.data();

    // The name TraceView lists the robot under -- USB, BLE and mDNS
    // (<name>.local, the TCP and OTA host) alike. BtpDevice takes at most
    // 29 characters (a BLE advertisement); settings.conf allows 31.
    std::snprintf(g_name, sizeof(g_name), "%s", cfg.name[0] != '\0' ? cfg.name : "BallyRobot");
    out.name = g_name;

    out.link = robot.bootLink();
    out.fallback = btpdev::Link::Serial;
    // 5 topics / 29 fields, the ~1.7 KB manifest and a 1 KB system.monitor
    // text: Medium holds them (8 topics, 48 fields).
    out.memory = btpdev::Memory::Medium;

    // The old "comms" task's place: core 0 with EKF, priority 4 (the shell
    // task runs one below). Core 1 stays the state machine's alone
    // (CONTRIBUTING.md).
    out.comms.core = 0;
    out.comms.priority = 4U;
    out.comms.stack_bytes = 8192U;

    // ESP-NOW: the dongle (MAC_ADDR build flag) on the configured channel.
#ifdef MAC_ADDR
    static constexpr uint8_t kDongleMac[6] = {MAC_ADDR};
    std::memcpy(out.espnow.peer_mac, kDongleMac, sizeof(kDongleMac));
#endif
    out.espnow.channel = cfg.espnow_channel;

    // TCP: networks from wifi.txt on the SD card, the same file "ota
    // wifi_add" writes.
    out.wifi.file = OTA_WIFI_LIST_FILE;

    // USB serial: ROBOT::init() installs the composite CDC + MSC device,
    // and the cable needs no password -- whoever holds it already has the
    // robot in hand. ESP-NOW, TCP and BLE stay encrypted.
    out.usb.install_driver = false;
    out.serial_cleartext = true;

    // MANIFEST source_info, next to what BtpDevice adds itself (name,
    // firmware, build, ESP-IDF, chip).
    esp_chip_info_t chip{};
    esp_chip_info(&chip);
    std::snprintf(g_chip_revision, sizeof(g_chip_revision), "%u.%u", chip.revision / 100U,
                  chip.revision % 100U);
    const char* host = cfg.ota_hostname;
    if (host[0] != '\0') {
        std::snprintf(g_ota_endpoint, sizeof(g_ota_endpoint),
                      std::strstr(host, ".local") != nullptr ? "%s" : "%s.local", host);
    }
    const esp_partition_t* running = esp_ota_get_running_partition();
    std::size_t i = 0U;
    const auto add = [&](const char* key, const char* label, const char* value) {
        if (value == nullptr || value[0] == '\0' || i >= btpdev::DeviceConfig::kMaxSourceInfo) return;
        out.source_info[i++] = btpdev::SourceInfo{key, label, value};
    };
    add("description", "Description", cfg.description);
    add("model", "Model", "BallyRobot");
    add("chip_revision", "Chip revision", g_chip_revision);
    add("ota_partition", "Running partition", running != nullptr ? running->label : "");
    // On a TCP boot the OTA server answers at BtpDevice's <name>.local
    // instead (the Wi-Fi and its mDNS are BtpDevice's then).
    if (robot.bootLink() != btpdev::Link::Tcp) add("ota_endpoint", "OTA endpoint", g_ota_endpoint);
}
