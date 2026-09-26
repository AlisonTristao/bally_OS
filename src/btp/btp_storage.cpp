// Where BtpDevice finds the key file and wifi.txt: the robot's SD card.
#include "btp/btp_robot.h"

#include <BallyRobot.h>

void robot_storage_config(const ROBOT& robot, btpdev::StorageConfig& out) {
    // No LittleFS partition on this robot (partitions.csv: two 8 MB OTA
    // slots): everything lives on the SD card.
    out.fs.partition = "";
    // The robot mounts the card itself (SDCard) and lends it to the USB host
    // in DEBUG (USBMassStorage), so BtpDevice only reads it: at begin(),
    // bally.key is imported into NVS and renamed bally.key.imported; the
    // key stays in NVS from then on, card or no card.
    out.app_volume = robot.sd_card.mount_point();
    out.key = btpdev::KeySource::Nvs;
    out.key_file = "bally.key";
}
