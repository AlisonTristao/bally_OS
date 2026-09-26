#ifndef BTP_ROBOT_H
#define BTP_ROBOT_H

// The robot's BtpDevice configuration -- the link, the key, the terminal and
// the telemetry all come from BTPDevice (sibling checkout, see
// CMakeLists.txt). One file per concern, the same split as BTPDevice's
// examples:
//
//   btp_device.cpp    name, link, radio, tasks, memory   (from settings.conf)
//   btp_storage.cpp   where the key file and wifi.txt are (the SD card)
//   btp_commands.cpp  the shell modules
//   btp_topics.cpp    the telemetry topics
//
// main.cpp builds the device from these after ROBOT::init().

#include <BtpDevice.h>

class ROBOT;

void robot_device_config(const ROBOT& robot, btpdev::DeviceConfig& out);
void robot_storage_config(const ROBOT& robot, btpdev::StorageConfig& out);
void robot_commands(btpdev::Commands& commands);
void robot_topics(btpdev::Topics& topics);

#endif  // BTP_ROBOT_H
