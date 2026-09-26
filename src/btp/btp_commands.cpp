// The robot's shell modules, on BtpDevice's shell: the terminal, remote
// commands and the robot's own jobs and scripts all run them.
#include "btp/btp_robot.h"

#include <BallyRobot.h>
#include <StatesManager.h>

void robot_commands(btpdev::Commands& commands) {
    ROBOT& robot = ROBOT::getInstance();
    // The subsystems' modules (motion, sensor, storage, ota, settings, ...),
    // registered straight on the TinyShell as they always were.
    robot.registerShell(commands.shell());
    // "state": its transition policy lives in src/robot/StatesManager.cpp.
    States::getInstance().register_shell_commands(commands.shell(), robot.logger);
}
