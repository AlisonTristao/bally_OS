#include "StatesManager.h"

stateName States::commconfig_function() {
    // All the logic lives on ROBOT (ROBOT::commConfigTick(), BallyRobot.cpp)
    // for the same reason blinkErrorLeds()/processDebug() do: it needs
    // leds/buttons/settings/sd_card, which are ROBOT's, not this class's.
    // See its own comment for the menu behaviour (T25b,
    // TAREFAS_TCP_BLE_ANDROID.txt, ETAPA 3B).
    return robot_.commConfigTick();
}
