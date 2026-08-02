#include "StatesManager.h"
    
stateName States::debug_function() {
    // Tests are scheduled by DEBUG wrappers and processed one step per pass.
    // The state-machine task already yields after machine.run().
    robot_.processDebug();
    return DEBUG;
}
