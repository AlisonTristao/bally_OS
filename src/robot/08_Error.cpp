#include "StatesManager.h"

stateName States::error_function() {
    // Fail-safe: force the motors off immediately (the flags' own timeout
    // would eventually do it too, but this cannot wait) and blink every LED
    // together so the failure is visible even without telemetry.
    robot_.stopMotors();
    robot_.blinkErrorLeds();
    return ERROR;
}
