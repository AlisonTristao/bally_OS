#ifndef ROBOT_STATIC_OBJECTS_H
#define ROBOT_STATIC_OBJECTS_H

#include <Arduino.h>

#include <Pinout.h>
#include <RGBLed.h>

#include <ArraySensor.h>
#include <Encoder.h>
#include <HBridge.h>
#include <Control.h>

#include <TinyShell.h>

#include <Flags.h>
#include <Logger.h>
#include <StateMachine.h>

class ROBOT {
public:
    static void init();
    static void applyStateOutputs(uint8_t state);
    static void stopMotors();
    static void brakeMotors();
    static void logStateMachineError(const char* message);

    static Signals_IN flags;
    static Logger logger;
    static StateMachine machine;
    static RGBLed rgb_led;
    static ArraySensor array_sensor;
    static HBridge motor_left;
    static HBridge motor_right;
    static Control control;
    static TinyShell shell;
};

#endif
