#ifndef ROBOT_STATIC_OBJECTS_H
#define ROBOT_STATIC_OBJECTS_H

// ==================== SYSTEM ====================
#include <Arduino.h>

// ==================== PROJECT HEADERS ====================
#include <Pinout.h> 
#include <RGBLed.h>

// ==================== CUSTOM MODULES ====================
#include <ArraySensor.h>
#include <Encoder.h>
#include <HBridge.h>
#include <Control.h>

// ==================== EXTERNAL LIBRARIES ====================
#include <TinyShell.h> 

// ==================== UTILITIES ====================
#include <Flags.h>
#include <Logger.h>
#include <StateMachine.h>

using namespace std;

// ==================== ROBOT CORE CLASS ====================
class ROBOT {
public:
    static void init();

    static void applyStateOutputs(uint8_t state);

    static void stopMotors();
    static void brakeMotors();
    static void applyMotorPWM(int32_t pwmLeft, int32_t pwmRight);

    static double readLine();
    static bool calibrateSensors(uint8_t samples, uint8_t delayMs);
    static void saveCalibration();
    static String calibrateStatus();
    static String debugSensors();

    static double controlSimplePD(double kp, double kd, double error, double sample);

    static void log(const String& msg, logType type);
    static void logStateMachineError(const char* message);
    static void sendLogger(logType type);
    static void sendLoggerLive();

    static void setFlagFilterTime(uint32_t timeMs);
    static void checkFlagsDuration();
    static uint8_t getButtons();
    static uint8_t getSideSensors();

    static TinyShell& shell();
    static String runLineCommand(const String& cmd);

    // public static members
    static RGBLed rgb_led_;
private:
    static ArraySensor array_sensor_;
    static HBridge motor_left_;
    static HBridge motor_right_;
    static Control control_;
    static TinyShell shell_;
};

#endif