#ifndef ROBOT_STATIC_OBJECTS_H
#define ROBOT_STATIC_OBJECTS_H

#include <Arduino.h>

#include <esp_timer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

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
    static void logStateMachineError(const char* message);

    // routine to be executed in parallel processing
    static void routine(void *param);

    // Interrupt/timer handling (moved from ParallelProcessing.h)
    static esp_timer_handle_t timer_get_handle;
    static void sampleISR(void* arg);
    static void configure_interruptions(void *param);

    // Callbacks for ESP-NOW (moved from EspNow.h)
    static void handleReceiveStatic(const uint8_t* mac, const uint8_t* incomingData, int len);

    // Callbacks for ESP-NOW (moved from EspNow.h)
    static void handleSendStatic(const uint8_t* mac, esp_now_send_status_t status);

    // Signals and flags for buttons, sensors, LEDs, and motors
    static Flags_in buttons;
    static Flags_in sideSensors;
    static Flags_out leds;
    static Flags_pwm motors;

    // core utility objects
    static Logger logger;
    static StateMachine machine;
    static TinyShell shell;
    static Control control;

    // peripheral objects
    static RGBLed rgb_led;
    static ArraySensor array_sensor;
    static HBridge motor_left;
    static HBridge motor_right;
};

// Define the buttons and side sensors as Flags_in objects with their respective indices
static FlagsIsrArg btnArgs[] = {
    {&ROBOT::buttons, 0},
    {&ROBOT::buttons, 1},
    {&ROBOT::buttons, 2}
};

static FlagsIsrArg sideArgs[] = {
    {&ROBOT::sideSensors, 0},
    {&ROBOT::sideSensors, 1}
};

#endif
