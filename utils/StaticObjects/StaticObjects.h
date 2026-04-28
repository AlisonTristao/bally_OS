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
    // default constructor
    ROBOT() {
        // save the instance of the robot class to be used in the static functions
        instance_ = this;
    }

    // default destructor
    virtual ~ROBOT() {};

    // initialize the robot, configure the pins, the wifi and the esp-now settings
    bool init();

    // log state machine errors in the logger
    static void staticInsertLog(const char* message);

    // routine to be executed in parallel processing
    static void routine(void *param);

    // configure the interruptions for the buttons and side sensors
    static void configure_interruptions(void *param);

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

private:
    // save a instance of the ROBOT class to be used in the static functions
    static ROBOT* instance_;
    bool initialized = false;

    // run received commands of the queue
    void executeCommandFromQueue();

    // exec a command in the shell and return the result as string
    void executeCommand(const char* command) const;

    // Interrupt/timer handling
    esp_timer_handle_t timer_get_handle;
    static void sampleISR(void* arg);

    // Callbacks for ESP-NOW (moved from EspNow.h)
    static void handleReceiveStatic(const uint8_t* mac, const uint8_t* incomingData, int len);

    // Callbacks for ESP-NOW (moved from EspNow.h)
    static void handleSendStatic(const uint8_t* mac, esp_now_send_status_t status);

    // reset the flags: buttons, side sensors, leds and motors
    void resetFlags();

    // verify the condictions to change the state machine to the next state
    uint32_t stateMachineTimer = 0;
    void checkStateMachine();

    // queue for the logs to be sent in the parallel processing
    QueueHandle_t receveivedDataQueue;
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
