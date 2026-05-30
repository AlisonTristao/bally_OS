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
    static ArraySensor<LEN_SENSOR> array_sensor;
private:
    // private peripheral objects
    static HBridge motor_left;
    static HBridge motor_right;

    // save a instance of the ROBOT class to be used in the static functions
    static ROBOT* instance_;
    bool initialized = false;

    // configure the pins, the i2c communication and other settings for the robot
    bool configurePins();

    // configure the wifi and the esp-now settings for the robot
    bool configureCommunication();

    // set the time limit for the flags, to reset them after a certain time
    void setTimeLimit();

    // run received commands of the queue
    void executeCommandFromQueue();

    // exec a command in the shell and return the result as string
    void executeCommand(const char* command) const;

    // set the outputs flags (leds and motors) to 0 after the time limit is reached
    void setOutputs();

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

// alias to rename de BIT_s with the buttons, side sensors, leds and motors flags
#define BTN1_idx            0
#define BTN2_idx            1
#define BTN3_idx            2
#define SENSOR_LEFT_idx     0
#define SENSOR_RIGHT_idx    1
#define LED1_idx            0
#define LED2_idx            1
#define MOTOR_LEFT_idx      0
#define MOTOR_RIGHT_idx     1

// Define the buttons and side sensors as Flags_in objects with their respective indices
static FlagsArg btnArgs[] = {
    {&ROBOT::buttons, BTN1_idx},
    {&ROBOT::buttons, BTN2_idx},
    {&ROBOT::buttons, BTN3_idx}
};

static FlagsArg sideArgs[] = {
    {&ROBOT::sideSensors, SENSOR_LEFT_idx},
    {&ROBOT::sideSensors, SENSOR_RIGHT_idx}
};

#endif
