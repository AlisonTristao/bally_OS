#ifndef ROBOT_STATIC_OBJECTS_H
#define ROBOT_STATIC_OBJECTS_H

#include <cstdint>
#include <esp_timer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <Settings.h>

#include <ArraySensor.h>
#include <Encoder.h>
#include <HBridge.h>

#include <TinyShell.h>
#include <TinyEKF.h>
//#include "driver/i2c.h"
//#include <ICM42688.h>
//#include <Wire.h> // pendencia - parar de usar isso aqui - Arduino 

#include <Flags.h>
#include <Logger.h>
#include <StateMachine.h>

// array of the sensor pins, used to initialize the array sensor
const uint8_t sensor_pins[LEN_SENSOR] = {D0, D1, D2, D3, D4, D5, D6, D7};

class ROBOT {
public:
    // singleton pattern
    // getInstance returns a reference to the single instance of the ROBOT class
    // we use this to acess the robot inside all files without needing to pass the instance as a parameter
    static ROBOT& getInstance() {
        static ROBOT instance;
        return instance;
    }
    void* getInstancePtr() {
        return static_cast<void*>(instance_);
    }

    // destructor, copy etc
    virtual ~ROBOT() {};
    ROBOT(const ROBOT&) = delete;
    ROBOT& operator=(const ROBOT&) = delete;

    // initialize the robot, configure the pins, the wifi and the esp-now settings
    bool init();

    // routine to be executed in parallel processing
    static void routine(void *param);

    // configure the interruptions for the buttons and side sensors
    static void configure_interruptions(void *param);

    // run the EKF to estimate the state of the robot
    void runEKF();

    // core utility objects
    static Logger logger;
    StateMachine machine;
    TinyShell shell;
    ArraySensor<LEN_SENSOR> array_sensor;
private:
    // default constructor
    ROBOT() :   machine(NONE, NULL, NULL),
                array_sensor(sensor_pins),
                motor_left(AIN1, AIN2, CH0, PWM_A), 
                motor_right(BIN1, BIN2, CH1, PWM_B),
                encoder_left(ENC_A0, ENC_A1), 
                encoder_right(ENC_B0, ENC_B1),
                EKF(),
                buttons("Buttons"),
                sideSensors("Side Sensors"),
                leds("LEDs"),
                motors("Motors"),
                btnArgs{{&buttons, BTN1_idx},
                        {&buttons, BTN2_idx},
                        {&buttons, BTN3_idx}},
                sideArgs{{&sideSensors, SENSOR_LEFT_idx},
                         {&sideSensors, SENSOR_RIGHT_idx}}
                //imu(Wire, 0x68), 
                {
        // save the instance of the robot class to be used in the static functions
        instance_ = this;
    }

    // private peripheral objects
    HBridge motor_left;
    HBridge motor_right;
    Encoder encoder_left;
    Encoder encoder_right;
    //ICM42688 imu;
    TinyEKF EKF;

    // Signals and flags for buttons, sensors, LEDs, and motors
    Flags_in buttons;
    Flags_in sideSensors;
    Flags_out leds;
    Flags_pwm motors;
    FlagsArg btnArgs[3];
    FlagsArg sideArgs[2];

    // save a instance of the ROBOT class to be used in the static functions
    static ROBOT* instance_;
    bool initialized = false;

    void initEKF();

    // configure the pins, the i2c communication and other settings for the robot
    bool configurePins();

    // configure the wifi and the esp-now settings for the robot
    bool configureCommunication();

    // start wrappers
    void startWrappers();

    // set the time limit for the flags, to reset them after a certain time
    void setTimeLimit();

    // run received commands of the queue
    void executeCommandFromQueue();

    // exec a command in the shell and return the result as string
    void executeCommand(const char* command) const;

    // set the outputs flags (leds and motors) to 0 after the time limit is reached
    void setOutputs();

    // get the speed of the robot based on the encoders values
    float getSpeedFromEncoders();
    float getOmegaFromEncoders();

    // Callbacks for ESP-NOW 
    static void handleReceiveStatic(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);

    // Callbacks for ESP-NOW
    static void handleSendStatic(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) ;

    // reset the flags: buttons, side sensors, leds and motors
    void resetFlags();

    // verify the condictions to change the state machine to the next state
    uint32_t stateMachineTimer = 0;
    void checkStateMachine();

    // queue for the logs to be sent in the parallel processing
    QueueHandle_t receivedDataQueue;
};

#endif