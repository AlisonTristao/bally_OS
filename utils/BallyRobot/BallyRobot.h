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

class ROBOT {
public:
    // default constructor
    // default constructor
    ROBOT() :   motor_left(AIN1, AIN2, CH0, PWM_A), 
                motor_right(BIN1, BIN2, CH1, PWM_B),
                encoder_left(ENC_A0, ENC_A1), 
                encoder_right(ENC_B0, ENC_B1),
                //imu(Wire, 0x68), 
                EKF() {
        // save the instance of the robot class to be used in the static functions
        instance_ = this;
    }

    // default destructor
    virtual ~ROBOT() {};

    // initialize the robot, configure the pins, the wifi and the esp-now settings
    bool init();
    static void* getInstance() {
        return static_cast<void*>(instance_);
    }

    // routine to be executed in parallel processing
    static void routine(void *param);

    // configure the interruptions for the buttons and side sensors
    static void configure_interruptions(void *param);

    // run the EKF to estimate the state of the robot
    void runEKF();

    // Signals and flags for buttons, sensors, LEDs, and motors
    static Flags_in buttons;
    static Flags_in sideSensors;
    static Flags_out leds;
    static Flags_pwm motors;

    // core utility objects
    static Logger logger;
    static StateMachine machine;
    static TinyShell shell;

    // peripheral objects
    static ArraySensor<LEN_SENSOR> array_sensor;
private:
    // private peripheral objects
    HBridge motor_left;
    HBridge motor_right;
    Encoder encoder_left;
    Encoder encoder_right;
    //ICM42688 imu;
    TinyEKF EKF;

    // save a instance of the ROBOT class to be used in the static functions
    static ROBOT* instance_;
    bool initialized = false;

    void initEKF();

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
