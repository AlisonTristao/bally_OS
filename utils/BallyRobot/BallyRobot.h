#ifndef ROBOT_STATIC_OBJECTS_H
#define ROBOT_STATIC_OBJECTS_H

#include <atomic>
#include <cstdint>
#include <optional>
#include <esp_timer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdio.h>
#include "esp_attr.h"

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
#include <OTAUpdater.h>
#include <RobotSettings.h>
#include <SDCard.h>
#include <StateMachine.h>
#include <USBMassStorage.h>

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
    virtual ~ROBOT() {
        // delete the queue
        if (receivedDataQueue == nullptr)
            return;
        vQueueDelete(receivedDataQueue);
        receivedDataQueue = nullptr;
    };
    ROBOT(const ROBOT&) = delete;
    ROBOT& operator=(const ROBOT&) = delete;

    // initialize the robot, configure the pins, the wifi and the esp-now settings
    bool init();

    // routine to be executed in parallel processing
    static void routine(void *param);

    // configure the interruptions for the buttons and side sensors
    static void initInterruptions(void *param);

    // excute a command in the shell and return the result as string
    static void runShell(void *param);

    // run the state machine main loop
    static void runStateMachine(void *param);

    // sample the sensor and wake up de run ekf task
    static IRAM_ATTR void sampleEKF(void *param);

    // run the ekf in parallel processing
    static void runEKF(void *param);

    // core utility objects
    static Logger logger;
    StateMachine machine;
    TinyShell shell;
    RobotSettings settings;
    // Constructed in init(), once pins are known from settings.load() —
    // ArraySensor's constructor configures GPIO/ADC immediately, so it
    // cannot run at static-init time like the other members below.
    std::optional<ArraySensor> array_sensor;
    SDCard sd_card;
    USBMassStorage usb_storage;
    OTAUpdater ota;

    // Execute scheduled DEBUG tests without blocking the state-machine task.
    void processDebug();
    void cancelDebugTests();

    // Keep selected shell responses out of the retained PSRAM log.
    void sendNextShellOutputDirect();
    bool consumeDirectShellOutputRequest();
private:
    // default constructor
    ROBOT() :   machine(NONE, NULL, NULL),
                sd_card(MISO, SCK, MOSI, CS),
                //motor_left(AIN1, AIN2, CH0, PWM_A),
                //motor_right(BIN1, BIN2, CH1, PWM_B),
                EKF(),
                buttons("Buttons"),
                sideSensors("Side Sensors"),
                leds("LEDs"),
                motors("Motors")
                //imu(Wire, 0x68), 
                {
        // save the instance of the robot class to be used in the static functions
        instance_ = this;
    }

    // private peripheral objects
    //HBridge motor_left;
    //HBridge motor_right;
    // Constructed in init(), see array_sensor above.
    std::optional<Encoder> encoder_left;
    std::optional<Encoder> encoder_right;
    //ICM42688 imu;
    TinyEKF EKF;
    TaskHandle_t ekf_task_handle = nullptr; 

    // Signals and flags for buttons, sensors, LEDs, and motors
    Flags_in buttons;
    Flags_in sideSensors;
    Flags_out leds;
    Flags_pwm motors;

    // save a instance of the ROBOT class to be used in the static functions
    static ROBOT* instance_;
    bool initialized = false;
    bool clock_synchronized = false;
    char last_log_file[SDFileInfo::MAX_NAME_LENGTH] = {};

    // Non-blocking array sensor test controlled by the DEBUG shell module.
    std::atomic<uint32_t> array_sensor_test_remaining{0};
    std::atomic<uint32_t> array_sensor_test_interval_ms{0};
    std::atomic<uint32_t> array_sensor_test_next_ms{0};
    std::atomic<bool> direct_next_shell_output{false};

    // matriz of data to kalman filter
    float control_input[EKF_CONTROL_DIM] = {0, 0}; // left and right motor pwm
    float measurement[EKF_MEASURE_DIM] = {0, 0, 0, 0, 0};

    void initEKF();

    // Date/time and SD log management used by the storage shell wrappers.
    bool updateDateTime(uint16_t year, uint8_t month, uint8_t day,
                        uint8_t hour, uint8_t minute, uint8_t second);
    bool makeLogFilename(char* filename, size_t capacity);
    bool findLatestLogFile(char* filename, size_t capacity);
    bool flushLoggerToSD(bool append);
    bool startArraySensorTest(uint32_t samples, uint32_t interval_ms);

    // configure the one pin needed before the SD card can be mounted (CS,
    // fixed at compile time — see include/Settings.h)
    bool configurePinsEarly();

    // configure the remaining pins, sourced from settings.data(), once
    // settings.load() has run
    bool configurePinsFromSettings();

    // configure the wifi and the esp-now settings for the robot
    bool configureCommunication();

    // start wrappers
    void startWrappers();

    // set the time limit for the flags, to reset them after a certain time
    void setTimeLimit();

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
