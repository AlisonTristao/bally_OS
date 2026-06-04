#ifndef SETTINGS_H
#define SETTINGS_H

// logger configuration (build flags)
// -> LOG_ALL
// -> LOG_INFO
// -> LOG_ERROR
// -> LOG_DEBUG

// timers
#define SAMPLE_MICROS   10000   // 10ms 100Hz - EKF sample rate
#define SAMPLE_MILLIS   (SAMPLE_MICROS / 1000) // sample rate in ms
#define FREQ_EKF        (1.0f / (SAMPLE_MICROS * 0.000001f))
#define SYSMON_FREQ_MS  5000    // 5000ms 0.2Hz - system monitor report frequency
#define WDOG_TIMEOUT_MS 1       // 1ms - watchdog timeout for the main loop, this is important to reset the robot in case of a deadlock or infinite loop
#define DELAY_FLAGS     250     // 250ms - time to reset flags and check state changes
#define CONTROL_TIME_MS 1

// esp32 core
#define PRIMARY_CORE    1       // void loop
#define SECONDARY_CORE  0       // parallel processing

// -------------------- Array sensor configuration --------------------
#define LEN_SENSOR      8       // number of sensors
#define SAMPLES         30      // number of samples for calibration
#define DELAY_SAMPLE    100     // delay between samples (ms)

// -------------------- EKF configuration --------------------
#define EKF_STATE_DIM   3
#define EKF_MEASURE_DIM 5
#define EKF_CONTROL_DIM 2
#define EKF_WHEEL_BASE  0.30f
#define EKF_K_R         1.0f
#define EKF_K_L         1.0f
#define EKF_TAU_R       0.15f
#define EKF_TAU_L       0.15f
#define EKF_IMU_RX      0.1f
#define EKF_IMU_RY      0.1f

// -------------------- Kalman filter noise configuration --------------------
#define ENCODER_PPR      70      // pulses per wheel revolution
#define WHEEL_RADIUS     0.0325f // wheel radius in meters
#define V_NOISE          0.1f    // process noise variance
#define W_NOISE          0.1f    // measurement noise variance
#define B_NOISE          0.0001f // bias noise variance
#define INITIAL_P        0.1f    // initial state covariance
#define ENC_NOISE        0.5f    // encoder noise variance
#define ACCEL_NOISE      0.5f    // IMU accel noise variance
#define GYRO_NOISE       0.001f  // IMU gyro noise variance

// -------------------- Channels --------------------
#define CH0             0
#define CH1             1
//#define CH2             2
//#define CH3             3

// -------------------- Array of LEDs --------------------
/*#define YELLOW          38
#define RED             37
#define BLUE            36
#define GREEN           35
#define UNK0            45
#define UNK1            46*/

// -------------------- H Bridge --------------------
/*#define PWM_A           35
#define AIN2            36
#define AIN1            37
#define BIN1            38
#define BIN2            39
#define PWM_B           40*/

// -------------------- RGB LED (ESP32-S3 SPI) --------------------
#define LED_RGB_PIN     48

// -------------------- Encoders --------------------
#define ENC_A0          21
#define ENC_A1          47
#define ENC_B0          45
#define ENC_B1          46

// -------------------- Buttons --------------------
#define BTN1            1
#define BTN2            2
#define BTN3            0

// -------------------- Side sensors --------------------
#define LEFT            39
#define RIGHT           40

// -------------------- Buzzer --------------------
#define BZR             6

// -------------------- Sensor --------------------
#define D0              18
#define D1              17
#define D2              16
#define D3              15
#define D4              7
#define D5              6
#define D6              5
#define D7              4

// -------------------- I2C devices --------------------
#define SDA_pin         4
#define SCL_pin         5

// -------------------- Voltage dividers --------------------
#define BAT             7

// -------------------- Flags indices --------------------
#define BTN1_idx            0
#define BTN2_idx            1
#define BTN3_idx            2
#define SENSOR_LEFT_idx     0
#define SENSOR_RIGHT_idx    1
#define LED1_idx            0
#define LED2_idx            1
#define MOTOR_LEFT_idx      0
#define MOTOR_RIGHT_idx     1

#endif // SETTINGS_H
