#ifndef PINOUT_H
#define PINOUT_H

// native libraries
#include <Arduino.h>

// logger configuration
// -> LOG_ALL             // register all logs
// -> LOG_INFO            // register configuration logs
// -> LOG_TELEMETRY       // register operational logs
// -> LOG_ERROR           // register error logs
// -> LOG_DEBUG           // register debug logs

// sampling activation
// -> SAMPLING_ACTIVE
//#define SAMPLING_ACTIVE

// timers
#define SAMPLE_MICROS   10000   // 10ms 100Hz - Taxa de amostragem para o EKF
#define SAMPLE_MILLIS   SAMPLE_MICROS / 1000 // taxa de amostragem em ms
#define FREQ_EKF        1/(SAMPLE_MICROS * 0.000001)
#define DELAY_FLAGS     250     // 250ms - Tempo para resetar os flags e verificar a mudança de estado da máquina de estados
#define CONTROL_TIME_MS 1

// esp32 core 
#define PRIMARY_CORE    1       // void loop
#define SECONDARY_CORE  0       // parallel processing 

// -------------------- Array sensor configuration --------------------
#define LEN_SENSOR      8       // quantidade de sensores
#define SAMPLES         30      // número de amostras para calibração
#define DELAY_SAMPLE    100     // delay entre amostras (ms)

// -------------------- Kalman filter configuration --------------------
#define ENCODER_PPR      70      // quantidade de pulsos por revolução do encoder
#define WHEEL_RADIUS     0.0325f // raio da roda em metros
#define V_NOISE          0.1f    // variância do ruído do processo
#define W_NOISE          0.1f    // variância do ruído da med
#define B_NOISE          0.0001f // variância do ruído do bias
#define INITIAL_P        0.1f    // valor inicial da covariância do estado
#define ENC_NOISE        0.5f    // variância do ruído dos encoders
#define ACCEL_NOISE      0.5f    // variância do ruído do IMU
#define GYRO_NOISE       0.001   // variância do ruído do giroscópio

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
#define PWM_A           35
#define AIN2            36
#define AIN1            37
#define BIN1            38
#define BIN2            39
#define PWM_B           40

// -------------------- RGB LED (ESP32-S3 SPI) --------------------
#define LED_RGB_PIN 48

// -------------------- Encoders --------------------
#define ENC_A0          21
#define ENC_A1          47
#define ENC_B0          20
#define ENC_B1          19

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
#define SDA             4
#define SCL             5

// -------------------- Voltage dividers --------------------
#define BAT             7

#endif // PINOUT_H