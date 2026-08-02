#ifndef SETTINGS_H
#define SETTINGS_H

// logger configuration (build flags)
// -> LOG_ALL
// -> LOG_INFO
// -> LOG_ERROR
// -> LOG_DEBUG

// -------------------- Timers and delays --------------------
#define SAMPLE_MICROS   1000   // 1ms 1000Hz - EKF sample rate
#define SAMPLE_MILLIS   (SAMPLE_MICROS / 1000) // sample rate in ms
#define FREQ_EKF        (1.0f / (SAMPLE_MICROS * 0.000001f))
#define SYSMON_FREQ_MS  10000   // 10000ms 0.1Hz - system monitor report frequency
#define WDOG_TIMEOUT_TK 1       // 1 tick - quantos loops do freRTOS demoramos para excecutar novamente a task
#define DELAY_FLAGS     250     // 250ms - time to reset flags and check state changes

// -------------------- Memory alias configuration --------------------
#define M2KB             (2 * 1024) // 2KB for the stack of the tasks, to avoid stack overflow
#define M4KB             (4 * 1024) // 4KB for the stack of the tasks, to avoid stack overflow
#define M8KB             (8 * 1024) // 8KB for the stack of the tasks, to avoid stack overflow
#define M16KB            (16 * 1024) // 16KB for the stack of the tasks, to avoid stack overflow
#define M32KB            (32 * 1024) // 32KB for the stack of the tasks, to avoid stack overflow

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

// -------------------- logger configuration --------------------
// here we define the data structures and definitions for the log messages, 
// that will be shared between the logger and the transport protocol (e.g., esp-now)

// max espnow packet size -> 250bytes

// protocol overhead: 
//   4 bytes for timestamp 
// + 1 byte for type 
// + 2 byte for packet number 
// + 2 byte for total packets 
// + 1 byte for checksum 
// + 4 byte para o length
// = 14 bytes
    
// therefore, max message size is 250 - 14 = 236 bytes
// but, to use a size that is a multiple of 4 for better memory alignment, we will use 230 bytes for the content
// the N16R8 chip has 8mb of external psram, which is 8 * 1024 * 1024 = 8388608 bytes
// if we use 250 bytes per message (including overhead), we can store up to 8388608 / 250 = 33554
// but to be safe, we use 90% of the available memory for the logger, which gives us a limit of 30.200 
// max packets in psram = 0.9 * 33554 = 30.200 -> round to 30000

#define MAX_PACKET_SIZE         250   // if we change the transport protocol, we can increase this value
#define PROTOCOL_OVERHEAD_SIZE  20    // overhead for the protocol, including timestamp, type, packet number, total packets and checksum
#define MAX_CONTENT_SIZE        229   // -1 to ensure we have space for the null terminator
#define MAX_PACKETS_IN_PSRAM    30000 // limit for messages in memory - watch out for available ram limits
#define LOGGER_MUTEX_TIMEOUT_MS 100   // time in ms to wait for the logger to be available - used to avoid deleting messages during printing

// to empty the array and free the mutex to other tasks, 
// the code flush the array in chuncks, and to free the core to other tasks,
// we limit a max chucks per flush
#define MAX_CHUNKS_PER_FLUSH    10 // 10 chucks per flush 
#define BLOCK_SIZE              16 // the chunck has 16 messages 

// -------------------- Channels --------------------
#define CH0             0
#define CH1             1
#define CH2             2
#define CH3             3

// ============================================================
// Pinout da ESP32-S3
// ============================================================

// -------------------- LEDs dos strapping pins --------------------
// warning: the GPIOs 43 and 44 are used for the serial communication
// and GPIO 45 and 46 are used for the flash memory
#define LED0            GPIO_NUM_45
#define LED1            GPIO_NUM_46
#define LED2            GPIO_NUM_43 // TX
#define LED3            GPIO_NUM_44 // RX

// -------------------- H Bridge / DRV825 --------------------
// Motor A = esquerdo
#define AIN1            GPIO_NUM_21  // IN1_E
#define AIN2            GPIO_NUM_47  // IN2_E

// Motor B = direito
#define BIN1            GPIO_NUM_3   // IN1_D
#define BIN2            GPIO_NUM_9   // IN2_D

// Leitura de corrente dos DRV825
#define CURRENT_A       GPIO_NUM_14  // i_E
#define CURRENT_B       GPIO_NUM_8   // i_D

// -------------------- Encoders --------------------
// Encoder A = motor esquerdo
#define ENC_A0          GPIO_NUM_41  // ENC1_E
#define ENC_A1          GPIO_NUM_42  // ENC2_E

// Encoder B = motor direito
#define ENC_B0          GPIO_NUM_39  // ENC1_D
#define ENC_B1          GPIO_NUM_40  // ENC2_D

// -------------------- Buttons --------------------
#define BTN0            GPIO_NUM_0   // BOOT
#define BTN1            GPIO_NUM_4
#define BTN2            GPIO_NUM_5

// -------------------- Side sensors --------------------
#define LEFT            GPIO_NUM_1   // SE_digital
#define RIGHT           GPIO_NUM_48  // SD_digital

// -------------------- Buzzer --------------------
#define BZR             GPIO_NUM_38

// -------------------- Analog multiplexer --------------------
#define S0              GPIO_NUM_12
#define S1              GPIO_NUM_11
#define S2              GPIO_NUM_10
#define SIG             GPIO_NUM_13

/*
 * D0 até D7 são canais do multiplexador.
 * Eles não são GPIOs da ESP32.
 */
#define D0              0U
#define D1              1U
#define D2              2U
#define D3              3U
#define D4              4U
#define D5              5U
#define D6              6U
#define D7              7U

// -------------------- I2C devices --------------------
#define SDA_pin         GPIO_NUM_18
#define SCL_pin         GPIO_NUM_17

// -------------------- SD Card / SPI --------------------
#define MISO            GPIO_NUM_6
#define SCK             GPIO_NUM_7
#define MOSI            GPIO_NUM_15
#define CS              GPIO_NUM_16

// -------------------- Voltage dividers --------------------
// #define BAT             GPIO_NUM_2

// -------------------- USB nativo --------------------
#define USB_D_MINUS     GPIO_NUM_19
#define USB_D_PLUS      GPIO_NUM_20

// -------------------- GPIOs livres no esquema --------------------
// #define GPIO_SPARE_1    GPIO_NUM_1
// #define GPIO_SPARE_2    GPIO_NUM_38

// -------------------- Flags indices --------------------
#define BTN1_idx            0
#define BTN2_idx            1
#define BTN3_idx            2
#define SENSOR_LEFT_idx     0
#define SENSOR_RIGHT_idx    1
#define LED0_idx            0
#define LED1_idx            1
#define LED2_idx            2
#define LED3_idx            3
#define MOTOR_LEFT_idx      0
#define MOTOR_RIGHT_idx     1

// remap the using the color 
#define LED_BLUE            LED0_idx
#define LED_GREEN           LED1_idx
#define LED_YELLOW          LED2_idx
#define LED_RED             LED3_idx

#endif // SETTINGS_H