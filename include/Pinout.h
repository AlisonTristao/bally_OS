#ifndef PINOUT_H
#define PINOUT_H

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
#define LED_RGB_MOSI    11      // SPI MOSI
#define LED_RGB_MISO    13      // SPI MISO (optional)
#define LED_RGB_SCK     12      // SPI SCK
#define LED_RGB_CS      10      // SPI Chip Select

// -------------------- Encoders --------------------
/*#define ENC_A0          21
#define ENC_A1          47
#define ENC_B0          20
#define ENC_B1          19

// -------------------- Buttons --------------------
#define BTN1            1
#define BTN2            2
#define BTN3            0

// -------------------- Buzzer --------------------
#define BZR             6*/

// -------------------- Sensor --------------------
#define D0        18
#define D1        17
#define D2        16
#define D3        15
#define D4        7
#define D5        6
#define D6        5
#define D7        4

// -------------------- Side sensors --------------------
/*#define LEFT            8 
#define RIGHT           17

// -------------------- I2C devices --------------------
#define SDA             4
#define SCL             5

// -------------------- Voltage dividers --------------------
#define BAT             7*/

// -------------------- Array sensor configuration --------------------
#define LEN_SENSOR      8       // quantidade de sensores
#define SAMPLES         30      // número de amostras para calibração
#define DELAY_SAMPLE    100     // delay entre amostras (ms)

#endif // PINOUT_H