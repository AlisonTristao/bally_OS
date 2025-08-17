#ifndef PINOUT_H
#define PINOUT_H

// -------------------- Channels --------------------
#define CH0             0
#define CH1             1
#define CH2             2
#define CH3             3

// -------------------- Array of LEDs --------------------
#define YELLOW          38
#define RED             37
#define BLUE            36
#define GREEN           35
#define UNK0            45
#define UNK1            46

// -------------------- H Bridge --------------------
#define AIN1            13
#define AIN2            12
#define BIN1            11
#define BIN2            10
#define PWM_A           9
#define PWM_B           14

// -------------------- Encoders --------------------
#define ENC_A0          21
#define ENC_A1          47
#define ENC_B0          20
#define ENC_B1          19

// -------------------- Buttons --------------------
#define BTN1            1
#define BTN2            2
#define BTN3            0

// -------------------- Buzzer --------------------
#define BZR             6

// -------------------- Multiplex --------------------
#define SIG             18
#define C0              39
#define C1              40
#define C2              41
#define C3              42

// -------------------- Side sensors --------------------
#define LEFT            8 
#define RIGHT           17

// -------------------- I2C devices --------------------
#define SDA             4
#define SCL             5

// -------------------- Voltage dividers --------------------
#define BAT             7

// -------------------- Array sensor configuration --------------------
#define LEN_SENSOR      8       // quantidade de sensores
#define INIT_MUX        4       // estado inicial do multiplexador
#define SAMPLES         30      // número de amostras para calibração
#define DELAY_SAMPLE    100     // delay entre amostras (ms)

#endif // PINOUT_H