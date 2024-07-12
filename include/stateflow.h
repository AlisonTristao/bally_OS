#ifndef STATEFLOW_H
#define STATEFLOW_H

#include <Arduino.h>
#include <arraySensor.h>

enum states
{
    POWER_ON,               // estado 1 -> inicial / Fim corrida
    CALIBRATION,            // estado 2 -> Calibrando sensores
    RUNNING,                 // estado 3 -> Corrida
    DEBUG
};

extern bool right_flag;     // -> Definada no interrupt.h
extern bool left_flag;      // -> Definada no interrupt.h
extern int tempo;           // -> Definada no interrupt.h
extern bool timer_flag;

extern states state;        // -> Definida na MAIN

int counter = 0;

void state_machine()
{
    if (state == DEBUG)
    {
        state = POWER_ON;   // Manter o estado em STATE4 se já estiver no final
    }
    else
    {
        state = static_cast<states>(state + 1); // Incrementar o estado
    }

    timer_flag = (state == RUNNING);
}

void power_func() {
    // pisca os leds em sequencia 
    digitalWrite(LED0, LOW);
    digitalWrite(LED1, HIGH);
    delay(DLY_LONG);
    digitalWrite(LED0, HIGH);
    digitalWrite(LED1, LOW);
    delay(DLY_LONG);
}

bool calibrate(arraySensor* sensor) {
    delay(DLY_LONG);
    sensor->calibrate(50, DLY_SHORT/10, LED0);  // calibrate the sensor
    Serial.println(sensor->calibrate_status()); // print the calibration
    bool calibrate = sensor->calibration_ok();  // return the calibration status
    if(!calibrate){                             // beeeep
        state = POWER_ON;
        ledcWriteTone(CH2, TONE_LOW);
        delay(DLY_LONG);
        ledcWriteTone(CH2, 0);
    } else {                                    // bep bep beeeep
        ledcWriteTone(CH2, TONE_HIGH);
        delay(DLY_SHORT);
        ledcWriteTone(CH2, 0);
        delay(DLY_SHORT);
        ledcWriteTone(CH2, TONE_HIGH);
        delay(DLY_SHORT);
        ledcWriteTone(CH2, 0);
        delay(DLY_SHORT);
        ledcWriteTone(CH2, TONE_HIGH);
        delay(DLY_LONG);
        ledcWriteTone(CH2, 0);
    }
    return calibrate;
}

void running_func()
{
    if (millis() >= tempo + 500 && (right_flag || left_flag))
    {
        if (left_flag && !right_flag && counter == 0)
        {
            counter++;
        }
        else if (left_flag && !right_flag && counter == 1)
        {
            right_flag = false;
            left_flag = false;
            counter = 0;
            // sinalization
            digitalWrite(LED2, right_flag);
            digitalWrite(LED5, left_flag);
            state_machine();
        }

        right_flag = false;
        left_flag = false;
    }

    // sinalization
    digitalWrite(LED2, right_flag);
    digitalWrite(LED5, left_flag);
}

#endif