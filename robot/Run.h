#ifndef RUN_H
#define RUN_H

// header
#include <Pinout.h>

// custom libraries
#include <ArraySensor.h>
#include <HBridge.h>
#include <Encoder.h>
#include <Control.h>

// pwm maximo 
int16_t PWM = 55;

uint32_t timer_ms = 0;

stateName run_to_finish() {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        ROBOT::logger.insert_log("state_changed: run -> finish", logType::INFO);
    #endif

    // return the stateName of the next state
    return FINISH;
}

stateName run_function() {
    timer_ms = millis();

    // calcula a posicao da linha
    double position = (ROBOT::array_sensor.read_line() - 4500)/100;

    // calcula o controle das rodas
    double pd_w = ROBOT::control.simplePD(4.0, 0.20, position, CONTROL_TIME_MS/1000.0);

    // velocidade total das rodas
    double velLeft  = PWM;  // mudar para pwmLeft
    double velRight = PWM; // mudar para pwmRight

    // aplica o double pd_w nas rodas
    velRight = PWM - pd_w;
    velLeft  = PWM + pd_w;

    // sat
    if(velLeft > 2*PWM)        velLeft = 2*PWM;
    if(velLeft < -2*PWM)       velLeft = -2*PWM;    
    
    ROBOT::motor_left.applyPWM((int32_t)velLeft);
    ROBOT::motor_right.applyPWM((int32_t)velRight);

    while (millis() - timer_ms < CONTROL_TIME_MS);

    return RUN;
}

stateName next_state_run(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0))
        return run_to_finish();

    // stay in the same state
    return RUN;
}

#endif