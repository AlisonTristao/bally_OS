#ifndef RUN_H
#define RUN_H

// header
#include <Settings.h>

// custom libraries
#include <ArraySensor.h>
#include <HBridge.h>
#include <Encoder.h>
#include <Control.h>

// pwm maximo 
int16_t PWM = 87;

// global variables
double  position    = 0.0,
        velLeft     = 0.0,
        velRight    = 0.0,
        pd_w        = 0.0;    
        
uint32_t timer_tempo = 0;


// verify if the object was createdpd_w
bool obj_created_run = false;

// array sensor
ArraySensor sensor(LEN_SENSOR, SIG, C0, C1, C2, C3);

// hbridge motor
HBridge motor_l(AIN1, AIN2, CH0, PWM_A);
HBridge motor_r(BIN1, BIN2, CH1, PWM_B);

// encoder
Encoder enc_l(ENC_A0, ENC_A1);
Encoder enc_r(ENC_B0, ENC_B1);

// controle
Control pd;

bool create_objects_run() {
    // create the sensor object
    sensor.init(INIT_MUX);
    obj_created_run = sensor.loadCalibration();

    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        if(obj_created_run){
            Logger::insert_log("calibration loaded", logType::INFO);
            Logger::insert_log("Values:\n" + sensor.calibrate_status(), logType::INFO);
        }else{
            return obj_created_run;
            Logger::insert_log("calibration not loaded", logType::ERROR);
        }
    #endif

    // create the motor objects
    motor_l.init();
    motor_r.init();

    // ISR to second core
	Encoder::core_to_run_ISR = SECONDARY_CORE;

    // create the encoder objects
    enc_l.init();
    enc_r.init();

    // all ok
    return obj_created_run;
}

bool run_function() {
    // verify if the object was created
    if (!obj_created_run) create_objects_run();

    timer_tempo = millis();

    position = (sensor.read_line() - 4500)/100;

    pd_w = pd.simplePD(5, 0.2, position, CONTROL_TIME_MS/1000.0);

    // velocidade toral das rodas
    velLeft  = PWM;  // mudar para pwmLeft
    velRight = PWM; // mudar para pwmRight

    // aplica o double pd_w nas rodas
    velRight = PWM - pd_w;
    velLeft  = PWM + pd_w;

    motor_r.applyPWM((int32_t)velRight);
    motor_l.applyPWM((int32_t)velLeft);

    while (millis() - timer_tempo < CONTROL_TIME_MS);

    // run the robot
    return true;
}

name next_state_run(uint8_t buttons){
    // if button 1 is pressed
    if(buttons & (1 << BIT_0)){
    
        // log message
        #if defined(LOG_ALL) || defined(LOG_INFO)
            Logger::insert_log("states: Run -> Finish", logType::INFO);
        #endif
        
        // brake motors
        motor_l.brake();
        motor_r.brake();

        return FINISH; 
    }

    // if button 2 is pressed
    /*if(buttons & (1 << BIT_1))
        return MAP;

    // if button 3 is pressed
    if(buttons & (1 << BIT_2))
        return RUN;*/

    // stay in the same state
    return RUN;
}

static IRAM_ATTR void sampleISR(void* arg) {
    #if defined(LOG_ALL) || defined(LOG_TELEMETRY)
        if(!(StateMachine::current_state == RUN)) return;
        Logger::insert_log(
                            String(position)            + ";" +
                            String(enc_l.getCount())    + ";" +
                            String(enc_r.getCount())    + ";" +
                            String(pd_w),
                            logType::TELEMETRY);
    #endif
}

#endif