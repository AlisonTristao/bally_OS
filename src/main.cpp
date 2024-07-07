#include <Arduino.h>
#include <pins.h>
#include "stateflow.h"
#include "arraySensor.h"
#include "interrupt.h"

#include "HBridge.h"
#include <ESP32Encoder.h>
#include "Encoder.h"

states state = POWER_ON;
arraySensor sensor(8, SIG, C0, C1, C2, C3, WHITE);


HBridge motorE(AIN1, AIN2, CH1, PWM_A);
HBridge motorD(BIN1, BIN2, CH2, PWM_B);

Encoder enc_left(A0, A1);
Encoder enc_right(B0, B1);


void setup() {
    Serial.begin(921600);


    ESP32Encoder encoderD;
    ESP32Encoder encoderE;

    enc_left.init();
    enc_right.init();

    encoderD.attachHalfQuad(A0, A1);
	  encoderE.attachHalfQuad(B0, B1);

    // init pins
    init_pins();
    motorD.init();
    motorE.init();
    motorD.applyPWM(2000);
    motorE.applyPWM(-1000);
  
    // Set omterruptions
    set_all_interruptions();

	  encoderD.clearCount();
	  encoderE.clearCount();
}

void loop()
{
  // debug
	//Serial.print(sensor.debub());
	//Serial.print(" ");
	//Serial.println(sensor.read_line());
    Serial.print(enc_left.getCount());
    Serial.print(" ");
    Serial.println(enc_right.getSpeed());
  
  	Serial.print(String((double)encoderD.getSpeed()));
	  Serial.println(" " + String((double)encoderE.getSpeed()));
	  delay(100);
  
    // wait 3 seconds
	  Serial.println(sensor.calibrate_status());

    // clear count
    enc_left.clearCount();
    enc_right.clearCount();
  
    // calibrate sensor
	  sensor.calibrate(30, 100, LED0);
  
    switch (state){ 
        
		case POWER_ON:
            state_1();
			break;

		case CALIBRATION:
            state_2();
			break;

		case RUNNING:
            state_3();
			break;
		
		default:
			break;
	}
}
