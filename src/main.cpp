#include <Arduino.h>
#include <pins.h>
#include "stateflow.h"
#include "arraySensor.h"
#include "interrupt.h"
#include "Encoder.h"

states state = POWER_ON;
arraySensor sensor(8, SIG, C0, C1, C2, C3, WHITE);

Encoder enc_left(A0, A1);
Encoder enc_right(B0, B1);


void setup() {
    Serial.begin(921600);

    // init pins
	init_pins(); 

    enc_left.init();
    enc_right.init();

    // Set omterruptions
    set_all_interruptions();

    // set init array senso0r
	sensor.set_init_arr(4);

	// init sensor
	sensor.init();

	// calibrate sensor
	sensor.calibrate(30, 100, LED0);

	// wait 3 seconds
	Serial.println(sensor.calibrate_status());

    // clear count
    enc_left.clearCount();
    enc_right.clearCount();
}

void loop() {

   	// debug
	//Serial.print(sensor.debub());
	//Serial.print(" ");
	//Serial.println(sensor.read_line());
    Serial.print(enc_left.getCount());
    Serial.print(" ");
    Serial.println(enc_right.getSpeed());

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
