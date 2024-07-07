#include <Arduino.h>
#include <pins.h>
#include "stateflow.h"
#include "arraySensor.h"
#include "interrupt.h"

states state = POWER_ON;
arraySensor sensor(8, SIG, C0, C1, C2, C3, WHITE);

void setup() {
    Serial.begin(921600);

    // init pins
	init_pins(); 

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
}

void loop() {

   	// debug
	//Serial.print(sensor.debub());
	//Serial.print(" ");
	//Serial.println(sensor.read_line());

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
