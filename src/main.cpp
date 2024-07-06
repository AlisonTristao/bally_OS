#include <Arduino.h>
#include "arraySensor.h"
#include <pins.h>
#include "stateflow.h"
#include "interrupt.h"

// obj sensor
arraySensor sensor(8, SIG, C0, C1, C2, C3, WHITE);

states state = POWER_ON;

void setup(){
	// init serial
	Serial.begin(921600);
	
	// init pins
	init_pins();

	set_all_interruptions();

	// set init array senso0r
	sensor.set_init_arr(4);

	// init sensor
	sensor.init();

	// calibrate sensor
	sensor.calibrate(30, 100, LED0);

	// wait 3 seconds
	Serial.println(sensor.calibrate_status());

	delay(3000);
}

void loop(){
	//set_all_interruptions();
	
	// debug
	//Serial.print(sensor.debub());
	//Serial.print(" ");
	//Serial.println(sensor.read_line());
	
	switch (state){ 

		case POWER_ON:
			Serial.println(" ESTADO 1");
			break;

		case CALIBRATION:
			Serial.println("ESTADO 2");
			break;

		case RUNNING:
			Serial.println("ESTADO 3");
			break;

		case STANDBY:
			Serial.println("ESTADO 4");
			break;
		
		default:
			break;
	}

}



