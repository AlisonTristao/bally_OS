#include <Arduino.h>
#include <pins.h>
#include "stateflow.h"
#include "arraySensor.h"
#include "leds.h"
#include "interrupt.h"
#include "Control.h"
#include <vector>
#include "debug.h"

#include "HBridge.h"
#include <ESP32Encoder.h>

using namespace std;

// init satate machine
states state = POWER_ON;
bool calibrated = false; // flag to check if the sensor is calibrated

// sensor array
arraySensor sensor(8, SIG, C0, C1, C2, C3, WHITE);

// motors
HBridge motorE(AIN1, AIN2, CH0, PWM_A);
HBridge motorD(BIN1, BIN2, CH1, PWM_B);

// encoders
ESP32Encoder encoderD;
ESP32Encoder encoderE;
double speedD, speedE, vm;

// control
Control controlW; // VELOCIDADE ANGULAR
Control controlT; // VELOCIDADE LINEAR

Control controle0;
Control controle1;

int16_t PWM = 65;
double position = 0.0, pid0 = 0.0;

int16_t velLeft = 0;
int16_t velRight = 0;

uint32_t timer = 0;

void setup()
{
	Serial.begin(921600);

	// init pins
	init_pins();

	// init interruptions
	set_all_interruptions();

	// init motors
	motorD.init();
	motorE.init();

	// init encoders
	encoderD.attachHalfQuad(A0, A1);
	encoderE.attachHalfQuad(B0, B1);

	// init sensor
	sensor.init();
	sensor.set_init_arr(4); // mux in the middle
}

uint32_t i = 0;
void loop()
{
	switch (state)
	{
	case POWER_ON:
	{
		// brake the wheels
		motorD.brake();
		motorE.brake();
		
		/* musica e led pisca s*/
		power_func();
		break;
	}

	case CALIBRATION:
	{
		/* calibrate sensors */
		if (!calibrated)
			calibrated = calibrate(&sensor);
		else
			ledUp((uint8_t)map(sensor.read_line(), 1000, 8000, 0, 5));
		break;
	}

	case RUNNING:
	{
		timer = millis();

		// get speed of motors
		speedD = (0.87*speedD) + (0.13*encoderD.getSpeed());
		speedE = (0.87*speedE) + (0.13*encoderE.getSpeed());
		vm = (speedD + speedE)/2;

		// controle linear
		PWM = controle1.Gabes_Control(1, 1000, 0, 2100-vm, SAMPLE_MS/1000.0);

		Serial.println(PWM);

		// posicao da linha
		position = (sensor.read_line() - 4500) / 100;

		pid0 = controle0.Gabes_Control(2.5, 250, 0.016, position, SAMPLE_MS / 1000.0);
		// Serial.println(pid0);

		// velocidade toral das rodas
		velLeft = PWM;
		velRight = PWM;

		// aplica o pid0 nas rodas
		if (pid0 > 0)
			velRight = PWM - pid0;
		else
			velLeft = PWM + pid0;

		motorD.applyPWM((int32_t)velRight);
		motorE.applyPWM((int32_t)velLeft);

		// digitalWrite(LED2, digitalRead(LEFT));
		// digitalWrite(LED5, digitalRead(RIGHT));
		running_func();

		while (millis() - timer < SAMPLE_MS);
		break;
	}

	case DEBUG: {
		motorD.brake();
		motorE.brake();
	}

	default:
		break;
	}
}