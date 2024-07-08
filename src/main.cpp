#include <Arduino.h>
#include <pins.h>
#include "stateflow.h"
#include "arraySensor.h"
#include "interrupt.h"

#include "HBridge.h"
#include <ESP32Encoder.h>

double pwm_ant_D = 0;
double pwm_ant_E = 0;
double pwm_D = 0;
double pwm_E = 0;

states state = POWER_ON;
arraySensor sensor(8, SIG, C0, C1, C2, C3, WHITE);

HBridge motorE(AIN1, AIN2, CH1, PWM_A);
HBridge motorD(BIN1, BIN2, CH2, PWM_B);

ESP32Encoder encoderD;
ESP32Encoder encoderE;

void setup()
{
	Serial.begin(921600);
	Serial.print("Iniciando...\n");

	// init pins
	init_pins();
	motorD.init();
	motorE.init();

	encoderD.attachHalfQuad(A0, A1);
	encoderE.attachHalfQuad(B0, B1);

	encoderD.clearCount();
	encoderE.clearCount();
	Serial.println("direito;esquerdo;PWM(%)");
	randomSeed(analogRead(15));
	// musica = random(0, 3);
	/* Musica_Pantera(); */
}

void loop()
{
	motorD.applyPWM(100);
	motorE.applyPWM(100);
	tempo = millis();
	while (abs(pwm_D - pwm_ant_D) <= 10)
	{
		if (millis() % 5 == 0)
		{
			pwm_D = encoderD.getSpeed();
			pwm_E = encoderE.getSpeed();
			pwm_ant_D = pwm_D;
			pwm_ant_E = pwm_E;
		}

		/*for (int i = 100; i > -110; i -= 10)
		{
			motorD.applyPWM(100);
			// motorE.applyPWM(i);
			delay(10);
			pwm_D = encoderD.getSpeed();
			pwm_E = encoderE.getSpeed();
			// Serial.print(String((double)encoderD.getSpeed()) + ";" + String((double)encoderE.getSpeed()) + ";" + i); // printa a velocidade do motor com base no pwm inserido
			// Serial.println(String(((int)(pwm_D - pwm_ant_D))) + " " + String(((int)(pwm_E - pwm_ant_E)))); // printa a diferença da velocidade com a mudança do pwm
			// Serial.print(" " + String(pwm_ant_D) + " " + String(pwm_ant_E) + " " + String(pwm_D) + " " + String(pwm_E));
			Serial.println(pwm_D);
			pwm_ant_D = pwm_D;
			// pwm_ant_E = pwm_E;*/
     }
  }
}