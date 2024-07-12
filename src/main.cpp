#include <Arduino.h>
#include <pins.h>
#include "stateflow.h"
#include "arraySensor.h"
#include "leds.h"
#include "interrupt.h"
#include "Control.h"
#include <vector>

#include "HBridge.h"
#include <ESP32Encoder.h>

using namespace std;

// amostragem flag
bool timer_flag = false;
uint32_t time_start = 0;
uint32_t cont = 0;

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
vector<double> countD, countE, sen_E, sen_D, PWM_D, PWM_E, timer_gabriel, ctrl, erro;

// control
Control controlW; // VELOCIDADE ANGULAR
Control controlT; // VELOCIDADE LINEAR

Control controle0;

double uTE = 0.0;
double uTD = 0.0;

int16_t PWM = 65;
double position = 0.0, pid0 = 0.0;

int16_t velLeft = 0;
int16_t velRight = 0;

uint32_t timer_tempo = 0;
double kp = 3;

void setup()
{
	Serial.begin(921600);

	// init pins
	init_pins();

	/*uint32_t tempoo = millis();
	while(digitalRead(BTN1) || digitalRead(BTN2)){
		if(millis() - tempoo > 3000){
			kp = 3.2;
			PWM = 60;
			Serial.println("NADA");
			break;
		}
		if(!digitalRead(BTN1)) {
			PWM = 50;
			Serial.println(PWM);
			break;
		}
		if(!digitalRead(BTN2)) {
			kp = 3;
			Serial.println(kp);
			break;
		}
	}*/

	// init interruptions
	set_all_interruptions();

	// init motors
	motorD.init();
	motorE.init();

	// init encoders
	encoderD.attachHalfQuad(ENC_A0, ENC_A1);
	encoderE.attachHalfQuad(ENC_B0, ENC_B1);

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
		timer_tempo = millis();

		// posicao da linha
		position = (sensor.read_line() - 4500) / 100;

		pid0 = controle0.simplePID(kp, 0.01, 0.04, position, SAMPLE_MS / 1000.0);
		// Serial.println(pid0);

		// velocidade toral das rodas
		velLeft = PWM;  // mudar para pwmLeft
		velRight = PWM; // mudar para pwmRight

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

		while (millis() - timer_tempo < SAMPLE_MS);
		break;
	}

	case DEBUG: {

		motorD.brake();
		motorE.brake();

       Serial.println("INICIO");
		for(int i = 0; i < countD.size(); i++){
			Serial.print("A:");
			Serial.println((int)countD[i]);
			delay(10);
			Serial.print("B:");
			Serial.println((int)countE[i]);
			delay(10);
			Serial.print("sE:");
			Serial.println((int)sen_E[i]);
			delay(10);
			Serial.print("sD:");
			Serial.println((int)sen_D[i]);
			delay(10);
			Serial.print("pD:");
			Serial.println((int)PWM_D[i]);
			delay(10);
			Serial.print("pE:");
			Serial.println((int)PWM_E[i]);
			delay(10);
			Serial.print("tG:");
			Serial.println((int)timer_gabriel[i]);
			delay(10);
			Serial.print("cT:");
			Serial.println((int)ctrl[i]);
			delay(10);
			Serial.print("eR:");
			Serial.println((int)erro[i]);
			delay(10);
		}
		Serial.println("FIM");
		state_machine();

		/* antigo
		
		Serial.println("INICIO");
		for(int i = 0; i < speedD.size(); i++){
			Serial.print("E:");
			Serial.println(speedE[i]);
			delay(10);
			Serial.print("D:");
			Serial.println(speedD[i]);
			delay(10);
			Serial.print("S:");
			Serial.println(s_d[i]);
			delay(10);
		}
		Serial.println("FIM");
		state_machine();

		digitalWrite(LED5, LOW);
		digitalWrite(LED4, HIGH);
		delay(DLY_LONG);
		digitalWrite(LED5, HIGH);
		digitalWrite(LED4, LOW);
		delay(DLY_LONG);
		
		Serial.println()    countD
        Serial.println()    countE
        Serial.println()    sen_E
        Serial.println()    PWM_D
        Serial.println()    PWM_E
        Serial.println()    timer_gabriel
        Serial.println()    ctrl
        Serial.println()    erro
        Serial.println()    bat
		
		*/

	}

	default:
		break;
	}
}