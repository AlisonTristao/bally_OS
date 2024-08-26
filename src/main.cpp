#include <Config.h>
#include <Interrupt.h>


StateMachine setup(SETUP, setup_function);

void setup() {
	

}

void loop() {
	StateMachine::run();
	delay(1);
} 