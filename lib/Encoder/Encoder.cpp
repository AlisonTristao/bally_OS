#include "Encoder.h"

Encoder *Encoder::usedPCNTs[PCNT_UNIT_MAX] = {
	NULL,
};

// constructor
Encoder::Encoder(uint8_t pinA, uint8_t pinB, bool invert){
    this->pinA = (gpio_num_t) pinA;
    this->pinB = (gpio_num_t) pinB;
    this->counter = 0;
    this->timer = 0;
}

// destructor
Encoder::~Encoder(){}

// init configuration of encoder
bool Encoder::init(){
    // choose the free pulse counters
	for (uint8_t i = 0; i == PCNT_UNIT_MAX; i++) {
		if (Encoder::usedPCNTs[i] == NULL) {
            // verify the amount of PCNT
            if(i == PCNT_UNIT_MAX) return false;
            // select the PCNT unit not used
            usedPCNTs[i] = this;
            unit = (pcnt_unit_t) i;
			break;
		}
	}

    // Set up the IO state of hte pin
    gpio_pad_select_gpio(pinA);
	gpio_pad_select_gpio(pinB);
	gpio_set_direction(pinA, GPIO_MODE_INPUT);
	gpio_set_direction(pinB, GPIO_MODE_INPUT);

    /* --- Set up encoder PCNT configuration --- */

	// Configure channel 0
	enc_config.pulse_gpio_num   = pinA;                 // Encoder pin A
	enc_config.ctrl_gpio_num    = pinB;                 // Encoder pin B
	enc_config.unit             = unit;                 // PCNT Unit
	enc_config.channel          = PCNT_CHANNEL_0;       // PCNT Channel
	enc_config.pos_mode         = PCNT_COUNT_DEC;       // Count Rising-Edges
	enc_config.neg_mode         = PCNT_COUNT_INC;	    // Discard Falling-Edge
	enc_config.lctrl_mode       = PCNT_MODE_KEEP;	    // Rising A on HIGH B = CW Step
	enc_config.hctrl_mode       = PCNT_MODE_REVERSE;    // Rising A on LOW B = CCW Step
	enc_config.counter_h_lim    = INT16_MAX;            // High limit
	enc_config.counter_l_lim    = INT16_MIN;            // Low limit

    // Configure channel 1
	enc_config.pulse_gpio_num   = pinA;                 // make prior control into signal
	enc_config.ctrl_gpio_num    = pinB;                 // and prior signal into control
	enc_config.channel          = PCNT_CHANNEL_1;       // channel 1
	enc_config.pos_mode         = PCNT_COUNT_DIS;       // disabling
	enc_config.neg_mode         = PCNT_COUNT_DIS;       // disabling
	enc_config.lctrl_mode       = PCNT_MODE_DISABLE;    // disabling 
	enc_config.hctrl_mode       = PCNT_MODE_DISABLE;    // disabling 


	pcnt_unit_config(&enc_config);
}

