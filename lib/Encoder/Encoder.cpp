#include "Encoder.h"

Encoder *Encoder::usedPCNTs[PCNT_UNIT_MAX] = {
	NULL,
};

// enable ISR on core
static IRAM_ATTR void install_ISR_on_core(void *param) {
	esp_err_t *result = (esp_err_t*) param;
	*result = pcnt_isr_service_install(0);
}

// ISR handler
static void esp32encoder_pcnt_intr_handler(void *param) {   
	Encoder *esp32enc = static_cast<Encoder*>(param);
	pcnt_unit_t unit = esp32enc->enc_config.unit;

	if (PCNT.status_unit[unit].COUNTER_H_LIM) {
        // increment the counter and clear PCNT
		esp32enc->counter = esp32enc->counter + esp32enc->enc_config.counter_h_lim;
		pcnt_counter_clear(unit);
	} else if (PCNT.status_unit[unit].COUNTER_L_LIM) {
        // decrement the counter and clear PCNT
		esp32enc->counter = esp32enc->counter + esp32enc->enc_config.counter_l_lim;
		pcnt_counter_clear(unit);
    } else if (PCNT.status_unit[unit].thres0_lat || PCNT.status_unit[unit].thres1_lat) {
		int16_t c;
		pcnt_get_counter_value(unit, &c);
		esp32enc->counter = esp32enc->counter + c;
		pcnt_set_event_value(unit, PCNT_EVT_THRES_0, -1);
		pcnt_set_event_value(unit, PCNT_EVT_THRES_1, 1);
		pcnt_event_enable(unit, PCNT_EVT_THRES_0);
		pcnt_event_enable(unit, PCNT_EVT_THRES_1);
		pcnt_counter_clear(unit);
	}
}

// constructor
Encoder::Encoder(uint8_t pinA, uint8_t pinB){
    this->pinA = (gpio_num_t) pinA;
    this->pinB = (gpio_num_t) pinB;
    this->counter = 0;
    this->timer = 0;
}

// destructor
Encoder::~Encoder(){
    pcnt_counter_pause(unit);
	pcnt_isr_handler_remove(this->enc_config.unit);
	Encoder::usedPCNTs[unit] = NULL;
}

// init configuration of encoder
bool Encoder::init(uint16_t filter, uint8_t core) {
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
	enc_config.pos_mode         = PCNT_COUNT_DEC;       // Rising-Edge
	enc_config.neg_mode         = PCNT_COUNT_INC;	    // Falling-Edge
	enc_config.lctrl_mode       = PCNT_MODE_KEEP;	    // Rising A on HIGH B = CW Step
	enc_config.hctrl_mode       = PCNT_MODE_REVERSE;    // Rising A on LOW B = CCW Step
	enc_config.counter_h_lim    = _INT16_MAX;            // High limit
	enc_config.counter_l_lim    = _INT16_MIN;            // Low limit

    // Initialize PCNT unit
	pcnt_unit_config(&enc_config);

    // Configure channel 1
	enc_config.pulse_gpio_num   = pinB;                 // make prior control into signal
	enc_config.ctrl_gpio_num    = pinA;                 // and prior signal into control
	enc_config.channel          = PCNT_CHANNEL_1;       // channel 1
	enc_config.pos_mode         = PCNT_COUNT_DEC;       // Rising-Edge
	enc_config.neg_mode         = PCNT_COUNT_INC;       // Falling-Edge
	enc_config.lctrl_mode       = PCNT_MODE_REVERSE;    // Rising A on HIGH B = CW Step 
	enc_config.hctrl_mode       = PCNT_MODE_KEEP;       // Rising A on LOW B = CCW Step 

    // Initialize PCNT unit
	pcnt_unit_config(&enc_config);

    // filter the signal
    if(filter >= 1023){
        filter = 1023;
        pcnt_set_filter_value(unit, filter);
        pcnt_filter_enable(unit);
    } else if (filter == 0) {
        pcnt_filter_disable(unit);
    }

    /* Enable events on maximum and minimum limit values */
	pcnt_event_enable(unit, PCNT_EVT_H_LIM);
	pcnt_event_enable(unit, PCNT_EVT_L_LIM);
	pcnt_counter_pause(unit); // Initial PCNT init
    
    // configure ISR of PCNT
    esp_err_t result = ESP_FAIL;
    esp_err_t er = esp_ipc_call_blocking(core, install_ISR_on_core, &result);

    // return the result of the configuration
    if(er != ESP_OK) return false;

    // Add ISR handler for this unit 
    if(pcnt_isr_handler_add(unit, esp32encoder_pcnt_intr_handler, this) != ESP_OK) return false;

    pcnt_set_event_value(unit, PCNT_EVT_THRES_0, -1);
    pcnt_set_event_value(unit, PCNT_EVT_THRES_1, 1);
    pcnt_event_enable(unit, PCNT_EVT_THRES_0);
    pcnt_event_enable(unit, PCNT_EVT_THRES_1);
	
	pcnt_counter_clear(unit);
	pcnt_intr_enable(unit);
	//pcnt_counter_resume(unit); // Initial PCNT init

    return true;
}

int64_t Encoder::getCountRaw() {
	int16_t c;
	int64_t compensate = 0;

	pcnt_get_counter_value(unit, &c);

	if (PCNT.int_st.val & BIT(unit)) {
		if (PCNT.status_unit[unit].COUNTER_H_LIM) {
			compensate = enc_config.counter_h_lim;
		} else if (PCNT.status_unit[unit].COUNTER_L_LIM) {
			compensate = enc_config.counter_l_lim;
		}
	}

	return compensate + c;
}

int64_t Encoder::getCount() {
	int64_t result = counter + getCountRaw();
	return result;
}

