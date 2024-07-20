#include "Encoder.h"

// variable static of PCNT
Encoder *Encoder::usedPCNTs[PCNT_UNIT_MAX] = {
	NULL,
};

bool Encoder::isr_installed = false;
uint8_t Encoder::core_to_run_ISR = 1;

// enable ISR on core
static void IRAM_ATTR install_ISR_on_core(void *param) {
	esp_err_t *result = (esp_err_t*) param;
	*result = pcnt_isr_service_install(0);
}

// observe the counter overflow
static void IRAM_ATTR pcnt_overflow_handle(void *param) {   
	// resolve the counter overflow
	Encoder *esp32enc = static_cast<Encoder*>(param);
	esp32enc->overflow();
}

// constructor
Encoder::Encoder(uint8_t pinA, uint8_t pinB){
    this->pinA = (gpio_num_t) pinA;
    this->pinB = (gpio_num_t) pinB;
    this->counter = 0;
}

// destructor
Encoder::~Encoder(){
    pcnt_counter_pause(unit);
	pcnt_isr_handler_remove(this->enc_config.unit);
	Encoder::usedPCNTs[unit] = NULL;
}

// init configuration of encoder
bool Encoder::init(uint16_t filter) {
    // choose the free pulse counters
	for (uint8_t i = 0; i <= PCNT_UNIT_MAX; i++) {
		if (Encoder::usedPCNTs[i] == NULL) {
            // verify the amount of PCNT
            if(i == PCNT_UNIT_MAX) return false;
            // select the PCNT unit not used
            usedPCNTs[i] = this;
            unit = (pcnt_unit_t) i;
            break;
		}
	}

    // set up the IO state of hte pin
    gpio_pad_select_gpio(pinA);
	gpio_pad_select_gpio(pinB);
	gpio_set_direction(pinA, GPIO_MODE_INPUT);
	gpio_set_direction(pinB, GPIO_MODE_INPUT);

    /* --- set encoder PCNT configuration --- */

	// configure channel 0
	enc_config.pulse_gpio_num   = pinA;                 // Encoder pin A
	enc_config.ctrl_gpio_num    = pinB;                 // Encoder pin B
	enc_config.unit             = unit;                 // PCNT Unit
	enc_config.channel          = PCNT_CHANNEL_0;       // PCNT Channel
	enc_config.pos_mode         = PCNT_COUNT_DEC;       // Rising-Edge
	enc_config.neg_mode         = PCNT_COUNT_INC;	    // Falling-Edge
	enc_config.lctrl_mode       = PCNT_MODE_KEEP;	    // Rising A on HIGH B = CW Step
	enc_config.hctrl_mode       = PCNT_MODE_REVERSE;    // Rising A on LOW B = CCW Step
	enc_config.counter_h_lim    = INT16_MAX;            // High limit
	enc_config.counter_l_lim    = INT16_MIN;            // Low limit

    // initialize PCNT unit
	pcnt_unit_config(&enc_config);

    // configure channel 1
	enc_config.pulse_gpio_num   = pinB;                 // make prior control into signal
	enc_config.ctrl_gpio_num    = pinA;                 // and prior signal into control
	enc_config.channel          = PCNT_CHANNEL_1;       // channel 1
	enc_config.lctrl_mode       = PCNT_MODE_REVERSE;    // Rising B on HIGH A = CW Step 
	enc_config.hctrl_mode       = PCNT_MODE_KEEP;       // Rising B on LOW A = CCW Step 

    // initialize PCNT unit
	pcnt_unit_config(&enc_config);

    // saturate the filter value
    if(filter >= 1023) filter = 1023;

    // filter the signal
    if (filter == 0) {
        pcnt_filter_disable(unit);
    } else {
        pcnt_set_filter_value(unit, filter);
        pcnt_filter_enable(unit);
    }

    /* enable events on maximum and minimum limit values */
	pcnt_event_enable(unit, PCNT_EVT_H_LIM);
	pcnt_event_enable(unit, PCNT_EVT_L_LIM);
    
    // configure ISR of PCNT
    if(!isr_installed) {
        esp_err_t result = ESP_FAIL;
        esp_err_t er = esp_ipc_call_blocking(core_to_run_ISR, install_ISR_on_core, &result);

        // return the result of the configuration
        if(er != ESP_OK) return false;

        isr_installed = true;
    }

    // add ISR handler for this unit 
    if(pcnt_isr_handler_add(unit, pcnt_overflow_handle, this) != ESP_OK) return false;
	
    // pcnt prepared for use
	pcnt_counter_clear(unit);
	pcnt_intr_enable(unit);
	pcnt_counter_resume(unit);
    return true;
}

int64_t Encoder::getCountRaw() {
	int16_t c;
	int64_t compensate = 0;

	pcnt_get_counter_value(unit, &c);

	if (PCNT.int_st.val & BIT(unit)) {
		if (PCNT.status_unit[unit].cnt_thr_h_lim_lat_un) {
			compensate = enc_config.counter_h_lim;
		} else if (PCNT.status_unit[unit].cnt_thr_l_lim_lat_un) {
			compensate = enc_config.counter_l_lim;
		}
	}

	return compensate + c;
}

int64_t Encoder::getCount() {
	int64_t result = counter + getCountRaw();
	return result;
}

void Encoder::overflow() {
	// check the counter overflow
	if (PCNT.status_unit[unit].cnt_thr_h_lim_lat_un) {
        // increment the counter and clear PCNT
		counter += enc_config.counter_h_lim;
	} else if (PCNT.status_unit[unit].cnt_thr_l_lim_lat_un) {
        // decrement the counter and clear PCNT
		counter += enc_config.counter_l_lim;
    }
	// clear PCNT
	pcnt_counter_clear(unit);
}

void Encoder::clearPCNT() {
	// clear the PCNT
	pcnt_counter_clear(unit);
	counter = 0;
}

void Encoder::pausePCNT() {
	// pause the PCNT
	pcnt_counter_pause(unit);
}

void Encoder::resumePCNT() {
	// resume the PCNT
	pcnt_counter_resume(unit);
}
