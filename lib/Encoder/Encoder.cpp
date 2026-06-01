
#include "Encoder.h"

// variable static of PCNT
Encoder *Encoder::usedPCNTs[4] = {
    NULL,
};

bool Encoder::isr_installed = false;
uint8_t Encoder::core_to_run_ISR = 1;

// observe the counter overflow
static bool pcnt_overflow_handle(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {   
    // resolve the counter overflow
    Encoder *esp32enc = static_cast<Encoder*>(user_ctx);
    esp32enc->overflow(edata->watch_point_value);
    return false;
}

// constructor
Encoder::Encoder(uint8_t pinA, uint8_t pinB){
    this->pinA = (gpio_num_t) pinA;
    this->pinB = (gpio_num_t) pinB;
    this->counter = 0;
    this->unit = NULL;
    this->channel_a = NULL;
    this->channel_b = NULL;
}

// destructor
Encoder::~Encoder(){
    if(unit) {
        pcnt_unit_stop(unit);
        pcnt_unit_disable(unit);
        if(channel_a) pcnt_del_channel(channel_a);
        if(channel_b) pcnt_del_channel(channel_b);
        pcnt_del_unit(unit);
    }
}

// init configuration of encoder
bool Encoder::init(uint16_t filter) {
    pcnt_unit_config_t unit_config = {};
    unit_config.high_limit = 32767;
    unit_config.low_limit = -32768;

    // initialize PCNT unit
    if (pcnt_new_unit(&unit_config, &unit) != ESP_OK) return false;

    pcnt_chan_config_t chan_a_config = {};
    chan_a_config.edge_gpio_num = pinA;
    chan_a_config.level_gpio_num = pinB;
    pcnt_new_channel(unit, &chan_a_config, &channel_a);

    pcnt_chan_config_t chan_b_config = {};
    chan_b_config.edge_gpio_num = pinB;
    chan_b_config.level_gpio_num = pinA;
    pcnt_new_channel(unit, &chan_b_config, &channel_b);

    // configure channel 0
    pcnt_channel_set_edge_action(channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(channel_a, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);

    // configure channel 1
    pcnt_channel_set_edge_action(channel_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    // saturate the filter value
    if(filter >= 1023) filter = 1023;

    // filter the signal
    if (filter > 0) {
        pcnt_glitch_filter_config_t filter_config = {};
        filter_config.max_glitch_ns = filter * 1000;
        pcnt_unit_set_glitch_filter(unit, &filter_config);
    }

    /* enable events on maximum and minimum limit values */
    pcnt_unit_add_watch_point(unit, 32767);
    pcnt_unit_add_watch_point(unit, -32768);

    // configure ISR of PCNT
    pcnt_event_callbacks_t cbs = {};
    cbs.on_reach = pcnt_overflow_handle;

    // add ISR handler for this unit 
    if(pcnt_unit_register_event_callbacks(unit, &cbs, this) != ESP_OK) return false;
    
    // pcnt prepared for use
    pcnt_unit_enable(unit);
    pcnt_unit_clear_count(unit);
    pcnt_unit_start(unit);
    return true;
}

int64_t Encoder::getCountRaw() {
    int c = 0;
    if(unit) pcnt_unit_get_count(unit, &c);
    return c;
}

int64_t Encoder::getCount() {
    int64_t result = counter + getCountRaw();
    return result;
}

int64_t Encoder::getCountDiff() {
    int64_t current_count = getCount();
    int64_t diff = current_count - last_counter;
    last_counter = current_count;
    return diff;
}

void Encoder::overflow(int watch_val) {
    // check the counter overflow
    if (watch_val == 32767) {
        // increment the counter and clear PCNT
        counter += 32767;
    } else if (watch_val == -32768) {
        // decrement the counter and clear PCNT
        counter += -32768;
    }
    // clear PCNT
}

void Encoder::clearPCNT() {
    // clear the PCNT
    if(unit) pcnt_unit_clear_count(unit);
    counter = 0;
}

void Encoder::pausePCNT() {
    // pause the PCNT
    if(unit) pcnt_unit_stop(unit);
}

void Encoder::resumePCNT() {
    // resume the PCNT
    if(unit) pcnt_unit_start(unit);
}