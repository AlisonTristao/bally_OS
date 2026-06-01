#include "HBridge.h"

void HBridge::init() {
    gpio_reset_pin((gpio_num_t)l1);
    gpio_set_direction((gpio_num_t)l1, GPIO_MODE_OUTPUT);
    
    gpio_reset_pin((gpio_num_t)l2);
    gpio_set_direction((gpio_num_t)l2, GPIO_MODE_OUTPUT);

    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_0;
    ledc_timer.duty_resolution  = LEDC_TIMER_12_BIT;
    ledc_timer.freq_hz          = 5000;
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel        = (ledc_channel_t)channelPWM;
    ledc_channel.timer_sel      = LEDC_TIMER_0;
    //ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num       = pinPWM;
    ledc_channel.duty           = 0;
    ledc_channel.hpoint         = 0;
    ledc_channel_config(&ledc_channel);

    mov = STOPPED;
}

void HBridge::changeDirection(movement move) {
    mov = move;
    switch (mov) {
    case FORWARD:
        gpio_set_level((gpio_num_t)l1, 1);
        gpio_set_level((gpio_num_t)l2, 0);
        break;
    case BACKWARD:
        gpio_set_level((gpio_num_t)l1, 0);
        gpio_set_level((gpio_num_t)l2, 1);
        break;
    case STOPPED:
        gpio_set_level((gpio_num_t)l1, 1);
        gpio_set_level((gpio_num_t)l2, 1);
        break;
    default:
        break;
    }
}

int32_t HBridge::convertPWM(int32_t pwm) {
    return (dead_zone + (pwm/SUPLIMIT) * (SUPLIMIT - dead_zone)) * gain;
}

int32_t HBridge::saturate(int32_t pwm) {
    if (pwm > SUPLIMIT)
        return SUPLIMIT;
    if (pwm < INFLIMIT)
        return INFLIMIT;
    return pwm;
}

void HBridge::choiceDirection(int32_t pwm) {
    // define a direção
    movement move = FORWARD;
    if (pwm > 0)
        move = FORWARD;
    if (pwm < 0)
        move = BACKWARD;

    // muda a direção
    if (mov != move)
        changeDirection(move);
}

void HBridge::applyPWM(int32_t new_pwm) {
    if (abs(new_pwm - pwm) < 3)
        return;
    pwm = new_pwm;
    choiceDirection(saturate(pwm));
    uint32_t duty = convertPWM(saturate(pwm));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channelPWM, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channelPWM);
}

void HBridge::brake() {
    // para o motor
    changeDirection(STOPPED);
    uint32_t duty = convertPWM(100);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channelPWM, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channelPWM);
}