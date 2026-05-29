#include "HBridge.h"
#include <driver/gpio.h>
#include <driver/ledc.h>

HBridge::HBridge(uint8_t l1, uint8_t l2, uint8_t channelPWM, uint8_t pinPWM)
{
    // define os pinos
    this->l1 = l1;
    this->l2 = l2;
    this->channelPWM = channelPWM;
    this->pinPWM = pinPWM;
}
void HBridge::init()
{
    // configura o pwm
    ledc_timer_config_t timer_cfg = {};
    timer_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_cfg.timer_num = LEDC_TIMER_0;
    timer_cfg.duty_resolution = LEDC_TIMER_12_BIT;
    timer_cfg.freq_hz = 5000;
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t channel_cfg = {};
    channel_cfg.gpio_num = pinPWM;
    channel_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_cfg.channel = static_cast<ledc_channel_t>(channelPWM);
    channel_cfg.intr_type = LEDC_INTR_DISABLE;
    channel_cfg.timer_sel = LEDC_TIMER_0;
    channel_cfg.duty = 0;
    channel_cfg.hpoint = 0;
    ledc_channel_config(&channel_cfg);
    // define a direcao
    mov = STOPPED;
}
void HBridge::changeDirection(movement move)
{
    // altera a rotação do motor
    mov = move;

    // define a rotação
    switch (mov)
    {
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
void HBridge::applyPWM(int32_t pwm)
{
    // limita o pwm
    if (pwm > +100)
        pwm = +100;
    if (pwm < -100)
        pwm = -100;
    // normaliza o pwm (tira da porcentagem)
    pwm = pwm * 40.95;
    // define a direção
    movement move = FORWARD;
    if (pwm > 0)
        move = FORWARD;
    if (pwm < 0)
        move = BACKWARD;

    // muda a direção
    if (mov != move)
        changeDirection(move);

    // aplica o pwm
    uint32_t duty = static_cast<uint32_t>(abs(pwm));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channelPWM), duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channelPWM));
}
void HBridge::brake()
{
    // para o motor
    changeDirection(STOPPED);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channelPWM), 4095);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channelPWM));
}
