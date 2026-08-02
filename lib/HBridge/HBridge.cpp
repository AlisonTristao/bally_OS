#include "HBridge.h"

HBridge::HBridge(
    uint8_t l1,
    uint8_t l2,
    uint8_t channelPWM1,
    uint8_t channelPWM2,
    uint8_t dead_zone
)
    : l1(l1),
      l2(l2),
      channelPWM1(channelPWM1),
      channelPWM2(channelPWM2),
      dead_zone(dead_zone > SUPLIMIT ? SUPLIMIT : dead_zone)
{
}

esp_err_t HBridge::initTimer()
{
    /*
     * O timer é compartilhado por todas as instâncias de HBridge.
     * Os quatro canais dos dois motores usam o mesmo timer.
     *
     * Esta função deve ser chamada durante a inicialização do sistema,
     * antes de iniciar as tasks que comandam os motores.
     */
    static bool timer_initialized = false;

    if (timer_initialized) {
        return ESP_OK;
    }

    ledc_timer_config_t ledc_timer = {};

    ledc_timer.speed_mode      = PWM_MODE;
    ledc_timer.timer_num       = PWM_TIMER;
    ledc_timer.duty_resolution = PWM_RESOLUTION;
    ledc_timer.freq_hz         = PWM_FREQUENCY_HZ;
    ledc_timer.clk_cfg         = LEDC_AUTO_CLK;

    const esp_err_t err = ledc_timer_config(&ledc_timer);

    if (err == ESP_OK) {
        timer_initialized = true;
    }

    return err;
}

esp_err_t HBridge::initChannel(
    uint8_t pin,
    uint8_t channel
)
{
    const gpio_num_t gpio = static_cast<gpio_num_t>(pin);

    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (channel >= static_cast<uint8_t>(LEDC_CHANNEL_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = gpio_reset_pin(gpio);

    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t ledc_channel = {};

    ledc_channel.speed_mode = PWM_MODE;
    ledc_channel.channel =
        static_cast<ledc_channel_t>(channel);

    ledc_channel.timer_sel = PWM_TIMER;
    ledc_channel.gpio_num  = pin;
    ledc_channel.duty      = 0;
    ledc_channel.hpoint    = 0;

    return ledc_channel_config(&ledc_channel);
}

esp_err_t HBridge::init()
{
    if (initialized) {
        return ESP_OK;
    }

    if (l1 == l2 || channelPWM1 == channelPWM2) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = initTimer();

    if (err != ESP_OK) {
        return err;
    }

    err = initChannel(l1, channelPWM1);

    if (err != ESP_OK) {
        return err;
    }

    err = initChannel(l2, channelPWM2);

    if (err != ESP_OK) {
        return err;
    }

    initialized = true;

    pwm = 0;
    mov = STOPPED;
    last_mov = FORWARD;

    return coast();
}

esp_err_t HBridge::setDuty(
    uint8_t channel,
    uint32_t duty
)
{
    if (duty > PWM_MAX_DUTY) {
        duty = PWM_MAX_DUTY;
    }

    return ledc_set_duty_and_update(
        PWM_MODE,
        static_cast<ledc_channel_t>(channel),
        duty,
        0
    );
}

int32_t HBridge::saturate(int32_t new_pwm) const
{
    if (new_pwm > SUPLIMIT) {
        return SUPLIMIT;
    }

    if (new_pwm < INFLIMIT) {
        return INFLIMIT;
    }

    return new_pwm;
}

movement HBridge::choiceDirection(int32_t command) const
{
    if (command > 0) {
        return FORWARD;
    }

    if (command < 0) {
        return BACKWARD;
    }

    return STOPPED;
}

uint32_t HBridge::convertPWM(int32_t command) const
{
    /*
     * A saturação já limita o valor entre -100 e +100,
     * então não existe risco ao calcular o módulo.
     */
    const uint32_t magnitude =
        command < 0
            ? static_cast<uint32_t>(-command)
            : static_cast<uint32_t>(command);

    if (magnitude == 0) {
        return 0;
    }

    /*
     * Converte:
     *
     * comando 1...100
     *
     * para:
     *
     * dead_zone...100 %
     *
     * Foi utilizada aritmética inteira para evitar o erro de
     * divisão inteira da implementação anterior.
     */
    const uint32_t available_range =
        SUPLIMIT - dead_zone;

    const uint32_t effective_percent =
        dead_zone +
        ((magnitude * available_range) + (SUPLIMIT / 2)) /
        SUPLIMIT;

    return (
        (effective_percent * PWM_MAX_DUTY) +
        (SUPLIMIT / 2)
    ) / SUPLIMIT;
}

esp_err_t HBridge::enterBrakeState()
{
    esp_err_t err;

    /*
     * O pino responsável pelo sentido atual já está em nível
     * máximo. Ele é atualizado primeiro para impedir um pulso
     * no sentido contrário durante a transição.
     */
    if (mov == BACKWARD ||
        (mov == STOPPED && last_mov == BACKWARD)) {

        err = setDuty(channelPWM2, PWM_MAX_DUTY);

        if (err != ESP_OK) {
            return err;
        }

        err = setDuty(channelPWM1, PWM_MAX_DUTY);
    }
    else {
        err = setDuty(channelPWM1, PWM_MAX_DUTY);

        if (err != ESP_OK) {
            return err;
        }

        err = setDuty(channelPWM2, PWM_MAX_DUTY);
    }

    return err;
}

esp_err_t HBridge::drive(
    movement direction,
    uint32_t driveDuty
)
{
    if (direction != FORWARD &&
        direction != BACKWARD) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Ao trocar diretamente o sentido, primeiro colocamos
     * a ponte no estado de frenagem.
     */
    const bool reversing =
        (mov == FORWARD && direction == BACKWARD) ||
        (mov == BACKWARD && direction == FORWARD);

    if (reversing) {
        const esp_err_t err = enterBrakeState();

        if (err != ESP_OK) {
            return err;
        }
    }

    /*
     * PWM drive/brake:
     *
     * FORWARD:
     *   IN1 permanece em 1.
     *   IN2 alterna entre 0 (drive) e 1 (brake).
     *
     * BACKWARD:
     *   IN2 permanece em 1.
     *   IN1 alterna entre 0 (drive) e 1 (brake).
     */
    const uint32_t brakeDuty =
        PWM_MAX_DUTY - driveDuty;

    esp_err_t err;

    if (direction == FORWARD) {
        err = setDuty(channelPWM1, PWM_MAX_DUTY);

        if (err != ESP_OK) {
            return err;
        }

        err = setDuty(channelPWM2, brakeDuty);
    }
    else {
        err = setDuty(channelPWM2, PWM_MAX_DUTY);

        if (err != ESP_OK) {
            return err;
        }

        err = setDuty(channelPWM1, brakeDuty);
    }

    if (err == ESP_OK) {
        mov = direction;
        last_mov = direction;
    }

    return err;
}

esp_err_t HBridge::applyPWM(int32_t new_pwm)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const int32_t command = saturate(new_pwm);
    const movement direction = choiceDirection(command);

    /*
     * Diferentemente da implementação anterior, PWM igual
     * a zero não mantém uma direção ativa nem aplica a
     * dead zone. Ele freia corretamente o motor.
     */
    if (direction == STOPPED) {
        return brake();
    }

    if (command == pwm && direction == mov) {
        return ESP_OK;
    }

    const uint32_t duty = convertPWM(command);

    const esp_err_t err = drive(direction, duty);

    if (err == ESP_OK) {
        pwm = command;
    }

    return err;
}

esp_err_t HBridge::brake()
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t err = enterBrakeState();

    if (err == ESP_OK) {
        pwm = 0;
        mov = BRAKING;
    }

    return err;
}

esp_err_t HBridge::coast()
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err;

    /*
     * Primeiro desativa a entrada oposta ao último sentido.
     * Isso evita um pulso no sentido contrário durante a
     * transição para roda livre.
     */
    if (last_mov == BACKWARD) {
        err = setDuty(channelPWM1, 0);

        if (err != ESP_OK) {
            return err;
        }

        err = setDuty(channelPWM2, 0);
    }
    else {
        err = setDuty(channelPWM2, 0);

        if (err != ESP_OK) {
            return err;
        }

        err = setDuty(channelPWM1, 0);
    }

    if (err == ESP_OK) {
        pwm = 0;
        mov = STOPPED;
    }

    return err;
}