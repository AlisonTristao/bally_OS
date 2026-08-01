#ifndef HBRIDGE_H
#define HBRIDGE_H

#include <cstdint>

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define SUPLIMIT  100
#define INFLIMIT -100

// Estado atual da ponte H
enum movement : uint8_t {
    FORWARD,
    BACKWARD,
    STOPPED,
    BRAKING
};

class HBridge {
public:
    /*
     * l1 e l2 são diretamente IN1 e IN2 do DRV8251A.
     *
     * Agora são necessários dois canais PWM:
     * - channelPWM1 controla l1/IN1;
     * - channelPWM2 controla l2/IN2.
     *
     * dead_zone é dado em porcentagem, entre 0 e 100.
     */
    HBridge(
        uint8_t l1,
        uint8_t l2,
        uint8_t channelPWM1,
        uint8_t channelPWM2,
        uint8_t dead_zone = 0
    );

    virtual ~HBridge() = default;

    esp_err_t init();

    /*
     * Comando entre -100 e +100:
     *
     *  +100 = máxima velocidade para frente;
     *     0 = freio ativo;
     *  -100 = máxima velocidade para trás.
     */
    esp_err_t applyPWM(int32_t pwm);

    // IN1 = 1 e IN2 = 1
    esp_err_t brake();

    // IN1 = 0 e IN2 = 0
    esp_err_t coast();

    int32_t getPWM() const {
        return pwm;
    }

    movement getMovement() const {
        return mov;
    }

private:
    static constexpr ledc_mode_t PWM_MODE =
        LEDC_LOW_SPEED_MODE;

    static constexpr ledc_timer_t PWM_TIMER =
        LEDC_TIMER_0;

    static constexpr ledc_timer_bit_t PWM_RESOLUTION =
        LEDC_TIMER_12_BIT;

    static constexpr uint32_t PWM_FREQUENCY_HZ =
        5000;

    static constexpr uint32_t PWM_MAX_DUTY =
        4095;

    static esp_err_t initTimer();

    esp_err_t initChannel(
        uint8_t pin,
        uint8_t channel
    );

    esp_err_t setDuty(
        uint8_t channel,
        uint32_t duty
    );

    esp_err_t enterBrakeState();

    esp_err_t drive(
        movement direction,
        uint32_t driveDuty
    );

    movement choiceDirection(
        int32_t pwm
    ) const;

    uint32_t convertPWM(
        int32_t pwm
    ) const;

    int32_t saturate(
        int32_t pwm
    ) const;

    int32_t pwm = 0;

    movement mov = STOPPED;
    movement last_mov = FORWARD;

    const uint8_t l1;
    const uint8_t l2;

    const uint8_t channelPWM1;
    const uint8_t channelPWM2;

    const uint8_t dead_zone;

    bool initialized = false;
};

#endif // HBRIDGE_H