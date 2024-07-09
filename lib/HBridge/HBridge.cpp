#include "HBridge.h"

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
    ledcSetup(channelPWM, 5000, 12);
    ledcAttachPin(pinPWM, channelPWM);
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
        digitalWrite(l1, HIGH);
        digitalWrite(l2, LOW);
        break;

    case BACKWARD:
        digitalWrite(l1, LOW);
        digitalWrite(l2, HIGH);
        break;

    case STOPPED:
        digitalWrite(l1, HIGH);
        digitalWrite(l2, HIGH);
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
    ledcWrite(channelPWM, abs(pwm));
}
void HBridge::brake()
{
    // para o motor
    changeDirection(STOPPED);
    ledcWrite(channelPWM, 4095);
}
