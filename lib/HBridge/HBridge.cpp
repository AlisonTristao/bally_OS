#include "HBridge.h"

void HBridge::init() {
    ledcSetup(channelPWM, 5000, 12);
    ledcAttachPin(pinPWM, channelPWM);
    mov = STOPPED;
}

void HBridge::changeDirection(movement move) {
    mov = move;
    switch (mov) {
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

void HBridge::applyPWM(int32_t pwm) {
    choiceDirection(saturate(pwm));
    ledcWrite(channelPWM, convertPWM(saturate(pwm)));
}

void HBridge::brake() {
    // para o motor
    changeDirection(STOPPED);
    ledcWrite(channelPWM, convertPWM(100));
}
