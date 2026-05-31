#ifndef WHEELS_H
#define WHEELS_H
#include <Arduino.h>

#define SUPLIMIT 100
#define INFLIMIT -100

// sentido da rotação das todas
enum movement {
    FORWARD,
    BACKWARD,
    STOPPED
};

// motor objeto
class HBridge {
    public:
        HBridge(uint8_t l1, uint8_t l2, uint8_t channelPWM, uint8_t pinPWM, uint8_t dead_zone = 0) : 
                l1(l1), l2(l2), channelPWM(channelPWM), pinPWM(pinPWM), dead_zone(dead_zone) {};
        virtual ~HBridge(){};
        void init();
        void applyPWM(int32_t pwm);
        void brake();
    private:
        void changeDirection(movement mov);
        void choiceDirection(int32_t pwm);
        int32_t convertPWM(int32_t pwm);
        int32_t saturate(int32_t pwm);
        movement mov;
        const uint8_t l1;
        const uint8_t l2;
        const uint8_t channelPWM;
        const uint8_t pinPWM;
        const uint8_t dead_zone;
        const float gain = 40.95;
};

#endif