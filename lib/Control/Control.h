#ifndef CONTROL_H
#define CONTROL_H

// autor Greg
#include <cstdint>
#include <Arduino.h>

class Control
{
public:
    // construtor
    Control();
    virtual ~Control(){};
    void setVerb(bool verb);
    double simplePID(double kp, double ki, double kd, double Erro, double sample);
    int32_t simplePD(double kp, double kd, double Erro, int32_t limit);
    int32_t simplePI(double kp, double ki, double Erro, int32_t limit);
private:
    uint32_t Time = 0;
    double k1, k2, ka;
    double Erro = 0, ErroPassado = 0, Ie = 0;
    double pid = 0, integral = 0, aw = 0;
    double limit = 100;
};

#endif