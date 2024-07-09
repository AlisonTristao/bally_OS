#include "Control.h"
#include <Arduino.h>

#define VERB


Control::Control()
{

}

double Control::simplePID(double kp, double ki, double kd, double Erro)
{
    double Ts = 0.05; // tempo de amostragem

    /// anti windup
    aw = (ki - 0.1) * Ts * (limit - pid);
    k1 = (kp + ki * Ts);
    k2 = -kp;

    // salva o ultimo erro
    pid += (k1 * Erro) + (k2 * ErroPassado) + aw + ((Erro - ErroPassado) / Ts) * kd; // pid com anti-windup

    // garante que o P e o D nao sature
    if (pid >= limit)
        pid = limit;
    else if (pid <= -limit)
        pid = -limit;

    #ifdef VERB // modo verboso
        Serial.printf("PID: %.2f\t", pid);
        Serial.printf("Erro passado: %.2f,\t Erro: %.2f\n", integral, Erro);
    #endif

    ErroPassado = Erro;
    return pid;
}

