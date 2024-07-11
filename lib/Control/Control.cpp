#include "Control.h"
#include <Arduino.h>

// #define VERB

Control::Control()
{
}

double Control::simplePID(double kp, double ki, double kd, double Erro, double sample)
{
    /// anti windup
    // aw = (ki - 0.1) * sample * (limit - pid);
    // k1 = (kp + ki * sample);
    // k2 = -kp;

    // salva o ultimo erro
    // pid += (k1 * Erro) + (k2 * ErroPassado) + aw + ((Erro - ErroPassado) / sample) * kd; // pid com anti-windup

    integral += Erro * sample;

    pid = kp * Erro + ki * integral + kd * (Erro - ErroPassado) / sample; // pid sem anti-windup

    if (ki * integral > limit)
        integral = limit / ki;
    else if (ki * integral < -limit)
        integral = -limit / ki;

    // pid = kp * Erro + ki * integral; // pid sem anti-windup

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

double Control::Gabes_Control(double kp, double ki, double kd, double Erro, double sample)
{
    kc = kp;
    td = kd / kc;
    ti = kc / ki;
    ef = ((td / 10) / (td / 10 + sample)) * ef + (sample / (td / 10 + sample)) * Erro;
    d_ef = (ef - ef_ant);

    u = u_ant + kc * d_ef + (kc / ti) * sample * ef + (kc * td) * ((d_ef) / sample - (d_ef_ant) / sample_ant);

    if (u > limit)
    {
        u = limit;
    }

    else if (u < -limit)
    {
        u = -limit;
    }

    ef_ant = ef;
    d_ef_ant = d_ef;
    sample_ant = sample;
    u_ant = u;
    Serial.println(u);
    return u;
}
