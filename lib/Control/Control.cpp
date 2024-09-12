#include "Control.h"
#include <Arduino.h>

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

    ErroPassado = Erro;
    return pid;
}

double Control::simplePD(double kp, double kd, double Erro, double sample)
{
    pd = kp * Erro + kd * (Erro - ErroPassado) / sample; // pid sem anti-windup

    if (ki * integral > limit)
        integral = limit / ki;
    else if (ki * integral < -limit)
        integral = -limit / ki;

    // garante que o P e o D nao sature
    if (pd >= limit)
        pd = limit;
    else if (pd <= -limit)
        pd = -limit;

    ErroPassado = Erro;
    return pd;
}

double Control::Gabes_Control(double kc, double ti, double td, double Erro, double sample)
{
    kp = kc;
    kd = kc*td;
    ki = kc/ti;

    d_erro = Erro - erro_ant;

    ef = ((td / 10) / (td / 10 + sample)) * ef + (sample / (td / 10 + sample)) * Erro;
    d_ef = (ef - ef_ant);


    //  u_ant   (    P    )   (          I            )   (                      D                                    ) 
    u = u_ant + (kc * d_erro) + ((kc / ti) * sample * Erro) + ((kc * td) * (((d_ef) / sample) - ((d_ef_ant) / sample_ant)));

    if (u > limit)
    {
        u = limit;
    }

    else if (u < -limit)
    {
        u = -limit;
    }
    
    erro_ant = Erro;
    ef_ant = ef;
    d_ef_ant = d_ef;
    sample_ant = sample;
    u_ant = u;
    
    return u;
}