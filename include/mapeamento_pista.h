#ifndef MAPEAMENTO_PISTA_H
#define MAPEAMENTO_PISTA_H

#include <Arduino.h>
#define sensor_fim_de_pista 17  //17 ou 8, tem que conferir

//variaveis
int theta[];                    //angulo da curva
int W = 0;                      //velocidade angular
int erro = 0;
int Velocidade = 0;             // velocidade do carrinho;
int posicaoX[];                 //posição em X
int posicaoY[];                 //posição em Y

int sign(int x) {               //função para retornar o sinal
    return (x > 0) - (x < 0);
}

void mapeamento()
{
    int VX = 0;                 //velocidade em X
    int VY = 0;                 //velocidade em Y
    int i = 1;                  //numero da amostra
    theta[0] = 0;
    posicaoX[0] = 0;
    posicaoY[0] = 0;
    while(analogRead (sensor_fim_de_pista) == 0)
    {
        //incluir codigo para seguir a linha*************************************************************************************************************************
        
        //calculo das cordenadas X e Y da pista
        theta[i] = theta[i-1] + W;
        VX = sign(cos(theta[i]))*sqrt(pow(Velocidade,2)/(1+(pow(tan(theta[i]),2))));
        VY = sign(sin(theta[i]))*sqrt((pow(Velocidade,2)*pow(tan(theta[i]),2))/(1+(pow(tan(theta[i]),2))));
        posicaoX[i] = posicaoX[i-1] + VX;
        posicaoY[i] = posicaoY[i-1] + VY;

        //adiconar a componente do erro
        posicaoX[i] = posicaoX[i] + erro*sin(theta[i]);
        posicaoY[i] = posicaoY[i] - erro*cos(theta[i]);
        i++;
        
        //incluir codigo para enviar os valores***********************************************************************************************************************
    }
}

#endif