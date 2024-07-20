#ifndef ENCODER_H
#define ENCODER_H
#include <Arduino.h>

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

// WARNING: Not all esp32 chips support the pcnt
#include <driver/pcnt.h>
#include <soc/pcnt_struct.h>
#include "esp_ipc.h"

/**********************/
/*  Class Of Encoder  */
/**********************/  

class Encoder{
    private:
        // pins of encoder
        gpio_num_t pinA, pinB;

        // PCNT 
        pcnt_unit_t unit;
        pcnt_config_t enc_config;

        // pulse counter
        int64_t counter = 0;

        // get counter 
        int64_t getCountRaw(); 

        // ISR handler of encoder
        static bool isr_installed;  

        // array of encoders
        static Encoder *usedPCNTs[PCNT_UNIT_MAX];
    public:
        Encoder(uint8_t pinA, uint8_t pinB);
        virtual ~Encoder();
        bool init(uint16_t filter = 0);
        
        // overflow of counter
        void overflow();

        // get the data of the encoder
        int64_t getCount();

        // manipulate the counter
        void clearPCNT();
        void pausePCNT();
        void resumePCNT();

        // set filter low pass Xrd order
        // ainda nao vou implementar

        // core of ISR
        static uint8_t core_to_run_ISR;
};

#endif // ENCODER_H