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
        /*
            @brief pins of the encoder
        */
        gpio_num_t pinA, pinB;

        /*
            @brief unit of the PCNT
        */
        pcnt_unit_t unit;

        /*
            @brief configuration of the PCNT
        */
        pcnt_config_t enc_config;

        /*
            @brief saves the counter of the encoder when the pcnt overflows
        */
        int64_t counter = 0;

        /*
            @brief get the raw counter of the encoder

            @return the raw counter of the encoder
        */
        int64_t getCountRaw(); 

        // ISR handler of encoder
        static bool isr_installed;  

        /*
            @brief array of used PCNTs
        */
        static Encoder *usedPCNTs[PCNT_UNIT_MAX];
    public:
        /*
            @brief Constructor of Encoder

            this library is used to read an encoder using the PCNT

            @param pinA: pin A of the encoder
            @param pinB: pin B of the encoder
        */
        Encoder(uint8_t pinA, uint8_t pinB);
        virtual ~Encoder();

        /*
            @brief init the encoder

            @param filter: filter to count the pulses
            @return true if the encoder was initialized
        */
        bool init(uint16_t filter = 0);
        
        /*
            @brief ISR of the encoder

            this function is called when the pulse counter overflows

            saves the actual value and clears the pulse counter
        */
        void overflow();

        /*
            @brief get the counter of the encoder

            @return the counter of the encoder
        */
        int64_t getCount();

        /*
            @brief clear the counter of the encoder
        */
        void clearPCNT();

        /*
            @brief pause the encoder
        */
        void pausePCNT();

        /*
            @brief resume the encoder
        */
        void resumePCNT();

        // set filter low pass Xrd order
        // ainda nao vou implementar

        /*
            @brief core to run the ISR of the encoder

            @param core: core to run the ISR
        */
        static uint8_t core_to_run_ISR;
};

#endif // ENCODER_H