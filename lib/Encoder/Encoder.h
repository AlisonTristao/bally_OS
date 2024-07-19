#ifndef ENCODER_H
#define ENCODER_H
#include <Arduino.h>

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

// WARNING: Not all esp32 chips support the pcnt
#include <driver/pcnt.h>
#include "esp_ipc.h"
#include <soc/pcnt_struct.h>

// configure the counter limit
#ifdef CONFIG_IDF_TARGET_ESP32S2
    #define COUNTER_H_LIM   cnt_thr_h_lim_lat_un
    #define COUNTER_L_LIM   cnt_thr_l_lim_lat_un
    #define thres0_lat      cnt_thr_thres0_lat_un
    #define thres1_lat      cnt_thr_thres1_lat_un

#elif CONFIG_IDF_TARGET_ESP32S3
    #define COUNTER_H_LIM   cnt_thr_h_lim_lat_un
    #define COUNTER_L_LIM   cnt_thr_l_lim_lat_un
    #define thres0_lat      cnt_thr_thres0_lat_un
    #define thres1_lat      cnt_thr_thres1_lat_un

#else
    #define COUNTER_H_LIM   h_lim_lat
    #define COUNTER_L_LIM   l_lim_lat

#endif

#define _INT16_MAX +32766
#define _INT16_MIN -32766

/**********************/
/*  Class Of Encoder  */
/**********************/  

// function pointer to interrupt service routine
typedef void (*isr_t)(void *);

class Encoder{
    private:
        // pins of encoder
        gpio_num_t pinA, pinB;

        // timer micros seconds of samples
        uint64_t timer;

        // get counter 
        int64_t getCountRaw(); 
    public:
        Encoder(uint8_t pinA, uint8_t pinB);
        virtual ~Encoder();

        // init configuration of encoder
        bool init(uint16_t filter = 0, uint8_t core = 1);
        
        // get the ticks of the encoder
        int64_t getCount();

        // get the speed of the encoder
        double getSpeed();

        // clear the counter
        void clearPCNT();
        void clearCount();

        // set filter low pass Xrd order
        // ainda nao vou implementar

        // debug
        String debub();

        // PCNT 
        pcnt_unit_t unit;
        pcnt_config_t enc_config;

        // pulse counter
        int64_t counter;

        // array of encoders
        static Encoder *usedPCNTs[PCNT_UNIT_MAX];
};

#endif // ENCODER_H