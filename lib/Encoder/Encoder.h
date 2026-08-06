#ifndef ENCODER_H
#define ENCODER_H

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

// WARNING: Not all esp32 chips support the pcnt
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_ipc.h"

class TinyShell;
class Logger;

/**********************/
/* Class Of Encoder  */
/**********************/  

class Encoder{
    private:
        /**
         * @brief pins of the encoder
         */
        gpio_num_t pinA, pinB;

        /**
         * @brief unit of the PCNT
         */
        pcnt_unit_handle_t unit;
        pcnt_channel_handle_t channel_a;
        pcnt_channel_handle_t channel_b;

        /**
         * @brief saves the counter of the encoder when the pcnt overflows
         */
        int64_t counter = 0;

        /**
         * @brief saves the last counter of the encoder to calculate the difference
         */
        int64_t last_counter = 0;

        /**
         * @brief get the raw counter of the encoder
         * @return the raw counter of the encoder
         */
        int64_t getCountRaw();

        // ISR handler of encoder
        static bool isr_installed;  

        /**
         * @brief array of used PCNTs
         */
        static Encoder *usedPCNTs[4];
    public:
        /**
         * @brief Constructor of Encoder - this library is used to read an encoder using the PCNT
         * @param pinA: pin A of the encoder
         * @param pinB: pin B of the encoder
         */
        Encoder(uint8_t pinA, uint8_t pinB);
        virtual ~Encoder();

        /**
         * @brief init the encoder
         * @param filter: filter to count the pulses
         * @return true if the encoder was initialized
         */
        bool init(uint16_t filter = 0);

        /**
         * @brief ISR of the encoder - this function is called when the pulse counter overflows and saves the actual value and clears the pulse counter
         */
        void overflow(int watch_val);

        /**
         * @brief get the counter of the encoder
         * @return the counter of the encoder
         */
        int64_t getCount();

        /**
         * @brief get the diference of the counter of the encoder since the last time this function was called
         * @return the diference of the counter of the encoder since the last time this function
         */
        int64_t getCountDiff();

        /**
         * @brief clear the counter of the encoder
         */
        void clearPCNT();

        /**
         * @brief pause the encoder
         */
        void pausePCNT();

        /**
         * @brief resume the encoder
         */
        void resumePCNT();

        // set filter low pass Xrd order
        // ainda nao vou implementar

        /**
         * @brief core to run the ISR of the encoder
         * @param core: core to run the ISR
         */
        static uint8_t core_to_run_ISR;

        /**
         * @brief Register the "sensor" module's "encoders" shell command,
         * reporting both wheel encoders' raw counts in one line. Static (and
         * takes both instances) since the differential-drive left/right
         * pairing is the caller's wiring, not something a single Encoder
         * instance knows about itself.
         */
        static void register_shell_commands(TinyShell& shell, Logger& logger,
                                            Encoder& left, Encoder& right);
};

#endif // ENCODER_H