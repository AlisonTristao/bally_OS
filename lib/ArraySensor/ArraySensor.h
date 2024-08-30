#ifndef ARRAYSENSOR_H
#define ARRAYSENSOR_H

#include <Arduino.h>
#include <Preferences.h>

// Autor: Alison Tristão
// Email: AlisonTristao@hotmail.com

/************************/
/* Class Of ArraySensor */
/************************/

class ArraySensor {
private:
    /*
        @brief pins of the sensor and multiplexer
    */
    uint8_t sig, c0, c1, c2, c3, len, init_arr;

    /*
        @brief saves the last position of the line when line not found
    */
    double lastPosition;

    /*
        @brief read the sensor using a multiplexer

        @param index: index of the sensor

        @return the value of the sensor
    */
    uint16_t read(uint8_t index);

    /*
        @brief normalize the value of the sensor

        @param value: value of the sensor
        @param index: index of the sensor

        @return the normalized value of the sensor
    */
    int16_t normalize(uint16_t value, uint8_t index);

    /*
        @brief array of the min and max values of the sensors
    */
    uint16_t *min, *max;

    /*
        @brief object to save the min and max values of the sensors into EPROM
    */
    Preferences preferences;

public:
    /*
        @brief Constructor of ArraySensor 

        this library is used to read an array of sensors using a multiplexer

        @param len: number of sensors
        @param sig: signal pin
        @param c0: control pin 0
        @param c1: control pin 1
        @param c2: control pin 2
        @param c3: control pin 3
    */
    ArraySensor(uint8_t len, uint8_t sig, uint8_t c0, uint8_t c1, uint8_t c2, uint8_t c3);
    virtual ~ArraySensor();

    /*
        @brief check if the calibration is ok

        verify if the difference between the min and max values is greater than 100

        @return true if the calibration is ok
    */
    bool calibration_ok();

    /*
        @brief set the initial array to read the sensors using a multiplexer
    */
    void set_init_arr(uint8_t init_arr);

    /*
        @brief calibrate the sensors

        read the sensors and save the min and max values

        @param n_samples: number of samples to calibrate
        @param delay_ms: delay between samples

        @return true if the calibration is ok
    */
    bool calibrate(uint8_t n_samples, uint8_t delay_ms);

    /*
        @brief 
        
        @return the min and max values of the sensors
    */
    String calibrate_status();

    /*
        @brief read the sensors and return the position of the line

        calculate the position of the line using the normalized values of the sensors and a gaussian mean

        @return the position of the line with values between 0 and 1000 * len of sensors
    */
    double read_line();

    /*
        @brief save the min and max values of the sensors into EPROM to use in the next time
    */
    void saveCalibration();

    /*
        @brief load the min and max values of the sensors from EPROM

        @return true if the calibration was loaded
    */
    bool loadCalibration();

    /*
        @brief return the normalized values of the sensors

        @return the normalized values of the sensors
    */
    String debub();
};

#endif // ARRAYSENSOR_H
