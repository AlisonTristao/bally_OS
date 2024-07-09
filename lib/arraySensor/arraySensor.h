#ifndef ARRAYSENSOR_H
#define ARRAYSENSOR_H
#include <Arduino.h>

// line colors
#define BLACK 1
#define WHITE 0

class arraySensor{
private:
    // mux pins
    uint8_t sig, c0, c1, c2, c3;
    uint8_t len, init_arr;
    bool lineColor;
    double lastPosition;

    // functions
    uint16_t read(uint8_t index);
    int16_t normalize(uint16_t value, uint8_t index);

public:
    arraySensor(uint8_t len, uint8_t sig, uint8_t c0, uint8_t c1, uint8_t c2, uint8_t c3, bool lineColor);
    virtual ~arraySensor();
    bool calibration_ok();
    
    // normalize array
    uint16_t *min, *max;

    // init variables
    void init();

    // set the initial array of the mux
    void set_init_arr(uint8_t init_arr);

    // calibrate the sensor
    bool calibrate(uint8_t n_samples, uint8_t delay_ms, uint8_t led);
    
    // get the line position
    String calibrate_status();

    // calculate the line position
    double read_line();

    // debug
    String debub();
};
#endif