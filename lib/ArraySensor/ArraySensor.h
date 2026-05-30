#ifndef ARRAYSENSOR_H
#define ARRAYSENSOR_H

#include <Preferences.h>
#include <cstdint>

// Autor: Alison Tristão
// Email: AlisonTristao@hotmail.com

/************************/
/* Class Of ArraySensor */
/************************/

template <uint8_t LEN>
class ArraySensor {
private:
    /**
     * @brief pins of the sensor and multiplexer
     */
    //uint8_t sig, c0, c1, c2, c3, len, init_arr;
    // using no mux
    uint8_t arr[LEN];

    /**
     * @brief saves the last position of the line when line not found
     */
    float lastPosition;

    /**
     * @brief read the sensor using a multiplexer
     * @param index: index of the sensor
     * @return the value of the sensor
     */
    uint16_t read(uint8_t index);

    /**
     * @brief normalize the value of the sensor
     * @param value: value of the sensor
     * @param index: index of the sensor
     * @return the normalized value of the sensor
     */
    int16_t normalize(uint16_t value, uint8_t index);

    /**
     * @brief array of the min and max values of the sensors
     */
    uint16_t min[LEN], max[LEN];

    /**
     * @brief object to save the min and max values of the sensors into EPROM
     */
    Preferences preferences;

public:
    /**
     * @brief Constructor of ArraySensor - this library is used to read an array of sensors using a multiplexer
     * @param len: number of sensors
     * @param sig: signal pin
     * @param c0: control pin 0
     * @param c1: control pin 1
     * @param c2: control pin 2
     * @param c3: control pin 3
     */
    ArraySensor(const uint8_t* arr);
    virtual ~ArraySensor() {};

    /**
     * @brief check if the calibration is ok - verify if the difference between the min and max values is greater than 100
     * @return true if the calibration is ok
     */
    bool calibration_ok();

    /**
     * @brief calibrate the sensors - read the sensors and save the min and max values
     * @param n_samples: number of samples to calibrate
     * @param delay_ms: delay between samples
     * @return true if the calibration is ok
     */
    bool calibrate(uint8_t n_samples, uint8_t delay_ms);

    /**
     * @brief get the status of the calibration
     * @return the min and max values of the sensors
     */
    std::string calibrate_status();

    /**
     * @brief calculate the position of the line using the normalized values of the sensors and a gaussian mean
     * @return the position of the line with values between 0 and 1000 * len of sensors
     */
    float get_line_position();

    /**
     * @brief save the min and max values of the sensors into EPROM to use in the next time
     */
    void saveCalibration();

    /**
     * @brief return true if the calibration was loaded
     * @return true if the calibration was loaded
     */
    bool loadCalibration();

    /**
     * @brief return the normalized values of the sensors
     * @return the normalized values of the sensors
     */
    std::string debug();

    /**
     * @brief return the values of the sensors
     * @return the values of the sensors
     */
    std::string raw();
};

template <uint8_t LEN>
ArraySensor<LEN>::ArraySensor(const uint8_t* arr) {
    for(uint8_t i = 0; i < LEN; i++) {
        this->arr[i] = arr[i];
        min[i] = 4095;
        max[i] = 0;
    }
}

template <uint8_t LEN>
uint16_t ArraySensor<LEN>::read(uint8_t index){
    // if the line is black, invert the value
    return 4095 - analogRead(arr[index]);
}

template <uint8_t LEN>
int16_t ArraySensor<LEN>::normalize(uint16_t value, uint8_t index){
    // normalize the value
    int32_t range = max[index] - min[index];
    if (range == 0) return 0; // avoid division by zero
    int32_t norm = ((value - min[index]) * 1000) / range;

    // sature the value
    if(norm < 0)        return 0;
    if(norm > 1000)     return 1000;
    return norm;
}

template <uint8_t LEN>
bool ArraySensor<LEN>::calibration_ok(){
    // check if the calibration is ok
    for(uint8_t i = 0; i < LEN; i++)
        if((max[i] - min[i]) <= 100) return false;
    return true;
}

template <uint8_t LEN>
bool ArraySensor<LEN>::calibrate(uint8_t n_samples, uint8_t delay_ms){
    // calibrate the sensors
    uint16_t value = 0;
    for(uint8_t i = 0; i < n_samples; i++){
        for(uint8_t j = 0; j < LEN; j++){
            value = read(j);
            // line white or black
            if (value < min[j]) min[j] = value;
            if (value > max[j]) max[j] = value;
        }
        delay(delay_ms);
    }

    // check if the calibration is ok
    return calibration_ok();
}

template <uint8_t LEN>
std::string ArraySensor<LEN>::calibrate_status(){
    // return the calibration status
    std::string status;
    for(uint8_t i = 0; i < LEN; i++){
        status +=   std::to_string(min[i])  + 
                    "-"             + 
                    std::to_string(max[i])  + 
                    "\n";
    }

    return status;
}

template <uint8_t LEN>
float ArraySensor<LEN>::get_line_position(){
    uint32_t value = 0, measure = 0;
    bool line = false;
    uint16_t val = 0;
    // read the sensor and normalize the value
    for (uint8_t i = 0; i < LEN; i++){
        val = normalize(read(i), i);
        value += val * (i+1) * 1000;
        measure += val;
        // check if the line is detected
        if(val > (LEN*100)) line = true;
    }

    // atualize last position
    if(line)    lastPosition = (float)value/(float)measure;
    // saturate last position
    else        lastPosition = lastPosition < (LEN*1000)/2 ? 1000 : LEN*1000;
    
    // return the last position
    return lastPosition;
}

template <uint8_t LEN>
std::string ArraySensor<LEN>::debug(){
    std::string status;
    for(uint8_t i = 0; i < LEN; i++)
        status += std::to_string(read(i)) + "\t";
    return status;
}

template <uint8_t LEN>
std::string ArraySensor<LEN>::raw(){
    std::string status;
    for(uint8_t i = 0; i < LEN; i++)
        status += std::to_string(read(i)) + "\t";
    return status;
}

template <uint8_t LEN>
void ArraySensor<LEN>::saveCalibration() {
    preferences.begin("calibration", false);
    char key[10]; // Buffer to store the keys

    for (uint8_t i = 0; i < LEN; i++) {
        snprintf(key, sizeof(key), "min%d", i);
        preferences.putUInt(key, min[i]);

        snprintf(key, sizeof(key), "max%d", i);
        preferences.putUInt(key, max[i]);
    }

    preferences.end();
}

template <uint8_t LEN>
bool ArraySensor<LEN>::loadCalibration() {
    
    if(!preferences.begin("calibration", true)) return false;
    char key[10]; // Buffer to store the keys

    for (uint8_t i = 0; i < LEN; i++) {
        snprintf(key, sizeof(key), "min%d", i);
        min[i] = preferences.getUInt(key, 0xFFFF);

        snprintf(key, sizeof(key), "max%d", i);
        max[i] = preferences.getUInt(key, 0x0000);
    }

    preferences.end();

    // check if the calibration is valid
    for (uint8_t i = 0; i < LEN; i++) {
        if (min[i] == 0xFFFF || max[i] == 0x0000) {
            return false; // calibration is not valid
        }
    }

    return true;
}

#endif // ARRAYSENSOR_H