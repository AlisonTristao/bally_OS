#include "ArraySensor.h"

ArraySensor::ArraySensor(uint8_t len, uint8_t sig, uint8_t c0, uint8_t c1, uint8_t c2, uint8_t c3){
    this->len = len;
    this->sig = sig;
    this->c0 = c0;
    this->c1 = c1;
    this->c2 = c2;
    this->c3 = c3;
    // init arrays
    min = new uint16_t[len];
    max = new uint16_t[len];
    for(uint8_t i = 0; i < len; i++){
        min[i] = 4095;
        max[i] = 0;
    }
}

ArraySensor::~ArraySensor(){
    // delete pointers
    delete[] min;
    delete[] max;
}

void ArraySensor::init(uint8_t init_arr){
    this->init_arr = init_arr;
}

uint16_t ArraySensor::read(uint8_t index){
    // set the index of the sensor
    index = index + init_arr;

    // set the mux to read the sensor
    digitalWrite(c0, bitRead(index, 0));
    digitalWrite(c1, bitRead(index, 1));
    digitalWrite(c2, bitRead(index, 2));
    digitalWrite(c3, bitRead(index, 3));

    // if the line is black, invert the value
    return 4095 - analogRead(sig);
}

int16_t ArraySensor::normalize(uint16_t value, uint8_t index){
    // normalize the value
    int16_t norm = map(value, min[index], max[index], 0, 1000);

    // sature the value
    if(norm < 0)        return 0;
    if(norm > 1000)     return 1000;
    return norm;
}

bool ArraySensor::calibration_ok(){
    // check if the calibration is ok
    for(uint8_t i = 0; i < len; i++)
        if((max[i] - min[i]) <= 100) return false;
    return true;
}

bool ArraySensor::calibrate(uint8_t n_samples, uint8_t delay_ms){
    // calibrate the sensors
    uint16_t value = 0;
    for(uint8_t i = 0; i < n_samples; i++){
        for(uint8_t j = 0; j < len; j++){
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

String ArraySensor::calibrate_status(){
    // return the calibration status
    String status;
    for(uint8_t i = 0; i < len; i++){
        status +=   String(min[i])  + 
                    "-"             + 
                    String(max[i])  + 
                    "\n";
    }

    return status;
}

double ArraySensor::read_line(){
    double value = 0, measure = 0;
    bool line = false;
    uint16_t val = 0;
    // read the sensor and normalize the value
    for (uint8_t i = 0; i < len; i++){
        val = normalize(read(i), i);
        value += val * (i+1) * 1000;
        measure += val;
        // check if the line is detected
        if(val > (len*100)) line = true;
    }

    // atualize last position
    if(line)    lastPosition = value/measure;
    // saturate last position
    else        lastPosition = lastPosition < (len*1000)/2 ? 1000 : len*1000;
    
    // return the last position
    return lastPosition;
}

String ArraySensor::debug(){
    String status;
    for(uint8_t i = 0; i < len; i++)
        status += String(normalize((i), i)) + "\t";
    return status;
}

void ArraySensor::saveCalibration() {
    preferences.begin("calibration", false);
    char key[10]; // Buffer to store the keys

    for (uint8_t i = 0; i < len; i++) {
        snprintf(key, sizeof(key), "min%d", i);
        preferences.putUInt(key, min[i]);

        snprintf(key, sizeof(key), "max%d", i);
        preferences.putUInt(key, max[i]);
    }

    preferences.end();
}

bool ArraySensor::loadCalibration() {
    
    if(!preferences.begin("calibration", true)) return false;
    char key[10]; // Buffer to store the keys

    for (uint8_t i = 0; i < len; i++) {
        snprintf(key, sizeof(key), "min%d", i);
        min[i] = preferences.getUInt(key, 0xFFFF);

        snprintf(key, sizeof(key), "max%d", i);
        max[i] = preferences.getUInt(key, 0x0000);
    }

    preferences.end();

    // check if the calibration is valid
    for (uint8_t i = 0; i < len; i++) {
        if (min[i] == 0xFFFF || max[i] == 0x0000) {
            return false; // calibration is not valid
        }
    }

    return true;
}