#ifndef ARRAYSENSOR_H
#define ARRAYSENSOR_H

#include <Arduino.h>
#include <Preferences.h>

// Autor: Alison Tristão
// Email: AlisonTristao@hotmail.com

class ArraySensor {
private:
    uint8_t sig, c0, c1, c2, c3, len, init_arr;
    double lastPosition;

    // Leitura e normalização do sensor
    uint16_t read(uint8_t index);
    int16_t normalize(uint16_t value, uint8_t index);

    // Normalização de arrays
    uint16_t *min, *max;

    // Instância de Preferences para salvar dados persistentes
    Preferences preferences;

public:
    ArraySensor(uint8_t len, uint8_t sig, uint8_t c0, uint8_t c1, uint8_t c2, uint8_t c3);
    virtual ~ArraySensor();
    bool calibration_ok();

    // Configura o array inicial do mux
    void set_init_arr(uint8_t init_arr);

    // Calibra o sensor
    bool calibrate(uint8_t n_samples, uint8_t delay_ms);

    // Obtém o status da calibração
    String calibrate_status();

    // Lê a posição da linha
    double read_line();

    // Salva os valores de calibração
    void saveCalibration();

    // Carrega os valores de calibração
    bool loadCalibration();

    // Debug
    String debub();
};

#endif // ARRAYSENSOR_H
