#ifndef RGBLED_H
#define RGBLED_H

#include <Arduino.h>
#include <Pinout.h>

/**
 * @brief Wrapper para controlar LED RGB endereçável via SPI
 * Utiliza protocolo SPI com pinos do ESP32-S3
 */
class RGBLed {
private:
    bool initialized;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t brightness;

public:
    /**
     * @brief Inicializa o LED RGB e configura os pinos SPI
     */
    RGBLed() 
        : initialized(false),
          red(0), green(0), blue(0),
          brightness(31) {
    }

    /**
     * @brief Inicializa o interface SPI para o LED RGB
     * @return true se inicializado com sucesso
     */
    bool begin() {
        // Configurar pinos como OUTPUT
        pinMode(LED_RGB_MOSI, OUTPUT);
        pinMode(LED_RGB_SCK, OUTPUT);
        pinMode(LED_RGB_CS, OUTPUT);
        
        // Deixar CS alto inicialmente
        digitalWrite(LED_RGB_CS, HIGH);
        
        initialized = true;
        
        // Desligar LED inicialmente
        off();
        
        return true;
    }

    /**
     * @brief Define a cor do LED RGB
     * @param r Valor de vermelho (0-255)
     * @param g Valor de verde (0-255)
     * @param b Valor de azul (0-255)
     */
    void setColor(uint8_t r, uint8_t g, uint8_t b) {
        if (!initialized) return;
        
        red = r;
        green = g;
        blue = b;
        
        updateLed();
    }

    /**
     * @brief Define o brilho do LED
     * @param b Valor de brilho (0-31)
     */
    void setBrightness(uint8_t b) {
        if (!initialized) return;
        
        brightness = (b > 31) ? 31 : b;
        updateLed();
    }

    /**
     * @brief Desliga o LED completamente
     */
    void off() {
        if (!initialized) return;
        
        red = 0;
        green = 0;
        blue = 0;
        brightness = 0;
        
        updateLed();
    }

    /**
     * @brief Define LED para verde (indicador de sucesso)
     */
    void setGreen() {
        setColor(0, 255, 0);
    }

    /**
     * @brief Define LED para vermelho (indicador de erro)
     */
    void setRed() {
        setColor(255, 0, 0);
    }

    /**
     * @brief Define LED para azul (indicador de espera/idle)
     */
    void setBlue() {
        setColor(0, 0, 255);
    }

    /**
     * @brief Define LED para amarelo (indicador de aviso)
     */
    void setYellow() {
        setColor(255, 255, 0);
    }

    /**
     * @brief Pisca o LED (alternância rápida)
     * @param duration Duração em milissegundos
     */
    void blink(uint32_t duration = 100) {
        if (!initialized) return;
        
        off();
        delay(duration);
        updateLed();
        delay(duration);
    }

private:
    /**
     * @brief Transmite um byte via SPI bit-bang
     */
    void spiWriteByte(uint8_t byte) {
        for (int i = 7; i >= 0; i--) {
            // Configurar MOSI
            digitalWrite(LED_RGB_MOSI, (byte >> i) & 1);
            
            // Pulse de clock
            digitalWrite(LED_RGB_SCK, HIGH);
            delayMicroseconds(1);
            digitalWrite(LED_RGB_SCK, LOW);
            delayMicroseconds(1);
        }
    }

    /**
     * @brief Atualiza o LED com as cores e brilho atuais via SPI
     */
    void updateLed() {
        if (!initialized) return;
        
        // Frame de inicio: 3 bytes 0x00
        uint8_t startFrame[3] = {0x00, 0x00, 0x00};
        
        // Frame de LED: 0xE0 + 5 bits de brilho + RGB
        uint8_t ledFrame[4];
        ledFrame[0] = 0xE0 | (brightness & 0x1F);
        ledFrame[1] = blue;
        ledFrame[2] = green;
        ledFrame[3] = red;
        
        // Frame de fim: 4 bytes 0xFF
        uint8_t endFrame[4] = {0xFF, 0xFF, 0xFF, 0xFF};
        
        // Selecionar dispositivo
        digitalWrite(LED_RGB_CS, LOW);
        delayMicroseconds(100);
        
        // Enviar frames
        for (int i = 0; i < 3; i++) spiWriteByte(startFrame[i]);
        for (int i = 0; i < 4; i++) spiWriteByte(ledFrame[i]);
        for (int i = 0; i < 4; i++) spiWriteByte(endFrame[i]);
        
        // Desselecionar dispositivo
        delayMicroseconds(100);
        digitalWrite(LED_RGB_CS, HIGH);
    }
};

#endif // RGBLED_H
