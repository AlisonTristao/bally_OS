#ifndef ESPNOW_H
#define ESPNOW_H

// ==================== SYSTEM & FRAMEWORK ====================
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>

// ==================== PROJECT HEADERS ====================
#include <Settings.h>
#include <Pinout.h>

// ==================== UTILITIES & LOGGING ====================
#include <Flags.h>
#include <Logger.h>

// ==================== EXTERNAL LIBRARIES ====================
#include <EspNowManager.h>

// ==================== ROBOT MODULES ====================
#include <RGBLed.h>

// espnow mac address to send
#define MAC_ADDR {0xDC, 0xDA, 0x0C, 0x30, 0xAA, 0x5C}

// Global ESP-NOW manager instance
EspNowManager espNowManager;

// Global RGB LED instance
RGBLed rgbLed;

void readMacAddress(){
    // logger the mac address
    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
    Serial.print("MAC Address: ");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    if (ret == ESP_OK) {
        Logger::insert_log("This MAC Address: " +
                        String(baseMac[0], HEX) + ":" +
                        String(baseMac[1], HEX) + ":" +
                        String(baseMac[2], HEX) + ":" +
                        String(baseMac[3], HEX) + ":" +
                        String(baseMac[4], HEX) + ":" +
                        String(baseMac[5], HEX), logType::INFO);
    } else {
        Logger::insert_log("Failed to read MAC address", logType::ERROR);
    }
}

// 📤 Callback de envio
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    (void)mac_addr;

    // Keep callback side-effect free for telemetry logger flow.
    if (status == ESP_NOW_SEND_SUCCESS) {
        rgbLed.setGreen();
    } else {
        rgbLed.setRed();
    }
}

// 📥 Callback de recebimento
void OnDataRecv(const uint8_t *mac_addr, const EspNowManager::message& incomingData) {
    (void)mac_addr;

    rgbLed.setBlue();

    char buffer[EspNowManager::MESSAGE_TEXT_SIZE + 1] = {0};
    memcpy(buffer, incomingData.msg, EspNowManager::MESSAGE_TEXT_SIZE);
    buffer[EspNowManager::MESSAGE_TEXT_SIZE] = '\0';

    // Keep command path compatible with existing shell flow.
    if (incomingData.type == EspNowManager::logType::INFO || incomingData.type == EspNowManager::logType::NONE) {
        Logger::insert_cmd(String(buffer));
    }
}

bool configure_wifi() {
    // Initialize RGB LED first
    if (!rgbLed.begin()) {
        Logger::insert_log("Failed to initialize RGB LED", logType::ERROR);
        return false;
    }
    
    rgbLed.setYellow(); // Indicar que está inicializando

    // turn on the wifi
    WiFi.mode(WIFI_STA);

    if (!WiFi.getMode() == WIFI_MODE_STA)
        return false;

    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::insert_log("WiFi configured", logType::INFO);
    #endif

    // starts ESP-NOW
    if (!espNowManager.begin(0, false))
        return false;

    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::insert_log("ESP-NOW initialized successfully", logType::INFO);
    #endif

    // register callbacks for send and receive
    espNowManager.setReceiveCallback(OnDataRecv);
    espNowManager.setSendCallback(OnDataSent);

    // add the peer 
    uint8_t peerAddress[] = MAC_ADDR;
    if (!espNowManager.addDevice(peerAddress, "peer", "Remote peer"))
        return false;

    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::insert_log("Peer added successfully", logType::INFO);
    #endif

    // register the mac address
    readMacAddress();
    
    // Signal that WiFi is ready
    rgbLed.setGreen();

    // all ok
    return true;
}

#endif