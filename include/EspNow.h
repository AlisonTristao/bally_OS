#ifndef ESPNOW_H
#define ESPNOW_H

// native libraries
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <WiFi.h>

// header
#include <Settings.h>

// static libraries
#include <Flags.h>
#include <Logger.h>

// espnow mac address to send
#define MAC_ADDR {0xDC, 0xDA, 0x0C, 0x30, 0xAA, 0x5C}

void readMacAddress(){
    // logger the mac address
    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
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
    // led sinalization
    if (status == ESP_NOW_SEND_SUCCESS)
        Signals_IN::setLed0on();
    else
        Signals_IN::setLed2on();
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    // led sinalization
    Signals_IN::setLed3on();

    // messege received 
    char *buffer = new char[len + 1];  // +1 para o terminador '\0'
    memcpy(buffer, data, len);
    buffer[len] = '\0';
    
    // save the string to logger (convert the buffer to string)
    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::insert_cmd(String(buffer));
    #endif

    delete[] buffer;
}

bool configure_wifi() {
    // turn on the wifi
    WiFi.mode(WIFI_STA);

    if (!WiFi.getMode() == WIFI_MODE_STA)
        return false;

    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::insert_log("WiFi configured", logType::INFO);
    #endif

    // starts ESP-NOW
    if (esp_now_init() != ESP_OK)
        return false;

    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::insert_log("ESP-NOW initialized successfully", logType::INFO);
    #endif

    // callback to send and received commands
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // add the peer 
    esp_now_peer_info_t peerInfo;
    uint8_t peerAddress[] = MAC_ADDR;
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) 
        return false;

    #if defined(LOG_ALL) || defined(LOG_INFO)
        Logger::insert_log("Peer added successfully", logType::INFO);
    #endif

    // register the mac address
    readMacAddress();

    // all ok
    return true;
}

#endif