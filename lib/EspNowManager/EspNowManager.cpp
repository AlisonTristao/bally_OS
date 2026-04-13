#include "EspNowManager.h"

#include <WiFi.h>
#include <cstring>

// Instância ativa para callbacks estáticos.
EspNowManager* EspNowManager::activeInstance_ = nullptr;

// Construtor.
EspNowManager::EspNowManager()
    : initialized_(false),
      channel_(0),
      encrypt_(false),
      receiveCallback_(nullptr),
      sendCallback_(nullptr),
      sendWaitPending_(false),
      sendWaitCompleted_(false),
      sendWaitStatus_(ESP_NOW_SEND_FAIL),
      sendWaitMac_{0, 0, 0, 0, 0, 0},
      sendWaitHasMac_(false) {
}

// Inicializa Wi-Fi station mode e ESP-NOW.
bool EspNowManager::begin(uint8_t channel, bool encrypt) {
    channel_ = channel;
    encrypt_ = encrypt;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        initialized_ = false;
        return false;
    }

    initialized_ = true;
    activeInstance_ = this;

    esp_now_register_recv_cb(handleReceiveStatic);
    esp_now_register_send_cb(handleSendStatic);

    return true;
}

// Desativa ESP-NOW.
void EspNowManager::end() {
    if (initialized_) {
        esp_now_deinit();
    }

    initialized_ = false;
    if (activeInstance_ == this) {
        activeInstance_ = nullptr;
    }
}

// Registra um peer no ESP-NOW.
bool EspNowManager::addPeer(const uint8_t mac[6]) {
    if (!initialized_ || mac == nullptr) {
        return false;
    }

    if (esp_now_is_peer_exist(mac)) {
        return true;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = channel_;
    peerInfo.encrypt = encrypt_;

    return esp_now_add_peer(&peerInfo) == ESP_OK;
}

// Remove um peer do ESP-NOW.
bool EspNowManager::removePeer(const uint8_t mac[6]) {
    if (!initialized_ || mac == nullptr) {
        return false;
    }

    if (!esp_now_is_peer_exist(mac)) {
        return true;
    }

    return esp_now_del_peer(mac) == ESP_OK;
}

// Envia mensagem para um MAC (fire-and-forget).
bool EspNowManager::sendToMac(const uint8_t mac[6], const message& outgoing) const {
    if (!initialized_ || mac == nullptr) {
        return false;
    }

    return esp_now_send(mac, reinterpret_cast<const uint8_t*>(&outgoing), sizeof(outgoing)) == ESP_OK;
}

// Envia mensagem e aguarda confirmação.
bool EspNowManager::sendToMacWithStatus(const uint8_t mac[6], const message& outgoing, bool& outDelivered, uint32_t timeoutMs) const {
    outDelivered = false;
    if (!initialized_ || mac == nullptr) {
        return false;
    }

    sendWaitPending_ = true;
    sendWaitCompleted_ = false;
    sendWaitStatus_ = ESP_NOW_SEND_FAIL;
    memcpy(sendWaitMac_, mac, sizeof(sendWaitMac_));
    sendWaitHasMac_ = true;

    if (esp_now_send(mac, reinterpret_cast<const uint8_t*>(&outgoing), sizeof(outgoing)) != ESP_OK) {
        sendWaitPending_ = false;
        sendWaitHasMac_ = false;
        return false;
    }

    const uint32_t start = millis();
    while (sendWaitPending_) {
        if ((millis() - start) >= timeoutMs) {
            sendWaitPending_ = false;
            sendWaitHasMac_ = false;
            return false;
        }
        delay(1);
    }

    sendWaitHasMac_ = false;
    if (!sendWaitCompleted_) {
        return false;
    }

    outDelivered = (sendWaitStatus_ == ESP_NOW_SEND_SUCCESS);
    return true;
}

// Define callback para recebimento.
void EspNowManager::setReceiveCallback(ReceiveCallback callback) {
    receiveCallback_ = callback;
}

// Define callback para status de envio.
void EspNowManager::setSendCallback(SendCallback callback) {
    sendCallback_ = callback;
}

// Adapter para callback estático de recebimento.
void EspNowManager::handleReceiveStatic(const uint8_t* mac, const uint8_t* incomingData, int len) {
    if (activeInstance_ == nullptr || activeInstance_->receiveCallback_ == nullptr || incomingData == nullptr || len <= 0) {
        return;
    }

    message incoming = {};

    if (len >= static_cast<int>(sizeof(message))) {
        memcpy(&incoming, incomingData, sizeof(message));
        incoming.msg[sizeof(incoming.msg) - 1] = '\0';
    } else {
        const size_t copySize = (static_cast<size_t>(len) < (sizeof(incoming.msg) - 1))
            ? static_cast<size_t>(len)
            : (sizeof(incoming.msg) - 1);
        memcpy(incoming.msg, incomingData, copySize);
        incoming.msg[copySize] = '\0';
        incoming.timer = millis();
        incoming.type = logType::NONE;
    }

    activeInstance_->receiveCallback_(mac, incoming);
}

// Adapter para callback estático de envio.
void EspNowManager::handleSendStatic(const uint8_t* mac, esp_now_send_status_t status) {
    if (activeInstance_ == nullptr) {
        return;
    }

    if (activeInstance_->sendWaitPending_ && activeInstance_->sendWaitHasMac_ && mac != nullptr) {
        if (memcmp(activeInstance_->sendWaitMac_, mac, 6) == 0) {
            activeInstance_->sendWaitStatus_ = status;
            activeInstance_->sendWaitCompleted_ = true;
            activeInstance_->sendWaitPending_ = false;
        }
    }

    if (activeInstance_->sendCallback_ == nullptr) {
        return;
    }

    activeInstance_->sendCallback_(mac, status);
}
