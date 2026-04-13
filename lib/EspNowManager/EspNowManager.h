#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include "logger.h"

/**
 * @brief EspNowManager simplificado: apenas envio point-to-point.
 *
 * Responsabilidades:
 * - Inicializar/desativar stack ESP-NOW
 * - Adicionar peer e enviar mensagens diretas (por MAC)
 * - Callbacks para eventos de recebimento e status
 */
class EspNowManager final {
public:
    /** Tamanho de payload. */
    static constexpr size_t MESSAGE_TEXT_SIZE = 231;

    /**
     * @brief Estrutura de mensagem para envio/recebimento.
     */
    struct message {
        uint32_t timer;
        char msg[MESSAGE_TEXT_SIZE];
        logType type;
    };

    /** Callback para mensagens recebidas. */
    using ReceiveCallback = void (*)(const uint8_t* mac, const message& incoming);
    /** Callback para status de envio. */
    using SendCallback = void (*)(const uint8_t* mac, esp_now_send_status_t status);

    /**
     * @brief Cria uma instância do gerenciador.
     */
    EspNowManager();

    /**
     * @brief Inicializa Wi-Fi station mode e ESP-NOW.
     * @param channel Canal ESP-NOW.
     * @param encrypt Ativa encriptação entre peers.
     * @return true se sucesso.
     */
    bool begin(uint8_t channel = 0, bool encrypt = false);

    /**
     * @brief Desativa ESP-NOW.
     */
    void end();

    /**
     * @brief Registra um peer no ESP-NOW.
     * @param mac Endereço MAC.
     * @return true se sucesso.
     */
    bool addPeer(const uint8_t mac[6]);

    /**
     * @brief Remove um peer do ESP-NOW.
     * @param mac Endereço MAC.
     * @return true se sucesso.
     */
    bool removePeer(const uint8_t mac[6]);

    /**
     * @brief Envia mensagem para um MAC (fire-and-forget).
     * @param mac Endereço MAC.
     * @param outgoing Payload.
     * @return true se enviado.
     */
    bool sendToMac(const uint8_t mac[6], const message& outgoing) const;

    /**
     * @brief Envia mensagem e aguarda confirmação de delivery.
     * @param mac Endereço MAC.
     * @param outgoing Payload.
     * @param outDelivered true se acknowledgeado com sucesso.
     * @param timeoutMs Tempo máximo de espera em ms.
     * @return true se callback recebido.
     */
    bool sendToMacWithStatus(const uint8_t mac[6], const message& outgoing, bool& outDelivered, uint32_t timeoutMs = 600) const;

    /**
     * @brief Define callback para mensagens recebidas.
     */
    void setReceiveCallback(ReceiveCallback callback);

    /**
     * @brief Define callback para status de envio.
     */
    void setSendCallback(SendCallback callback);

private:
    /** Instância ativa para callbacks estáticos. */
    static EspNowManager* activeInstance_;

    bool initialized_;
    uint8_t channel_;
    bool encrypt_;
    ReceiveCallback receiveCallback_;
    SendCallback sendCallback_;

    mutable volatile bool sendWaitPending_;
    mutable volatile bool sendWaitCompleted_;
    mutable esp_now_send_status_t sendWaitStatus_;
    mutable uint8_t sendWaitMac_[6];
    mutable bool sendWaitHasMac_;

    /** Adapter para callback estático de recebimento. */
    static void handleReceiveStatic(const uint8_t* mac, const uint8_t* incomingData, int len);

    /** Adapter para callback estático de envio. */
    static void handleSendStatic(const uint8_t* mac, esp_now_send_status_t status);
};
