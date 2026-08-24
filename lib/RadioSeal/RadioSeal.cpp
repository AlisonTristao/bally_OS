#include "RadioSeal.h"

#include <KeyStore.h>

namespace RadioSeal {
namespace {

KeyStore* g_store = nullptr;

}  // namespace

void configure(KeyStore& store) noexcept { g_store = &store; }

bool seal(void* /*context*/, const btp::Header& header,
         std::uint16_t payload_size, const std::uint8_t* plaintext,
         std::uint8_t* out) noexcept {
    if (g_store == nullptr || !g_store->loaded()) return false;

    const btp::AeadKey key{g_store->key_l(), KeyStore::kKeyLength};
    return btp::aead_seal_aes_gcm(key, header, payload_size, plaintext, out) ==
           btp::AeadError::Ok;
}

bool open(const btp::Header& header, std::uint16_t ciphertext_size,
         const std::uint8_t* ciphertext_and_tag,
         std::uint8_t* out_plaintext) noexcept {
    if (g_store == nullptr || !g_store->loaded()) return false;

    // Never trust a "consumed" frame that arrived unsealed or under a
    // cipher this channel does not use: fail closed rather than fall back
    // to reading it as plaintext. Channel C is AES-128-GCM only (the cipher
    // bally.key records for key L), so anything else -- including a
    // well-formed ChaCha20-Poly1305 frame -- is refused here, not silently
    // accepted under the wrong assumption.
    if ((header.flags & btp::kFlagEncrypted) == 0U) return false;
    if (btp::cipher_id(header.flags) != btp::CipherId::AesGcm) return false;

    const btp::AeadKey key{g_store->key_l(), KeyStore::kKeyLength};
    return btp::aead_open_aes_gcm(key, header, ciphertext_size,
                                  ciphertext_and_tag, out_plaintext) ==
           btp::AeadError::Ok;
}

bool seal_e(void* /*context*/, const btp::Header& header,
           std::uint16_t payload_size, const std::uint8_t* plaintext,
           std::uint8_t* out) noexcept {
    if (g_store == nullptr || !g_store->loaded()) return false;

    const btp::AeadKey key{g_store->key_e(), KeyStore::kKeyLength};
    return btp::aead_seal_aes_gcm(key, header, payload_size, plaintext, out) ==
           btp::AeadError::Ok;
}

bool open_e(const btp::Header& header, std::uint16_t ciphertext_size,
           const std::uint8_t* ciphertext_and_tag,
           std::uint8_t* out_plaintext) noexcept {
    if (g_store == nullptr || !g_store->loaded()) return false;

    if ((header.flags & btp::kFlagEncrypted) == 0U) return false;
    if (btp::cipher_id(header.flags) != btp::CipherId::AesGcm) return false;

    const btp::AeadKey key{g_store->key_e(), KeyStore::kKeyLength};
    return btp::aead_open_aes_gcm(key, header, ciphertext_size,
                                  ciphertext_and_tag, out_plaintext) ==
           btp::AeadError::Ok;
}

}  // namespace RadioSeal
