#ifndef RADIO_SEAL_H
#define RADIO_SEAL_H

#include <btp/aead.hpp>
#include <btp/codec.hpp>

#include <cstddef>
#include <cstdint>

class KeyStore;

/**
 * @brief The only place in this firmware that calls into btp::aead. Seals
 * and opens channel C traffic (dongle<->robot, key L -- see
 * include/bally_channels.h) with AES-128-GCM, using the key a configured
 * KeyStore holds in RAM.
 *
 * Mirrors bally_dongle's lib/RadioSeal exactly, adapted to this side's
 * KeyStore being an owned instance (ROBOT::key_store) rather than a global
 * DongleKeyStore: configure() binds which instance to read from, so this
 * file never needs to know how the key got into RAM or who owns it.
 *
 * DELIBERATELY NOT INCLUDED BY BtpTransport.h/.cpp, and therefore never
 * compiled into anything env:native has to link. BTP/library.json's srcDir
 * pulls src/aead.cpp into every consumer unconditionally, and on a host with
 * neither mbedtls nor PSA backend that translation unit is intentionally
 * empty (BTP/src/aead.cpp's own comment: a stub returning an error would let
 * a build that believes it is encrypting ship without doing so -- so it
 * fails at LINK time instead). If BtpTransport ever called
 * btp::aead_seal_aes_gcm directly, `pio test -e native` would fail to link
 * every test binary the moment any of them touched
 * BtpEndpoint::send_logical/send_fragment.
 *
 * The fix is BtpEndpoint::SealFn: a function-pointer callback instead of a
 * direct dependency, so the actual crypto call lives here, in a library only
 * utils/BallyRobot (ESP-IDF/Arduino-only, never built under env:native) ever
 * includes. On the real target the ESP-IDF SDK's mbedtls/PSA backend links
 * without any extra configuration.
 */
namespace RadioSeal {

// BTP/docs/encryption.md section 2: the payload grows by exactly this many
// octets once sealed, regardless of cipher. Redeclared here (btp::aead
// itself has no public constant for it, only a comment) rather than
// hardcoded a second time at every call site.
constexpr std::size_t kTagSize = 16U;

/**
 * @brief Binds the KeyStore this namespace reads key L from. Call once, from
 * ROBOT::init() -- order relative to key_store.load_from_card() does not
 * matter, since seal()/open() read the KeyStore's CURRENT state at call
 * time, not a snapshot taken here, and both already fail closed on
 * !store.loaded().
 */
void configure(KeyStore& store) noexcept;

/**
 * @brief Matches BtpEndpoint::SealFn exactly, so this can be passed as the
 * callback at every channel C origination point with no wrapper needed.
 * `context` is unused (the key lives in the KeyStore configure() bound, a
 * process-wide holder like BtpEndpoint's own identity), kept only so the
 * signature matches SealFn's shape.
 *
 * Fails (false, nothing written to `out`) when no KeyStore is configured or
 * it has no loaded key -- fail-closed: no key, no frame, never a silent
 * cleartext fallback.
 */
bool seal(void* context, const btp::Header& header, std::uint16_t payload_size,
         const std::uint8_t* plaintext, std::uint8_t* out) noexcept;

/**
 * @brief Opens one already-reassembled channel C message.
 *
 * `header` MUST be the canonical logical header RxRouter hands back on
 * Outcome::Routed (FRAGMENTED cleared, fragment_index 0, fragment_count 1 --
 * RxRouter.cpp/fragmentation.cpp restore exactly that shape on completion,
 * and decode()'s own validation already guarantees it for an unfragmented
 * frame). `ciphertext_size` is the routed payload size unchanged (it already
 * includes the trailing tag); `out_plaintext` needs room for
 * `ciphertext_size - kTagSize` octets.
 *
 * Refuses (false) on any of: no key configured, the ENCRYPTED flag not set,
 * a cipher other than AES-128-GCM, or a tag that does not verify. The
 * caller MUST drop the message on false -- there is no fallback to reading
 * the still-sealed bytes as if they were plaintext.
 */
bool open(const btp::Header& header, std::uint16_t ciphertext_size,
         const std::uint8_t* ciphertext_and_tag,
         std::uint8_t* out_plaintext) noexcept;

}  // namespace RadioSeal

#endif  // RADIO_SEAL_H
