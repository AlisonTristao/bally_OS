#include <KeyStore.h>

#include <cstring>

// btp::crc32 is CRC-32/ISO-HDLC (poly 0xEDB88320 reflected, 0xFFFFFFFF seed,
// final inversion) -- the same routine and the same convention the frame
// envelope already uses, and the one scripts/provision_key.py was checked
// against. Reused instead of adding a second CRC to the firmware.
#include <btp/codec.hpp>

// SDCard is an ESP-IDF driver (driver/gpio.h, sdmmc_cmd.h), so it cannot be
// pulled into a host build. Only the thin file-reading layer at the bottom
// needs it; parse() stays pure and is what the native test exercises.
#if defined(ESP_PLATFORM)
#include <SDCard.h>
#endif

namespace {

// Field offsets of the layout documented in KeyStore.h. Named rather than
// inlined so the parser reads like the table.
constexpr std::size_t kOffMagic = 0U;
constexpr std::size_t kOffFormat = 8U;
constexpr std::size_t kOffCipher = 9U;
constexpr std::size_t kOffKdf = 10U;
constexpr std::size_t kOffIters = 12U;
constexpr std::size_t kOffSalt = 16U;
constexpr std::size_t kOffKeyE = 32U;
constexpr std::size_t kOffKeyL = 48U;
constexpr std::size_t kOffVerifyE = 64U;
constexpr std::size_t kOffVerifyL = 72U;
constexpr std::size_t kOffCrc = 92U;

constexpr std::uint8_t kMagic[8] = {'B', 'T', 'P', 'K', 'E', 'Y', 0x00, 0x00};

// ASCII, no NUL terminator: each label is hashed as 13 octets. Distinct per
// channel so one tag can never stand in for the other.
constexpr char kVerifyLabelE[] = "bally-canal-b";
constexpr char kVerifyLabelL[] = "bally-canal-c";

std::uint32_t read_u32_le(const std::uint8_t* data) noexcept {
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8U) |
           (static_cast<std::uint32_t>(data[2]) << 16U) |
           (static_cast<std::uint32_t>(data[3]) << 24U);
}

// Fixed-time compare. The verify tags are public, so this is not guarding a
// secret; it is here so that copying this helper for something that is secret
// stays safe by default.
bool equal_bytes(const std::uint8_t* left, const std::uint8_t* right,
                 std::size_t size) noexcept {
    std::uint8_t diff = 0U;
    for (std::size_t i = 0U; i < size; ++i) {
        diff = static_cast<std::uint8_t>(diff | (left[i] ^ right[i]));
    }
    return diff == 0U;
}

// Overwrite through a volatile pointer so the compiler cannot drop the store
// as dead on a buffer that is about to go out of scope or be reused.
void wipe(std::uint8_t* data, std::size_t size) noexcept {
    volatile std::uint8_t* target = data;
    for (std::size_t i = 0U; i < size; ++i) target[i] = 0U;
}

// ---------------------------------------------------------------------------
// SHA-256 (FIPS 180-4) and HMAC (RFC 2104), self-contained.
//
// Why not mbedtls, which does ship with the SDK: on ESP-IDF 6.0.1 mbedtls is
// 4.x / TF-PSA-Crypto, where the legacy digest API became private.
// <mbedtls/md.h> still exists, but every mbedtls_md_hmac* declaration in it
// sits inside "#if defined(MBEDTLS_DECLARE_PRIVATE_IDENTIFIERS)", which only
// mbedtls's own translation units define, and <mbedtls/sha256.h> moved to
// <mbedtls/private/sha256.h>. The supported replacement is PSA
// (psa_mac_compute), whose headers need MBEDTLS_CONFIG_FILE and
// TF_PSA_CRYPTO_USER_CONFIG_FILE on the command line to configure themselves
// the same way the linked library was built -- and PlatformIO compiles lib/*
// outside the IDF component graph, without those defines. Depending on that
// would be depending on a mismatch that only shows up at run time.
//
// The code below needs no configuration, is allocation-free, and also builds
// in env:native -- which is what lets the unit test actually exercise the
// verify_e/verify_l failures instead of skipping them off-target. Its
// correctness is not taken on faith: the golden vector in test/test_keystore
// was produced by Python's hmac/hashlib over the real key and label, so any
// deviation here surfaces immediately as Error::VerifyE. It was also checked
// against RFC 4231 test cases 1, 2 and 7 -- case 7 covering both the
// longer-than-block-size key and a multi-block message, neither of which the
// key file itself ever reaches.
//
// Only ever used on a 13-octet message with a 16-octet key, twice per boot.
// ---------------------------------------------------------------------------

constexpr std::size_t kShaDigestSize = 32U;
constexpr std::size_t kShaBlockSize = 64U;

constexpr std::uint32_t kShaRoundConstants[64] = {
    0x428A2F98U, 0x71374491U, 0xB5C0FBCFU, 0xE9B5DBA5U,
    0x3956C25BU, 0x59F111F1U, 0x923F82A4U, 0xAB1C5ED5U,
    0xD807AA98U, 0x12835B01U, 0x243185BEU, 0x550C7DC3U,
    0x72BE5D74U, 0x80DEB1FEU, 0x9BDC06A7U, 0xC19BF174U,
    0xE49B69C1U, 0xEFBE4786U, 0x0FC19DC6U, 0x240CA1CCU,
    0x2DE92C6FU, 0x4A7484AAU, 0x5CB0A9DCU, 0x76F988DAU,
    0x983E5152U, 0xA831C66DU, 0xB00327C8U, 0xBF597FC7U,
    0xC6E00BF3U, 0xD5A79147U, 0x06CA6351U, 0x14292967U,
    0x27B70A85U, 0x2E1B2138U, 0x4D2C6DFCU, 0x53380D13U,
    0x650A7354U, 0x766A0ABBU, 0x81C2C92EU, 0x92722C85U,
    0xA2BFE8A1U, 0xA81A664BU, 0xC24B8B70U, 0xC76C51A3U,
    0xD192E819U, 0xD6990624U, 0xF40E3585U, 0x106AA070U,
    0x19A4C116U, 0x1E376C08U, 0x2748774CU, 0x34B0BCB5U,
    0x391C0CB3U, 0x4ED8AA4AU, 0x5B9CCA4FU, 0x682E6FF3U,
    0x748F82EEU, 0x78A5636FU, 0x84C87814U, 0x8CC70208U,
    0x90BEFFFAU, 0xA4506CEBU, 0xBEF9A3F7U, 0xC67178F2U,
};

std::uint32_t rotate_right(std::uint32_t value, unsigned bits) noexcept {
    return (value >> bits) | (value << (32U - bits));
}

struct Sha256 {
    std::uint32_t state[8] = {0x6A09E667U, 0xBB67AE85U, 0x3C6EF372U,
                              0xA54FF53AU, 0x510E527FU, 0x9B05688CU,
                              0x1F83D9ABU, 0x5BE0CD19U};
    std::uint8_t block[kShaBlockSize] = {};
    std::size_t pending = 0U;
    std::uint64_t total = 0U;  // message length in octets
};

void sha256_compress(Sha256& sha) noexcept {
    std::uint32_t w[64] = {};
    for (std::size_t i = 0U; i < 16U; ++i) {
        w[i] = (static_cast<std::uint32_t>(sha.block[i * 4U]) << 24U) |
               (static_cast<std::uint32_t>(sha.block[i * 4U + 1U]) << 16U) |
               (static_cast<std::uint32_t>(sha.block[i * 4U + 2U]) << 8U) |
               static_cast<std::uint32_t>(sha.block[i * 4U + 3U]);
    }
    for (std::size_t i = 16U; i < 64U; ++i) {
        const std::uint32_t s0 = rotate_right(w[i - 15U], 7U) ^
                                 rotate_right(w[i - 15U], 18U) ^
                                 (w[i - 15U] >> 3U);
        const std::uint32_t s1 = rotate_right(w[i - 2U], 17U) ^
                                 rotate_right(w[i - 2U], 19U) ^
                                 (w[i - 2U] >> 10U);
        w[i] = w[i - 16U] + s0 + w[i - 7U] + s1;
    }

    std::uint32_t a = sha.state[0], b = sha.state[1], c = sha.state[2];
    std::uint32_t d = sha.state[3], e = sha.state[4], f = sha.state[5];
    std::uint32_t g = sha.state[6], h = sha.state[7];

    for (std::size_t i = 0U; i < 64U; ++i) {
        const std::uint32_t sigma1 =
            rotate_right(e, 6U) ^ rotate_right(e, 11U) ^ rotate_right(e, 25U);
        const std::uint32_t choose = (e & f) ^ (~e & g);
        const std::uint32_t t1 =
            h + sigma1 + choose + kShaRoundConstants[i] + w[i];
        const std::uint32_t sigma0 =
            rotate_right(a, 2U) ^ rotate_right(a, 13U) ^ rotate_right(a, 22U);
        const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
        const std::uint32_t t2 = sigma0 + majority;

        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }

    sha.state[0] += a;
    sha.state[1] += b;
    sha.state[2] += c;
    sha.state[3] += d;
    sha.state[4] += e;
    sha.state[5] += f;
    sha.state[6] += g;
    sha.state[7] += h;
}

void sha256_update(Sha256& sha, const std::uint8_t* data,
                   std::size_t size) noexcept {
    sha.total += size;
    for (std::size_t i = 0U; i < size; ++i) {
        sha.block[sha.pending++] = data[i];
        if (sha.pending == kShaBlockSize) {
            sha256_compress(sha);
            sha.pending = 0U;
        }
    }
}

void sha256_finish(Sha256& sha, std::uint8_t* digest) noexcept {
    const std::uint64_t bits = sha.total * 8U;

    const std::uint8_t padding = 0x80U;
    sha256_update(sha, &padding, 1U);
    const std::uint8_t zero = 0x00U;
    while (sha.pending != kShaBlockSize - 8U) sha256_update(sha, &zero, 1U);

    for (std::size_t i = 0U; i < 8U; ++i) {
        sha.block[kShaBlockSize - 8U + i] =
            static_cast<std::uint8_t>(bits >> (56U - (8U * i)));
    }
    sha256_compress(sha);

    for (std::size_t i = 0U; i < 8U; ++i) {
        digest[i * 4U] = static_cast<std::uint8_t>(sha.state[i] >> 24U);
        digest[i * 4U + 1U] = static_cast<std::uint8_t>(sha.state[i] >> 16U);
        digest[i * 4U + 2U] = static_cast<std::uint8_t>(sha.state[i] >> 8U);
        digest[i * 4U + 3U] = static_cast<std::uint8_t>(sha.state[i]);
    }
}

// HMAC-SHA256 of one contiguous message. Keys longer than the block size are
// hashed first, per RFC 2104; the keys used here are 16 octets, so that branch
// never runs on the robot but is kept so the helper is not silently wrong if
// reused.
void hmac_sha256(const std::uint8_t* key, std::size_t key_size,
                 const std::uint8_t* message, std::size_t message_size,
                 std::uint8_t* digest) noexcept {
    std::uint8_t padded_key[kShaBlockSize] = {};
    if (key_size > kShaBlockSize) {
        Sha256 shrink;
        sha256_update(shrink, key, key_size);
        sha256_finish(shrink, padded_key);
    } else {
        std::memcpy(padded_key, key, key_size);
    }

    std::uint8_t pad[kShaBlockSize] = {};

    for (std::size_t i = 0U; i < kShaBlockSize; ++i) {
        pad[i] = static_cast<std::uint8_t>(padded_key[i] ^ 0x36U);
    }
    Sha256 inner;
    sha256_update(inner, pad, sizeof(pad));
    sha256_update(inner, message, message_size);
    std::uint8_t inner_digest[kShaDigestSize] = {};
    sha256_finish(inner, inner_digest);

    for (std::size_t i = 0U; i < kShaBlockSize; ++i) {
        pad[i] = static_cast<std::uint8_t>(padded_key[i] ^ 0x5CU);
    }
    Sha256 outer;
    sha256_update(outer, pad, sizeof(pad));
    sha256_update(outer, inner_digest, sizeof(inner_digest));
    sha256_finish(outer, digest);

    // Everything above is derived from a key; none of it stays on the stack.
    wipe(padded_key, sizeof(padded_key));
    wipe(pad, sizeof(pad));
    wipe(inner_digest, sizeof(inner_digest));
}

// True when HMAC-SHA256(key, label) starts with the expected tag.
bool verify_tag_matches(const std::uint8_t* key, std::size_t key_size,
                        const char* label, std::size_t label_size,
                        const std::uint8_t* expected,
                        std::size_t expected_size) noexcept {
    std::uint8_t digest[kShaDigestSize] = {};
    hmac_sha256(key, key_size,
                reinterpret_cast<const std::uint8_t*>(label), label_size,
                digest);
    const bool matches = equal_bytes(digest, expected, expected_size);
    wipe(digest, sizeof(digest));
    return matches;
}

}  // namespace

KeyStore::Error KeyStore::parse(const std::uint8_t* data, std::size_t size,
                                Contents* out) noexcept {
    if (data == nullptr || size != kFileSize) return Error::Size;

    for (std::size_t i = 0U; i < sizeof(kMagic); ++i) {
        if (data[kOffMagic + i] != kMagic[i]) return Error::Magic;
    }
    if (data[kOffFormat] != kFormatTwoKeys) return Error::Format;
    if (data[kOffCipher] != kCipherAes128Gcm) return Error::Cipher;
    if (data[kOffKdf] != kKdfPbkdf2HmacSha256) return Error::Kdf;

    // CRC before the key material: a corrupted file must be reported as
    // corrupted, not as a wrong password.
    if (btp::crc32(data, kOffCrc) != read_u32_le(data + kOffCrc)) {
        return Error::Crc32;
    }

    // Neither the salt nor the iteration count is validated here, on purpose.
    // The robot never runs PBKDF2 -- both fields are carried for the bench and
    // for whoever has to reproduce the derivation elsewhere, so a value the
    // robot does not use must not be able to reject a good card. The CRC above
    // already covers them against corruption, and
    // scripts/provision_key.py --verify is what checks them against the
    // project's contract.

    // Each key is checked against its own label, so the report says which one
    // is wrong: that is the entire reason the two tags exist.
    if (!verify_tag_matches(data + kOffKeyE, kKeyLength, kVerifyLabelE,
                            sizeof(kVerifyLabelE) - 1U, data + kOffVerifyE,
                            kVerifyLength)) {
        return Error::VerifyE;
    }
    if (!verify_tag_matches(data + kOffKeyL, kKeyLength, kVerifyLabelL,
                            sizeof(kVerifyLabelL) - 1U, data + kOffVerifyL,
                            kVerifyLength)) {
        return Error::VerifyL;
    }

    if (out != nullptr) {
        out->iterations = read_u32_le(data + kOffIters);
        std::memcpy(out->salt, data + kOffSalt, kSaltLength);
        std::memcpy(out->key_e, data + kOffKeyE, kKeyLength);
        std::memcpy(out->key_l, data + kOffKeyL, kKeyLength);
        std::memcpy(out->verify_e, data + kOffVerifyE, kVerifyLength);
        std::memcpy(out->verify_l, data + kOffVerifyL, kVerifyLength);
    }
    return Error::Ok;
}

const char* KeyStore::error_field(Error error) noexcept {
    switch (error) {
        case Error::Ok:       return "ok";
        case Error::Size:     return "size";
        case Error::Magic:    return "magic";
        case Error::Format:   return "format";
        case Error::Cipher:   return "cipher";
        case Error::Kdf:      return "kdf";
        case Error::Crc32:    return "crc32";
        case Error::VerifyE:  return "verify_e";
        case Error::VerifyL:  return "verify_l";
        case Error::File:     return "file";
    }
    return "unknown";
}

bool KeyStore::load(const std::uint8_t* data, std::size_t size) noexcept {
    Contents parsed{};
    const Error error = parse(data, size, &parsed);
    if (error != Error::Ok) {
        // A failed reload must not leave the previous keys reachable: the
        // caller is about to report an error, not to keep running with key
        // material it can no longer account for.
        unload();
        last_error_ = error;
        return false;
    }

    contents_ = parsed;
    wipe(parsed.key_e, sizeof(parsed.key_e));
    wipe(parsed.key_l, sizeof(parsed.key_l));
    loaded_ = true;
    last_error_ = Error::Ok;
    return true;
}

void KeyStore::unload() noexcept {
    wipe(contents_.key_e, sizeof(contents_.key_e));
    wipe(contents_.key_l, sizeof(contents_.key_l));
    contents_ = Contents{};
    loaded_ = false;
}

const std::uint8_t* KeyStore::key_e() const noexcept {
    return loaded_ ? contents_.key_e : nullptr;
}

const std::uint8_t* KeyStore::key_l() const noexcept {
    return loaded_ ? contents_.key_l : nullptr;
}

const std::uint8_t* KeyStore::verify_e() const noexcept {
    return loaded_ ? contents_.verify_e : nullptr;
}

const std::uint8_t* KeyStore::verify_l() const noexcept {
    return loaded_ ? contents_.verify_l : nullptr;
}

#if defined(ESP_PLATFORM)

bool KeyStore::load_from_card(SDCard& card) noexcept {
    if (!card.is_mounted()) {
        unload();
        last_error_ = Error::File;
        return false;
    }

    // One octet more than the format: a longer file then reads as kFileSize+1
    // and is rejected by name ("size") instead of being silently truncated
    // into something that looks valid.
    std::uint8_t buffer[kFileSize + 1U] = {};
    std::size_t bytes_read = 0U;
    if (!card.read_file(KEY_STORE_FILE, buffer, sizeof(buffer), &bytes_read)) {
        unload();
        last_error_ = Error::File;
        return false;
    }

    const bool ok = load(buffer, bytes_read);
    // The buffer held both keys in the clear; do not leave them on the stack.
    wipe(buffer, sizeof(buffer));
    return ok;
}

#else

// Host builds have no SD driver. Kept as a definition rather than omitted so
// that using it off-target is a clean runtime failure instead of a link error
// in whatever test happens to reference it.
bool KeyStore::load_from_card(SDCard&) noexcept {
    unload();
    last_error_ = Error::File;
    return false;
}

#endif  // ESP_PLATFORM
