#ifndef KEY_STORE_H
#define KEY_STORE_H

#include <cstddef>
#include <cstdint>

// Only a reference is needed, so the SD driver stays out of this header (see
// CONTRIBUTING.md, "Nova biblioteca" item 2). That also keeps KeyStore's
// parser buildable in env:native, where no ESP-IDF driver exists.
class SDCard;

// SD card root file holding the two already-derived channel keys. Same
// convention as ROBOT_SETTINGS_FILE (lib/RobotSettings/RobotSettings.h):
// relative to the SDCard mount point.
#define KEY_STORE_FILE "bally.key"

/**
 * @brief The robot's two provisioned BTP keys, read once at boot.
 *
 * The system has three channels and two keys of different reach:
 *
 *   channel B  TraceView <-> robot   key E, one per robot. Isolates robots
 *                                    from each other; protects data and
 *                                    commands.
 *   channel C  dongle    <-> robot   key L, fleet-wide. One dongle talks to
 *                                    many robots and only administers the
 *                                    link.
 *
 * The two passwords behind them are independent by purpose. If both keys came
 * out of a single password by domain separation, whoever held that password
 * would hold both keys and the two channels would collapse into one. No
 * password ever travels over the protocol: each is typed at the end that
 * needs it.
 *
 * Nobody types anything into the robot, so the robot NEVER runs PBKDF2. It
 * only reads keys that are already derived, from a 96-octet file written by
 * scripts/provision_key.py -- which is the canonical source of the salt and
 * the iteration count that the dongle and TraceView must reproduce byte for
 * byte.
 *
 * SECURITY RULE, do not weaken it: neither key_e() nor key_l() may ever be
 * printed, logged, hex-dumped or traced, not even partially and not even
 * behind a DEBUG flag. If a diagnostic is needed, print verify_e()/verify_l()
 * instead: those are the images of the keys under a one-way function, are
 * public by construction, and are exactly what tells the bench WHICH key is
 * wrong.
 *
 * No dynamic allocation: the file image is parsed off the caller's buffer and
 * the keys live in this object's fixed-size fields.
 */
class KeyStore {
public:
    // Wire layout of KEY_STORE_FILE, mirrored from provision_key.py. Every
    // integer is little-endian.
    //
    //   offset size field
    //   0      8    magic     "BTPKEY\0\0"
    //   8      1    format    0x02 (two keys)
    //   9      1    cipher    0x00 AES-128-GCM (BTP CIPHER_ID 0)
    //   10     1    kdf       0x01 PBKDF2-HMAC-SHA256
    //   11     1    reserved  0x00
    //   12     4    iters     u32 LE
    //   16     16   salt
    //   32     16   key_e
    //   48     16   key_l
    //   64     8    verify_e  HMAC-SHA256(key_e, "bally-canal-b")[:8]
    //   72     8    verify_l  HMAC-SHA256(key_l, "bally-canal-c")[:8]
    //   80     12   reserved  zeros
    //   92     4    crc32     CRC-32/ISO-HDLC over octets 0..91, LE
    static constexpr std::size_t kFileSize = 96U;

    // AES-128-GCM is BTP CIPHER_ID 0 and takes a 16-octet key
    // (BTP/docs/encryption.md section 3). Key sizes are not interchangeable
    // there, so this is a hard length, not a maximum.
    static constexpr std::size_t kKeyLength = 16U;
    static constexpr std::size_t kSaltLength = 16U;
    static constexpr std::size_t kVerifyLength = 8U;

    static constexpr std::uint8_t kFormatTwoKeys = 0x02U;
    static constexpr std::uint8_t kCipherAes128Gcm = 0x00U;
    static constexpr std::uint8_t kKdfPbkdf2HmacSha256 = 0x01U;

    /**
     * @brief Which field rejected the file.
     *
     * One enumerator per field so the failure can be reported by name. That
     * is the whole reason verify_e/verify_l exist: a robot that fails at boot
     * saying "verify_l" is diagnosable, whereas a robot that boots fine and
     * silently has one dead channel is not.
     */
    enum class Error : std::uint8_t {
        Ok = 0,
        Size,      // wrong length: not exactly kFileSize octets
        Magic,     // not "BTPKEY\0\0"
        Format,    // format octet is not kFormatTwoKeys
        Cipher,    // cipher octet is not kCipherAes128Gcm
        Kdf,       // kdf octet is not kKdfPbkdf2HmacSha256
        Crc32,     // stored CRC does not cover the first 92 octets
        VerifyE,   // key_e does not match verify_e: wrong password E
        VerifyL,   // key_l does not match verify_l: wrong password L
        File,      // card not mounted, or KEY_STORE_FILE unreadable
    };

    /**
     * @brief Everything a validated file carries.
     *
     * key_e/key_l are secret; salt, iterations and the verify tags are not.
     * Callers that only need the keys should use the KeyStore accessors and
     * leave this type to the parser and its tests.
     */
    struct Contents {
        std::uint32_t iterations = 0U;
        std::uint8_t salt[kSaltLength] = {};
        std::uint8_t key_e[kKeyLength] = {};
        std::uint8_t key_l[kKeyLength] = {};
        std::uint8_t verify_e[kVerifyLength] = {};
        std::uint8_t verify_l[kVerifyLength] = {};
    };

    /**
     * @brief Validate a complete file image and, optionally, extract it.
     *
     * Pure: no SD card, no ESP-IDF, no allocation, so this is what the
     * env:native test exercises -- including the two HMAC-SHA256 tag checks,
     * whose backend is built into this library exactly so that the host build
     * covers them too (see the comment in KeyStore.cpp). Fields are checked in
     * the order of the layout above (size, magic, format, cipher, kdf, crc32,
     * then the two verify tags) and the first failure is returned, so the
     * reported field is always the outermost thing that is wrong.
     *
     * @param data Buffer holding the file. May be null only when size is 0.
     * @param size Number of octets read; must be exactly kFileSize.
     * @param out Optional destination. Left untouched when validation fails.
     */
    static Error parse(const std::uint8_t* data, std::size_t size,
                       Contents* out = nullptr) noexcept;

    /**
     * @brief Name of the field an Error refers to, for logging.
     *
     * Returns the field name as it appears in the layout ("magic", "crc32",
     * "verify_e", ...), never a key.
     */
    static const char* error_field(Error error) noexcept;

    /**
     * @brief Parse an in-memory file image and keep the keys in this object.
     *
     * On failure the previously held keys are wiped, so a bad reload never
     * leaves stale key material behind.
     */
    bool load(const std::uint8_t* data, std::size_t size) noexcept;

    /**
     * @brief Read KEY_STORE_FILE from the card and load it.
     *
     * Goes through SDCard, the one file access path this firmware has; it
     * does not open the VFS itself. Only available on the ESP-IDF build.
     */
    bool load_from_card(SDCard& card) noexcept;

    /**
     * @brief Forget the keys, overwriting the storage that held them.
     */
    void unload() noexcept;

    bool loaded() const noexcept { return loaded_; }

    /** @brief Which field rejected the last load, or Ok. */
    Error last_error() const noexcept { return last_error_; }

    /**
     * @brief Channel B key (TraceView <-> robot), kKeyLength octets.
     *
     * The caller does NOT own this memory and must not copy it around: it
     * points into this KeyStore, stays valid while it remains loaded, and is
     * invalidated by unload() or a failed reload. Null when not loaded.
     * Never print it (see the class comment).
     */
    const std::uint8_t* key_e() const noexcept;

    /**
     * @brief Channel C key (dongle <-> robot), kKeyLength octets.
     *
     * Same ownership and same non-printing rule as key_e().
     */
    const std::uint8_t* key_l() const noexcept;

    /**
     * @brief Public 8-octet tag of key E. Safe to log.
     */
    const std::uint8_t* verify_e() const noexcept;

    /**
     * @brief Public 8-octet tag of key L. Safe to log.
     */
    const std::uint8_t* verify_l() const noexcept;

    /** @brief PBKDF2 iteration count recorded in the file. */
    std::uint32_t iterations() const noexcept { return contents_.iterations; }

private:
    Contents contents_{};
    bool loaded_ = false;
    Error last_error_ = Error::Ok;
};

#endif  // KEY_STORE_H
