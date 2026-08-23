#!/usr/bin/env python3
"""Write the robot's bally.key (two provisioned BTP keys) to an SD card.

This script is the CANONICAL definition of the key-derivation contract. The
robot never runs PBKDF2: nobody types a password into it, so it only ever
reads keys that are already derived (see lib/KeyStore). The dongle and
TraceView, which do have a keyboard in front of them, derive their key from
the typed password at runtime -- and they MUST reproduce, byte for byte, what
the constants below produce. Change anything here and every previously
provisioned card, dongle and TraceView install stops agreeing.

Why two independent passwords instead of one password and two domains: the
two channels have different reach. Key E is per robot (channel B, TraceView
<-> robot: isolates robots from each other, protects data and commands). Key L
belongs to the whole fleet (channel C, dongle <-> robot: one dongle talks to
many robots, and only administers the link). If both keys came out of a single
password, whoever holds that password holds both keys and the two channels
collapse into one. Neither password ever travels over the protocol.
"""

import argparse
import getpass
import hashlib
import hmac
import os
import struct
import sys
import zlib

# ---------------------------------------------------------------------------
# The derivation contract. These constants are what the three ends have to
# agree on; everything else in this file is bookkeeping.
# ---------------------------------------------------------------------------

# Fixed project salt: the 16 ASCII octets of the string below, with no NUL
# terminator. A fixed salt is a deliberate trade-off -- it is what lets the
# dongle and TraceView derive the same key from the same typed password with
# no provisioning handshake at all. It costs the usual property of a random
# salt (two installs with the same password get the same key), which is
# acceptable here because these are per-fleet/per-robot secrets, not user
# account passwords.
KDF_SALT = b"bally-kdf-salt-1"

# PBKDF2-HMAC-SHA256 iteration count. High enough to make an offline guess of
# a weak password expensive, low enough that the dongle/TraceView side can
# still derive at startup. The robot never pays this cost at all.
KDF_ITERATIONS = 200000

# AES-128-GCM (BTP CIPHER_ID 0, BTP/docs/encryption.md section 3) takes a
# 16-octet key, so that is the PBKDF2 output length. Not interchangeable with
# ChaCha20-Poly1305's 32 octets: BTP rejects a key length that does not match
# the selected cipher exactly.
KEY_LENGTH = 16

# Verification labels: ASCII, no NUL terminator. verify_* is the first 8
# octets of HMAC-SHA256(key, label). It is NOT a secret -- it is the image of
# the key under a one-way function -- and it exists so a wrong password shows
# up at boot as "key E is wrong" instead of, weeks later, as "channel B does
# not work". Distinct labels per channel keep one tag from ever standing in
# for the other.
VERIFY_LABEL_E = b"bally-canal-b"
VERIFY_LABEL_L = b"bally-canal-c"
VERIFY_LENGTH = 8

# Passwords are encoded UTF-8 before entering PBKDF2. Stated explicitly
# because it is exactly the kind of implicit choice that makes two ends
# disagree the first time somebody types a non-ASCII password.
PASSWORD_ENCODING = "utf-8"

# ---------------------------------------------------------------------------
# File layout: exactly 96 octets, every integer little-endian.
#
#   offset size field
#   0      8    magic     "BTPKEY\0\0"
#   8      1    format    0x02 (two keys)
#   9      1    cipher    0x00 AES-128-GCM (BTP CIPHER_ID 0)
#   10     1    kdf       0x01 PBKDF2-HMAC-SHA256
#   11     1    reserved  0x00
#   12     4    iters     u32 LE
#   16     16   salt
#   32     16   key_e     channel B (TraceView <-> robot), one per robot
#   48     16   key_l     channel C (dongle <-> robot), fleet-wide
#   64     8    verify_e  HMAC-SHA256(key_e, "bally-canal-b")[:8]
#   72     8    verify_l  HMAC-SHA256(key_l, "bally-canal-c")[:8]
#   80     12   reserved  zeros
#   92     4    crc32     CRC-32/ISO-HDLC over octets 0..91, LE
# ---------------------------------------------------------------------------

MAGIC = b"BTPKEY\x00\x00"
FORMAT_TWO_KEYS = 0x02
CIPHER_AES_128_GCM = 0x00
KDF_PBKDF2_HMAC_SHA256 = 0x01

FILE_SIZE = 96
OFF_MAGIC = 0
OFF_FORMAT = 8
OFF_CIPHER = 9
OFF_KDF = 10
OFF_RESERVED1 = 11
OFF_ITERS = 12
OFF_SALT = 16
OFF_KEY_E = 32
OFF_KEY_L = 48
OFF_VERIFY_E = 64
OFF_VERIFY_L = 72
OFF_RESERVED2 = 80
OFF_CRC = 92

DEFAULT_FILENAME = "bally.key"


def derive_key(password):
    """Derive one 16-octet channel key from a typed password."""
    return hashlib.pbkdf2_hmac(
        "sha256",
        password.encode(PASSWORD_ENCODING),
        KDF_SALT,
        KDF_ITERATIONS,
        dklen=KEY_LENGTH,
    )


def verify_tag(key, label):
    """Public 8-octet tag that identifies a key without revealing it."""
    return hmac.new(key, label, hashlib.sha256).digest()[:VERIFY_LENGTH]


def file_crc32(blob):
    """CRC-32/ISO-HDLC over the first 92 octets.

    zlib.crc32 is the same polynomial (0xEDB88320, reflected), the same
    0xFFFFFFFF initial value and the same final inversion as btp::crc32 in
    BTP/src/codec.cpp, so the firmware can validate this file with the CRC
    routine it already links instead of growing a second one. Checked against
    that implementation on "", "a", "123456789", "bally-kdf-salt-1", "BTPKEY"
    and a 92-octet ramp.
    """
    return zlib.crc32(blob[:OFF_CRC]) & 0xFFFFFFFF


def build_blob(password_e, password_l):
    """Build the complete 96-octet file image from the two passwords."""
    key_e = derive_key(password_e)
    key_l = derive_key(password_l)

    blob = bytearray(FILE_SIZE)
    blob[OFF_MAGIC:OFF_MAGIC + 8] = MAGIC
    blob[OFF_FORMAT] = FORMAT_TWO_KEYS
    blob[OFF_CIPHER] = CIPHER_AES_128_GCM
    blob[OFF_KDF] = KDF_PBKDF2_HMAC_SHA256
    blob[OFF_RESERVED1] = 0x00
    blob[OFF_ITERS:OFF_ITERS + 4] = struct.pack("<I", KDF_ITERATIONS)
    blob[OFF_SALT:OFF_SALT + 16] = KDF_SALT
    blob[OFF_KEY_E:OFF_KEY_E + KEY_LENGTH] = key_e
    blob[OFF_KEY_L:OFF_KEY_L + KEY_LENGTH] = key_l
    blob[OFF_VERIFY_E:OFF_VERIFY_E + VERIFY_LENGTH] = verify_tag(
        key_e, VERIFY_LABEL_E)
    blob[OFF_VERIFY_L:OFF_VERIFY_L + VERIFY_LENGTH] = verify_tag(
        key_l, VERIFY_LABEL_L)
    # OFF_RESERVED2..OFF_CRC stay zero, and the CRC covers them, so a future
    # version cannot quietly reuse those octets without invalidating readers
    # that predate the change.
    blob[OFF_CRC:OFF_CRC + 4] = struct.pack("<I", file_crc32(blob))
    return bytes(blob)


def check_blob(blob):
    """Return a list of human-readable problems, empty when the file is good.

    Deliberately follows the same field order lib/KeyStore validates in, so a
    complaint on the bench reads like what the robot prints at boot.
    """
    if len(blob) != FILE_SIZE:
        # Nothing below can be trusted at the wrong size, so stop here.
        return ["size: expected %d octets, got %d" % (FILE_SIZE, len(blob))]

    problems = []
    magic = bytes(blob[OFF_MAGIC:OFF_MAGIC + 8])
    if magic != MAGIC:
        problems.append("magic: expected %s, got %s"
                        % (MAGIC.hex(), magic.hex()))
    if blob[OFF_FORMAT] != FORMAT_TWO_KEYS:
        problems.append("format: expected 0x%02X, got 0x%02X"
                        % (FORMAT_TWO_KEYS, blob[OFF_FORMAT]))
    if blob[OFF_CIPHER] != CIPHER_AES_128_GCM:
        problems.append("cipher: expected 0x%02X, got 0x%02X"
                        % (CIPHER_AES_128_GCM, blob[OFF_CIPHER]))
    if blob[OFF_KDF] != KDF_PBKDF2_HMAC_SHA256:
        problems.append("kdf: expected 0x%02X, got 0x%02X"
                        % (KDF_PBKDF2_HMAC_SHA256, blob[OFF_KDF]))

    iters = struct.unpack("<I", blob[OFF_ITERS:OFF_ITERS + 4])[0]
    if iters != KDF_ITERATIONS:
        problems.append("iters: expected %d, got %d" % (KDF_ITERATIONS, iters))

    salt = bytes(blob[OFF_SALT:OFF_SALT + 16])
    if salt != KDF_SALT:
        problems.append("salt: expected %s, got %s"
                        % (KDF_SALT.hex(), salt.hex()))

    stored_crc = struct.unpack("<I", blob[OFF_CRC:OFF_CRC + 4])[0]
    expected_crc = file_crc32(blob)
    if stored_crc != expected_crc:
        problems.append("crc32: expected 0x%08X, got 0x%08X"
                        % (expected_crc, stored_crc))

    # verify_e/verify_l are deliberately NOT re-derived here: that needs the
    # passwords, which --verify never asks for. They are checked by whoever
    # holds the keys, which on the robot is lib/KeyStore at boot.
    return problems


def read_password(channel, description):
    """Prompt twice for one password, refusing empty and mismatched input.

    Typed twice because a typo here does not fail now -- it fails much later,
    and only as "that channel does not work", with no hint of which end is
    wrong.
    """
    while True:
        first = getpass.getpass("Password %s (%s): " % (channel, description))
        if not first:
            print("Empty password refused.", file=sys.stderr)
            continue
        second = getpass.getpass("Repeat password %s: " % channel)
        if first != second:
            print("The two entries differ, try again.", file=sys.stderr)
            continue
        return first


def report(verb, path, blob):
    """Print what was written (or read back).

    Prints no key and no password, not even partially. verify_e/verify_l are
    public by construction and are the whole point of being printed: they are
    what lets the bench tell WHICH key is wrong.
    """
    print("%s %s" % (verb, path))
    print("  size       : %d octets" % len(blob))
    print("  iterations : %d"
          % struct.unpack("<I", blob[OFF_ITERS:OFF_ITERS + 4])[0])
    print("  verify_e   : %s  (channel B, TraceView <-> robot)"
          % bytes(blob[OFF_VERIFY_E:OFF_VERIFY_E + VERIFY_LENGTH]).hex())
    print("  verify_l   : %s  (channel C, dongle <-> robot)"
          % bytes(blob[OFF_VERIFY_L:OFF_VERIFY_L + VERIFY_LENGTH]).hex())


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Provision bally.key (channel keys E and L) on an SD card.",
        epilog="Neither password travels over the protocol: each one is typed "
               "at the end that needs it. The robot only ever reads the "
               "already-derived keys from this file.")
    parser.add_argument(
        "output", nargs="?", default=DEFAULT_FILENAME,
        help="path of the file to write (default: %s, i.e. the root of the "
             "card when run from there)" % DEFAULT_FILENAME)
    parser.add_argument(
        "--verify", metavar="FILE",
        help="write nothing: read FILE and check size, magic, format, cipher, "
             "kdf, iters, salt and crc32, naming whatever does not match")
    parser.add_argument(
        "--force", action="store_true",
        help="overwrite an existing output file")
    # A password on the command line lands in the shell history and in the
    # process list. These two options exist only so an automated test can
    # rebuild the golden vector without a TTY -- never provision a real card
    # with them.
    parser.add_argument(
        "--password-e", metavar="PW",
        help="FOR AUTOMATED TESTS ONLY: password for key E, non-interactive")
    parser.add_argument(
        "--password-l", metavar="PW",
        help="FOR AUTOMATED TESTS ONLY: password for key L, non-interactive")
    args = parser.parse_args(argv)

    if args.verify:
        try:
            with open(args.verify, "rb") as handle:
                blob = handle.read()
        except OSError as error:
            print("cannot read %s: %s" % (args.verify, error), file=sys.stderr)
            return 2
        problems = check_blob(blob)
        if problems:
            print("%s is NOT valid:" % args.verify)
            for problem in problems:
                print("  %s" % problem)
            return 1
        print("%s is valid" % args.verify)
        report("read", args.verify, blob)
        return 0

    if (args.password_e is None) != (args.password_l is None):
        parser.error("--password-e and --password-l go together")

    if args.password_e is not None:
        if not args.password_e or not args.password_l:
            parser.error("empty password refused")
        password_e, password_l = args.password_e, args.password_l
    else:
        print("Two independent passwords. Neither is ever transmitted; each "
              "is typed at the end that needs it.")
        password_e = read_password("E", "channel B, TraceView <-> this robot")
        password_l = read_password("L", "channel C, dongle <-> fleet")

    if os.path.exists(args.output) and not args.force:
        print("%s already exists (use --force to overwrite)" % args.output,
              file=sys.stderr)
        return 2

    blob = build_blob(password_e, password_l)
    with open(args.output, "wb") as handle:
        handle.write(blob)
    report("wrote", args.output, blob)
    return 0


if __name__ == "__main__":
    sys.exit(main())
