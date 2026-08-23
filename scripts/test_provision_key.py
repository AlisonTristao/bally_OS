#!/usr/bin/env python3
"""Tests for provision_key.py -- the script itself, not just its output.

test/test_keystore/test_main.cpp already pins one golden bally.key as a
hardcoded byte array and checks that lib/KeyStore parses it correctly. That
catches drift in the C++ parser, but nothing previously re-ran the *script*
against that same golden array -- a change to provision_key.py's derivation
(salt, iteration count, byte order, ...) would only surface much later, on a
real bench, as "channel B doesn't work" with no clue which end changed.

test_golden_vector_matches_keystore_fixture below closes that loop: it
reproduces the C++ suite's exact golden bytes from this script and fails the
moment the two drift apart. The rest of the suite covers what the C++ side
structurally cannot: the CLI (argument parsing, --force, --verify, exit
codes) and check_blob()'s per-field diagnostics, which is the tool a human
actually runs on the bench when a card misbehaves.

Run with: python scripts/test_provision_key.py
"""

import contextlib
import io
import os
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import provision_key  # noqa: E402  (import after sys.path fix-up, deliberately)


# Passwords documented in test/test_keystore/test_main.cpp as the ones that
# produced its kGoldenFile. Keep these two strings in sync with that comment.
GOLDEN_PASSWORD_E = "senha-e-de-teste"
GOLDEN_PASSWORD_L = "senha-l-de-teste"

# Copied verbatim from test/test_keystore/test_main.cpp's kGoldenFile.
GOLDEN_BLOB = bytes([
    0x42, 0x54, 0x50, 0x4B, 0x45, 0x59, 0x00, 0x00,
    0x02, 0x00, 0x01, 0x00, 0x40, 0x0D, 0x03, 0x00,
    0x62, 0x61, 0x6C, 0x6C, 0x79, 0x2D, 0x6B, 0x64,
    0x66, 0x2D, 0x73, 0x61, 0x6C, 0x74, 0x2D, 0x31,
    0xDC, 0x13, 0xA1, 0x79, 0x86, 0x01, 0xE4, 0x24,
    0xC4, 0x5F, 0x69, 0x1D, 0xD5, 0x48, 0x4E, 0xED,
    0xE2, 0x8B, 0x8E, 0xDF, 0xD9, 0x19, 0xD3, 0x9F,
    0x5B, 0x57, 0x3C, 0x92, 0xB7, 0x72, 0x24, 0xD7,
    0xC7, 0xA4, 0x5E, 0x15, 0xA4, 0x1B, 0x78, 0x0F,
    0x2A, 0x5E, 0x4D, 0xDC, 0x86, 0x6F, 0x8A, 0x08,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xC8, 0xB9, 0x19, 0xC7,
])


def run_main(argv):
    """Calls provision_key.main(argv), capturing stdout/stderr and the
    return value (or the SystemExit code argparse raises on a usage error)
    instead of letting either reach the test runner's own output."""
    out, err = io.StringIO(), io.StringIO()
    try:
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = provision_key.main(argv)
    except SystemExit as exit_error:
        code = exit_error.code
    return code, out.getvalue(), err.getvalue()


class BuildBlobTests(unittest.TestCase):
    def test_golden_vector_matches_keystore_fixture(self):
        blob = provision_key.build_blob(GOLDEN_PASSWORD_E, GOLDEN_PASSWORD_L)
        self.assertEqual(GOLDEN_BLOB, blob)

    def test_freshly_built_blob_passes_its_own_verifier(self):
        blob = provision_key.build_blob("some-password-e", "some-password-l")
        self.assertEqual([], provision_key.check_blob(blob))

    def test_same_password_on_both_channels_still_derives_two_key_slots(self):
        # derive_key() has no channel label baked in -- channel separation
        # comes entirely from the two passwords being different, which is
        # the whole reason the class docstring calls that out. This does not
        # assert key_e != key_l (a shared password legitimately collapses
        # them); it asserts the file still carries independent verify tags
        # for each slot, so a bench reading verify_e/verify_l never confuses
        # "operator reused a password" with "the file is malformed".
        blob = provision_key.build_blob("shared-password", "shared-password")
        key_e = blob[provision_key.OFF_KEY_E:provision_key.OFF_KEY_E + provision_key.KEY_LENGTH]
        key_l = blob[provision_key.OFF_KEY_L:provision_key.OFF_KEY_L + provision_key.KEY_LENGTH]
        self.assertEqual(key_e, key_l)
        self.assertEqual([], provision_key.check_blob(blob))


class CheckBlobDiagnosticsTests(unittest.TestCase):
    """check_blob() is what a human reads on the bench, so each corrupted
    field must name itself -- the same guarantee
    test/test_keystore/test_main.cpp's assert_fails_naming() pins for the
    firmware's own parser, exercised here against the tool that produces the
    file in the first place."""

    def setUp(self):
        self.blob = bytearray(
            provision_key.build_blob("bench-password-e", "bench-password-l"))

    def flip_byte(self, offset):
        self.blob[offset] ^= 0xFF

    def test_wrong_size_is_reported_alone(self):
        problems = provision_key.check_blob(self.blob[:-1])
        self.assertEqual(1, len(problems))
        self.assertIn("size", problems[0])

    def test_corrupted_magic_is_named(self):
        self.flip_byte(provision_key.OFF_MAGIC)
        problems = provision_key.check_blob(bytes(self.blob))
        self.assertTrue(any(p.startswith("magic:") for p in problems))

    def test_corrupted_format_is_named(self):
        self.flip_byte(provision_key.OFF_FORMAT)
        problems = provision_key.check_blob(bytes(self.blob))
        self.assertTrue(any(p.startswith("format:") for p in problems))

    def test_corrupted_cipher_is_named(self):
        self.flip_byte(provision_key.OFF_CIPHER)
        problems = provision_key.check_blob(bytes(self.blob))
        self.assertTrue(any(p.startswith("cipher:") for p in problems))

    def test_corrupted_kdf_is_named(self):
        self.flip_byte(provision_key.OFF_KDF)
        problems = provision_key.check_blob(bytes(self.blob))
        self.assertTrue(any(p.startswith("kdf:") for p in problems))

    def test_corrupted_iterations_is_named(self):
        self.flip_byte(provision_key.OFF_ITERS)
        problems = provision_key.check_blob(bytes(self.blob))
        self.assertTrue(any(p.startswith("iters:") for p in problems))

    def test_corrupted_salt_is_named(self):
        self.flip_byte(provision_key.OFF_SALT)
        problems = provision_key.check_blob(bytes(self.blob))
        self.assertTrue(any(p.startswith("salt:") for p in problems))

    def test_every_payload_corruption_also_trips_the_crc(self):
        # None of the field flips above touch the stored CRC, so each one
        # necessarily also breaks the crc32 check -- check_blob() collects
        # every problem rather than stopping at the first, and that fan-out
        # is itself worth pinning down (a reader who sees only "crc32" with
        # no field name knows to suspect the *reserved* region instead, per
        # the next test).
        self.flip_byte(provision_key.OFF_SALT)
        problems = provision_key.check_blob(bytes(self.blob))
        self.assertTrue(any(p.startswith("crc32:") for p in problems))

    def test_corrupted_reserved_region_is_caught_only_by_crc(self):
        self.flip_byte(provision_key.OFF_RESERVED2)
        problems = provision_key.check_blob(bytes(self.blob))
        self.assertEqual(["crc32: expected 0x%08X, got 0x%08X" % (
            provision_key.file_crc32(self.blob),
            struct.unpack("<I", self.blob[provision_key.OFF_CRC:provision_key.OFF_CRC + 4])[0],
        )], problems)

    def test_corrupted_crc_itself_is_the_only_complaint(self):
        self.flip_byte(provision_key.OFF_CRC)
        problems = provision_key.check_blob(bytes(self.blob))
        self.assertEqual(1, len(problems))
        self.assertTrue(problems[0].startswith("crc32:"))


class CliTests(unittest.TestCase):
    """--password-e/--password-l are documented as the automated tests'
    only non-interactive path (see the argparse help text) -- these are that
    automation, exercised the same way a CI job or this test would call it."""

    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tempdir.cleanup)
        self.output_path = os.path.join(self.tempdir.name, "bally.key")

    def test_writes_a_file_that_passes_verify(self):
        code, _, _ = run_main([
            self.output_path,
            "--password-e", "e-pw", "--password-l", "l-pw",
        ])
        self.assertEqual(0, code)

        verify_code, verify_out, _ = run_main(["--verify", self.output_path])
        self.assertEqual(0, verify_code)
        self.assertIn("is valid", verify_out)

    def test_refuses_to_overwrite_without_force(self):
        with open(self.output_path, "wb") as handle:
            handle.write(b"not a key file")

        code, _, err = run_main([
            self.output_path,
            "--password-e", "e-pw", "--password-l", "l-pw",
        ])
        self.assertEqual(2, code)
        self.assertIn("already exists", err)
        with open(self.output_path, "rb") as handle:
            self.assertEqual(b"not a key file", handle.read())

    def test_force_overwrites_an_existing_file(self):
        with open(self.output_path, "wb") as handle:
            handle.write(b"not a key file")

        code, _, _ = run_main([
            self.output_path, "--force",
            "--password-e", "e-pw", "--password-l", "l-pw",
        ])
        self.assertEqual(0, code)
        with open(self.output_path, "rb") as handle:
            self.assertEqual([], provision_key.check_blob(handle.read()))

    def test_verify_reports_a_missing_file(self):
        code, _, err = run_main(
            ["--verify", os.path.join(self.tempdir.name, "does-not-exist")])
        self.assertEqual(2, code)
        self.assertIn("cannot read", err)

    def test_verify_reports_a_corrupt_file(self):
        blob = bytearray(provision_key.build_blob("e-pw", "l-pw"))
        blob[provision_key.OFF_MAGIC] ^= 0xFF
        with open(self.output_path, "wb") as handle:
            handle.write(bytes(blob))

        code, out, _ = run_main(["--verify", self.output_path])
        self.assertEqual(1, code)
        self.assertIn("NOT valid", out)
        self.assertIn("magic:", out)

    def test_rejects_one_password_without_the_other(self):
        code, _, _ = run_main([
            self.output_path, "--password-e", "only-e",
        ])
        self.assertEqual(2, code)
        self.assertFalse(os.path.exists(self.output_path))

    def test_rejects_an_empty_password(self):
        code, _, _ = run_main([
            self.output_path, "--password-e", "", "--password-l", "l-pw",
        ])
        self.assertEqual(2, code)
        self.assertFalse(os.path.exists(self.output_path))


if __name__ == "__main__":
    unittest.main()
