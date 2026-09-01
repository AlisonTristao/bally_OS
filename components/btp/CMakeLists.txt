# BTP as an ESP-IDF component, NOT a PlatformIO lib_deps entry.
#
# Why: under `framework = espidf` PlatformIO compiles every lib_deps library
# with its own LDF-driven builder, OUTSIDE the ESP-IDF CMake component graph.
# A library built that way never receives the mbedtls / PSA include paths, so
# BTP/src/aead.cpp -- which is #if-guarded on __has_include(<psa/crypto.h>) /
# <mbedtls/gcm.h> -- compiles to nothing and every RadioSeal call comes out as
# `undefined reference to btp::aead_seal_aes_gcm` at link time. (The dongle
# escapes this only because it is `framework = arduino`, where the Arduino SDK
# puts mbedtls on the global include path.)
#
# Registering BTP here, with REQUIRES mbedtls, is what puts <psa/crypto.h> on
# aead.cpp's include path. This ESP-IDF is mbedtls 4.x / TF-PSA-Crypto, where
# <mbedtls/gcm.h> moved to mbedtls/private/ -- so only the PSA backend links.
# See the sibling checkout's src/aead.cpp "Backend selection" comment.
#
# Sources come from the sibling checkout c:/git/BTP -- the SAME tree
# [env:native] builds through `symlink://../BTP` -- so the firmware and the
# host test suite can never diverge on the protocol. Expected revision:
# v2.5.0 -- v2.0.0 + the PSA backend + btp::messages (2.2.0) + the manifest
# verbatim relay (2.3.0) + btp::telemetry, the PACKED_LE / TLV_LE sample codec
# (2.4.0) + its body-only mode (2.5.0). Keep this in step with [env:native]'s
# symlink; this can become a submodule pinned to the tag.
#
# The btp lib_deps entry is removed from [env:esp32-s3] for the same reason a
# component and a lib_deps library of the same name would fight over codec.o.

set(btp_root "${CMAKE_CURRENT_LIST_DIR}/../../../BTP")

if(NOT EXISTS "${btp_root}/src/aead.cpp")
    message(FATAL_ERROR
        "BTP source not found at ${btp_root}. bally_OS expects the BTP repo "
        "checked out as a sibling directory (../BTP), the same as "
        "[env:native]'s symlink://../BTP.")
endif()

idf_component_register(
    SRCS
        "${btp_root}/src/codec.cpp"
        "${btp_root}/src/fragmentation.cpp"
        "${btp_root}/src/stream.cpp"
        "${btp_root}/src/messages.cpp"
        "${btp_root}/src/telemetry.cpp"
        "${btp_root}/src/aead.cpp"
    INCLUDE_DIRS "${btp_root}/include"
    REQUIRES mbedtls)
