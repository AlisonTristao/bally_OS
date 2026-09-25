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
# v2.40.0 -- v2.0.0 + the PSA backend + btp::messages (2.2.0) + the manifest
# verbatim relay (2.3.0) + btp::telemetry, the PACKED_LE / TLV_LE sample codec
# (2.4.0) + its body-only mode (2.5.0) + btp::session, the btp::DedupCache
# command-deduplication cache (2.6.0) + btp::endpoint, the identity /
# sequencing / seal-fragment-encode transmit pipeline BtpEndpoint derives from
# (2.7.0) + btp::receiver, the decode / CRC / reassembly receive pipeline
# node_ now owns directly (2.8.0; RxRouter, the old thin wrapper over it, is
# gone) + btp::catalog (2.12.0+source_info 2.35.0+body-only topics 2.39.0),
# which node_->serve_catalog() now answers CONTROL/MANIFEST_REQUEST from
# directly (ManifestCatalog::populate() is the one-time load, see
# BallyRobot.h's node_ comment -- ManifestResponder, which used to hand-encode
# MANIFEST_DATA itself, is gone) + btp::subscription's rate-policy/reboot-
# eviction/introspection additions (2.40.0, not adopted here yet -- see
# SubscriptionResponder's own class comment for why: TelemetryPublisher is
# still the one table for the wire AND the publish queue/stats). node_ also
# answers TERMINAL_IN directly now (RobotLink::terminal()) but still leaves
# CommandProcessor's async task model alone -- Node's NodeActionFn is
# synchronous only (see its own "first cut" comment in btp/node.hpp).
# ACTUAL minimum today is v2.45.0, past this comment's last update: some
# TelemetryPublisher::FieldSchema fields (current_a/current_b, pwm_left/
# pwm_right) now declare a min_value/max_value range, which
# ManifestCatalog::populate() forwards onto btp::FieldRecord::min_value/
# max_value -- a field only that version added, gated behind
# manifest_format_version 3 (btp::Node::emit_manifest() picks it automatically
# via btp::Catalog::has_field_ranges(), no code here depends on the format
# number directly). v2.44.0 first added it but wrote min_value/max_value for
# EVERY field once format 3 was used, even ones with nothing to declare --
# +16 bytes/field, which alone ate nearly all of this robot's fixed 1900-byte
# manifest scratch buffer; v2.45.0 made it per-field opt-in (flags bit
# kFieldHasRange, derived from whether a range is actually set).
# Since then the minimum is v2.46.0: RobotLink / RobotTcpLink / RobotBleLink
# all have has_open() true, so node_ / tcp_node_ / ble_node_ now DROP any
# unsealed message past the handshake (NodeConfig::accept_cleartext(),
# default false) -- the robot needs TraceView >= 4.6.4 on a direct TCP/BLE
# session, which seals everything it sends. 2.46.0 also fragments every
# send to <= 250-octet frames, which is what lets tcp_node_ / ble_node_ send
# the ~1.7 KB MANIFEST_DATA at all (it used to fail as one oversized frame).
# Keep this in step with [env:native]'s symlink; this can become a submodule
# pinned to the tag.
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
        "${btp_root}/src/session.cpp"
        "${btp_root}/src/endpoint.cpp"
        "${btp_root}/src/receiver.cpp"
        "${btp_root}/src/catalog.cpp"
        "${btp_root}/src/subscription.cpp"
        "${btp_root}/src/node.cpp"
        "${btp_root}/src/aead.cpp"
    INCLUDE_DIRS "${btp_root}/include"
    REQUIRES mbedtls)
