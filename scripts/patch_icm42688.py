Import("env")

# lib_deps pins https://github.com/AlisonTristao/ICM42688_ESPIDF with no
# version, and upstream master never actually grew the busOpen()/scanBus()
# API that utils/BallyRobot/BallyRobot.cpp's "scan_i2c"/"test_i2c" DEBUG
# shell commands already call (see ROBOT::registerDebugCommands()) -- and
# whoAmI() has always been protected there. This re-applies the missing
# pieces to the fetched copy before every build, the same way
# patch_esp_tinyusb.py re-applies local fixes to a re-fetched
# espressif/esp_tinyusb. Only rewrites the files when a fix is missing, so
# repeated builds are no-ops.

import os
import re

LIB_DIR = os.path.join(env["PROJECT_DIR"], ".pio", "libdeps", env["PIOENV"], "ICM42688", "src")
HEADER = os.path.join(LIB_DIR, "ICM42688.h")
SOURCE = os.path.join(LIB_DIR, "ICM42688.cpp")

MARKER = "bool scanBus(std::string& found);"

STRING_INCLUDE_ANCHOR = '#include "driver/i2c_master.h"'
STRING_INCLUDE = STRING_INCLUDE_ANCHOR + "\n#include <string>  // scanBus() (see patch_icm42688.py)"

# whoAmI() is only ever called from within the class in upstream master, so
# it stays protected there; BallyRobot.cpp calls it directly on ROBOT's imu
# member, so it has to move to public. Only the declaration line is
# relocated -- its doc comment is harmless left behind in the protected
# block.
WHOAMI_DECL_PATTERN = re.compile(r"^[ \t]*uint8_t whoAmI\(\);[ \t]*\n", re.MULTILINE)

NEW_PUBLIC_METHODS = """\t/**
\t * @brief      True once begin() has created the I2C bus (see
\t * initI2CBus()), even if the WHO_AM_I check right after that in begin()
\t * failed -- lets a caller reuse this object's already-open bus (e.g. to
\t * rescan for devices) instead of opening a second i2c_master_bus on the
\t * same pins.
\t */
\tbool busOpen() const { return _bus_handle != nullptr; }

\t/**
\t * @brief      Probe every 7-bit address on the already-open bus (see
\t * busOpen()) and collect the ones that ACK.
\t * @param      found  Receives "0xNN " for every address that answered;
\t * cleared first. Left untouched on a false return.
\t * @return     true if at least one address ACKed.
\t */
\tbool scanBus(std::string& found);

\t/**
\t * @brief      Read the WHO_AM_I register.
\t * @return     Value of the WHO_AM_I register.
\t */
\tuint8_t whoAmI();

"""

PROTECTED_ANCHOR = "\n protected:\n"

SCANBUS_IMPL_ANCHOR = "/* get Raw Bias (Offsets)*/"
SCANBUS_IMPL = """/* probes every 7-bit address on the bus begin() already opened, without
 * touching _dev_handle (which is only bound to _address) -- see
 * ICM42688::busOpen()/scanBus() in ICM42688.h */
bool ICM42688::scanBus(std::string& found) {
\tfound.clear();
\tif (_bus_handle == nullptr) {
\t\treturn false;
\t}

\tbool any_found = false;
\tchar hex[6];
\tfor (uint16_t address = 0x08; address <= 0x77; ++address) {
\t\tif (i2c_master_probe(_bus_handle, static_cast<uint8_t>(address), I2C_TIMEOUT_MS) != ESP_OK) {
\t\t\tcontinue;
\t\t}
\t\tsnprintf(hex, sizeof(hex), "0x%02X ", static_cast<unsigned>(address));
\t\tfound += hex;
\t\tany_found = true;
\t}
\treturn any_found;
}

""" + SCANBUS_IMPL_ANCHOR

CSTDIO_INCLUDE_ANCHOR = '#include "ICM42688.h"'
CSTDIO_INCLUDE = CSTDIO_INCLUDE_ANCHOR + "\n#include <cstdio>  // scanBus()'s snprintf (see patch_icm42688.py)"


def apply_header_patch():
    if not os.path.isfile(HEADER):
        return False

    with open(HEADER, "r", encoding="utf-8") as f:
        content = f.read()

    if MARKER in content:
        return False

    if STRING_INCLUDE_ANCHOR not in content:
        raise SystemExit(
            "patch_icm42688: driver/i2c_master.h include anchor not found in "
            "ICM42688.h -- upstream likely changed, update scripts/patch_icm42688.py"
        )
    content = content.replace(STRING_INCLUDE_ANCHOR, STRING_INCLUDE, 1)

    if not WHOAMI_DECL_PATTERN.search(content):
        raise SystemExit(
            "patch_icm42688: 'uint8_t whoAmI();' declaration not found in "
            "ICM42688.h -- upstream likely changed, update scripts/patch_icm42688.py"
        )
    content = WHOAMI_DECL_PATTERN.sub("", content, count=1)

    if PROTECTED_ANCHOR not in content:
        raise SystemExit(
            "patch_icm42688: ' protected:' anchor not found in ICM42688.h -- "
            "upstream likely changed, update scripts/patch_icm42688.py"
        )
    content = content.replace(PROTECTED_ANCHOR, "\n" + NEW_PUBLIC_METHODS + " protected:\n", 1)

    with open(HEADER, "w", encoding="utf-8") as f:
        f.write(content)
    return True


def apply_source_patch():
    if not os.path.isfile(SOURCE):
        return False

    with open(SOURCE, "r", encoding="utf-8") as f:
        content = f.read()

    if "ICM42688::scanBus(" in content:
        return False

    if CSTDIO_INCLUDE_ANCHOR not in content:
        raise SystemExit(
            "patch_icm42688: ICM42688.h include anchor not found in "
            "ICM42688.cpp -- upstream likely changed, update scripts/patch_icm42688.py"
        )
    content = content.replace(CSTDIO_INCLUDE_ANCHOR, CSTDIO_INCLUDE, 1)

    if SCANBUS_IMPL_ANCHOR not in content:
        raise SystemExit(
            "patch_icm42688: computeOffsets() anchor comment not found in "
            "ICM42688.cpp -- upstream likely changed, update scripts/patch_icm42688.py"
        )
    content = content.replace(SCANBUS_IMPL_ANCHOR, SCANBUS_IMPL, 1)

    with open(SOURCE, "w", encoding="utf-8") as f:
        f.write(content)
    return True


def apply_patch():
    if not os.path.isdir(LIB_DIR):
        print("patch_icm42688: ICM42688 not fetched yet, skipping (will patch on next build)")
        return

    header_changed = apply_header_patch()
    source_changed = apply_source_patch()
    if header_changed or source_changed:
        print("patch_icm42688: applied busOpen()/scanBus()/public whoAmI() to ICM42688")


apply_patch()
