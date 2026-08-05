Import("env")

# managed_components/ is gitignored: the ESP-IDF component manager owns it and
# can silently re-fetch a pristine copy of espressif/esp_tinyusb (component
# version bump, lock file mismatch, or a full re-resolution triggered by an
# unrelated dependency change). That wipes the local fixes below. Re-apply
# them here, before every build, so a fresh fetch can never silently drop
# them. This only rewrites the file when a fix is missing, so repeated builds
# are no-ops.
#
# Note: on a completely fresh clone, managed_components/ does not exist yet
# on the very first "pio run" (the component manager fetches it during that
# same build, after this script already ran). Building a second time applies
# the patch normally.

import os

TARGET = os.path.join(
    env["PROJECT_DIR"], "managed_components", "espressif__esp_tinyusb", "tinyusb_msc.c"
)

NFAT_OLD = "const MKFS_PARM opt = {format_flags, 0, 0, 0, alloc_unit_size};"
NFAT_NEW = """// n_fat=0 makes FatFs's f_mkfs() default to a single FAT copy instead of
    // the usual redundant pair (see ff.c: n_fat = (opt->n_fat >= 1 && <= 2) ?
    // opt->n_fat : 1). A single-FAT FAT32 volume is valid per spec but
    // unusual enough that it is worth avoiding for host compatibility.
    const MKFS_PARM opt = {format_flags, 2, 0, 0, alloc_unit_size};"""

CAP16_MARKER = "SCSI_CMD_SERVICE_ACTION_IN_16"

CAP16_DEFINES_ANCHOR = (
    "// Invoked when received GET_MAX_LUN request, "
    "required for multiple LUNs implementation"
)
CAP16_DEFINES = """/* Not part of TinyUSB's msc.h. Modern host storage stacks can issue this to
 * read a 64-bit capacity (needed once a device exceeds the 32-bit LBA range
 * READ CAPACITY(10) supports); left unhandled it falls to the default STALL
 * below. */
#define SCSI_CMD_SERVICE_ACTION_IN_16                   0x9E /** SERVICE ACTION IN(16) opcode **/
#define SCSI_SAI_READ_CAPACITY_16                       0x10 /** Service action requesting READ CAPACITY(16) **/

""" + CAP16_DEFINES_ANCHOR

CAP16_CASE_ANCHOR = """    default:
        ESP_LOGW(TAG, "tud_msc_scsi_cb() invoked: %d", scsi_cmd[0]);"""
CAP16_CASE = """    case SCSI_CMD_SERVICE_ACTION_IN_16:
        if ((scsi_cmd[1] & 0x1F) == SCSI_SAI_READ_CAPACITY_16) {
            msc_storage_obj_t *storage = NULL;

            MSC_ENTER_CRITICAL();
            bool found = _msc_storage_get_by_lun(lun, &storage);
            MSC_EXIT_CRITICAL();

            uint32_t capacity = 0;
            uint32_t sec_size = 0;
            if (found && storage != NULL) {
                tinyusb_msc_get_storage_capacity((tinyusb_msc_storage_handle_t) storage, &capacity);
                tinyusb_msc_get_storage_sector_size((tinyusb_msc_storage_handle_t) storage, &sec_size);
            }

            if (capacity == 0 || sec_size == 0) {
                tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, SCSI_CODE_ASC_MEDIUM_NOT_PRESENT, SCSI_CODE_ASCQ);
                ret = -1;
                break;
            }

            // READ CAPACITY(16) response: 8-byte last LBA, 4-byte block size,
            // then 20 bytes of protection/alignment/provisioning info that are
            // all valid left as zero for a plain, unprotected, non-thin volume.
            uint8_t resp[32] = {0};
            uint64_t last_lba = (uint64_t)capacity - 1;
            for (int i = 0; i < 8; i++) {
                resp[i] = (uint8_t)(last_lba >> (8 * (7 - i)));
            }
            resp[8]  = (uint8_t)(sec_size >> 24);
            resp[9]  = (uint8_t)(sec_size >> 16);
            resp[10] = (uint8_t)(sec_size >> 8);
            resp[11] = (uint8_t)(sec_size);

            ret = (int32_t)(bufsize < sizeof(resp) ? bufsize : sizeof(resp));
            memcpy(buffer, resp, (size_t) ret);
        } else {
            ESP_LOGW(TAG, "tud_msc_scsi_cb() unsupported SERVICE ACTION IN(16) action: 0x%02x", scsi_cmd[1] & 0x1F);
            tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, SCSI_CODE_ASC_INVALID_COMMAND_OPERATION_CODE, SCSI_CODE_ASCQ);
            ret = -1;
        }
        break;
""" + CAP16_CASE_ANCHOR


def apply_patch():
    if not os.path.isfile(TARGET):
        print("patch_esp_tinyusb: managed_components not fetched yet, skipping (will patch on next build)")
        return

    with open(TARGET, "r", encoding="utf-8") as f:
        content = f.read()

    changed = False

    if NFAT_NEW not in content:
        if NFAT_OLD not in content:
            raise SystemExit(
                "patch_esp_tinyusb: n_fat anchor not found in tinyusb_msc.c -- "
                "espressif/esp_tinyusb likely changed upstream, update scripts/patch_esp_tinyusb.py"
            )
        content = content.replace(NFAT_OLD, NFAT_NEW, 1)
        changed = True

    if CAP16_MARKER not in content:
        if CAP16_DEFINES_ANCHOR not in content or CAP16_CASE_ANCHOR not in content:
            raise SystemExit(
                "patch_esp_tinyusb: SERVICE ACTION IN(16) anchors not found in tinyusb_msc.c -- "
                "espressif/esp_tinyusb likely changed upstream, update scripts/patch_esp_tinyusb.py"
            )
        content = content.replace(CAP16_DEFINES_ANCHOR, CAP16_DEFINES, 1)
        content = content.replace(CAP16_CASE_ANCHOR, CAP16_CASE, 1)
        changed = True

    if changed:
        with open(TARGET, "w", encoding="utf-8") as f:
            f.write(content)
        print("patch_esp_tinyusb: applied local fixes to tinyusb_msc.c")


apply_patch()
