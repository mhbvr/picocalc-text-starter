//
// diskio.c - FatFS disk I/O adapter for PicoCalc SD card
//

#include "ff.h"
#include "diskio.h"
#include "sd_card.h"
#include "sdfs.h"
#include "pico/stdlib.h"

FATFS sdfs_volume;
static bool mounted = false;
static repeating_timer_t detect_timer;

// ---------------------------------------------------------------------------
// FatFS disk I/O interface
// ---------------------------------------------------------------------------

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;
    return sd_card_init() == SD_ERR_NONE ? 0 : STA_NOINIT;
}

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;
    if (!sd_card_present()) return STA_NODISK;
    return 0;
}

DRESULT disk_read(BYTE pdrv, BYTE *buf, LBA_t sector, UINT count)
{
    if (pdrv != 0) return RES_PARERR;
    // Use sd_read_blocks() for any count: issues a single CMD18 when count > 1
    return sd_read_blocks(sector, count, buf) == SD_ERR_NONE ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buf, LBA_t sector, UINT count)
{
    if (pdrv != 0) return RES_PARERR;
    // Use sd_write_blocks() for any count: issues a single CMD25 when count > 1
    return sd_write_blocks(sector, count, buf) == SD_ERR_NONE ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buf)
{
    if (pdrv != 0) return RES_PARERR;
    switch (cmd) {
        case CTRL_SYNC:       return RES_OK;
        case GET_SECTOR_SIZE: *(WORD  *)buf = 512; return RES_OK;
        case GET_BLOCK_SIZE:  *(DWORD *)buf = 1;   return RES_OK;
        case GET_SECTOR_COUNT: {
            uint32_t n;
            if (sd_get_sector_count(&n) != SD_ERR_NONE) return RES_ERROR;
            *(LBA_t *)buf = n;
            return RES_OK;
        }
        default:              return RES_PARERR;
    }
}

// ---------------------------------------------------------------------------
// Mount management and hot-plug
// ---------------------------------------------------------------------------

bool sdfs_is_ready(void)
{
    if (sd_card_present() && !mounted) {
        if (f_mount(&sdfs_volume, "", 1) == FR_OK) {
            mounted = true;
        }
    } else if (!sd_card_present() && mounted) {
        f_mount(NULL, "", 0);
        mounted = false;
    }
    return mounted;
}

static bool sdfs_detect_callback(repeating_timer_t *rt)
{
    (void)rt;
    sdfs_is_ready();
    return true; // keep repeating
}

void sdfs_init(void)
{
    sd_init(); // GPIO/SPI setup

    // Poll every 500 ms for card insertion/removal
    add_repeating_timer_ms(-500, sdfs_detect_callback, NULL, &detect_timer);
}
