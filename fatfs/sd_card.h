#pragma once
#include <stdbool.h>
#include <stdint.h>

/* ── Hardware configuration ──────────────────────────────────────── */
#define SD_SPI        spi0
#define SD_MISO       16
#define SD_CS         17
#define SD_SCK        18
#define SD_MOSI       19
#define SD_DETECT     22
#define SD_INIT_BAUD  400000     // 400 kHz — required by spec during card init
#define SD_FAST_BAUD  25000000   // 25 MHz — normal operation

/* ── Driver tunables ─────────────────────────────────────────────── */
#define SD_CRC_ENABLED      1    // 1 = verify CRC on commands & data blocks
#define SD_READ_RETRIES     3    // retry count for transient read errors
#define SD_CMD_RETRIES      10   // CMD0 retry count during power-on
#define SD_INIT_TIMEOUT_MS  1000 // ACMD41 loop timeout
#define SD_READ_TIMEOUT_MS  100  // wait-for-data-token timeout
#define SD_WRITE_TIMEOUT_MS 500  // wait-for-write-complete timeout

/* ── Error codes returned by sd_last_error() ─────────────────────── */
typedef enum {
    SD_ERR_NONE = 0,
    SD_ERR_NO_CARD,       // card detect pin not asserted
    SD_ERR_TIMEOUT,       // operation timed out
    SD_ERR_CMD,           // R1 response had error bits set
    SD_ERR_CRC_CMD,       // CRC error on command response (R1 bit 3)
    SD_ERR_CRC_DATA,      // CRC16 mismatch on read data block
    SD_ERR_DATA_TOKEN,    // unexpected or error data token
    SD_ERR_WRITE_REJECT,  // card rejected write data (CRC or write error)
    SD_ERR_CARD,          // card internal error (error token bit)
    SD_ERR_PARAM,         // invalid parameter (address out of range)
} sd_error_t;

/* ── Public API ──────────────────────────────────────────────────── */
void         sd_init(void);
bool         sd_card_present(void);
bool         sd_is_sdhc(void);
bool         sd_card_init(void);
bool         sd_read_block(uint32_t sector, uint8_t *buf);
bool         sd_write_block(uint32_t sector, const uint8_t *buf);
bool         sd_read_blocks(uint32_t sector, uint32_t count, uint8_t *buf);
bool         sd_write_blocks(uint32_t sector, uint32_t count, const uint8_t *buf);
bool         sd_get_sector_count(uint32_t *count);
sd_error_t   sd_last_error(void);
