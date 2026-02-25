//
// sd_card.c - Production-ready SPI SD card driver for PicoCalc
//
// Implements SPI Mode (CPOL=0, CPHA=0) per Physical Layer Simplified Spec v9.00.
// Supports SDSC (v1 & v2) and SDHC/SDXC. Features:
//   - CRC7 on command packets, CRC16-CCITT on data blocks (SD_CRC_ENABLED)
//   - CMD18 multi-block read, CMD25 multi-block write
//   - CSD register parsing for card capacity
//   - Structured error reporting via sd_last_error()
//

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "sd_card.h"

// ---------------------------------------------------------------------------
// Command numbers (§4.7.4 Detailed Command Description)
// ---------------------------------------------------------------------------
#define CMD0   0    // GO_IDLE_STATE        — reset to idle / SPI mode
#define CMD8   8    // SEND_IF_COND         — voltage check (SDv2 detection)
#define CMD9   9    // SEND_CSD             — read card-specific data register
#define CMD12  12   // STOP_TRANSMISSION    — end CMD18 multi-block read
#define CMD16  16   // SET_BLOCKLEN         — set block length for SDSC
#define CMD17  17   // READ_SINGLE_BLOCK
#define CMD18  18   // READ_MULTIPLE_BLOCK
#define CMD24  24   // WRITE_BLOCK
#define CMD25  25   // WRITE_MULTIPLE_BLOCK
#define CMD55  55   // APP_CMD             — prefix for ACMD
#define CMD58  58   // READ_OCR            — read operation conditions register
#define CMD59  59   // CRC_ON_OFF          — enable/disable CRC checking
#define ACMD23 23   // SET_WR_BLK_ERASE_COUNT — pre-erase hint before CMD25
#define ACMD41 41   // SD_SEND_OP_COND     — card init, report power-up status

// ---------------------------------------------------------------------------
// R1 response bit masks (§7.3.2.1)
// Bit set = error, except IDLE which is expected during card init.
// ---------------------------------------------------------------------------
#define R1_IDLE        0x01  // in idle state (normal during init only)
#define R1_ERASE_RESET 0x02  // erase sequence was cleared
#define R1_ILLEGAL_CMD 0x04  // illegal command received
#define R1_CRC_ERROR   0x08  // CRC check of last command failed
#define R1_ERASE_SEQ   0x10  // error in erase sequence
#define R1_ADDR_ERROR  0x20  // address misaligned
#define R1_PARAM_ERROR 0x40  // parameter out of allowed range
#define R1_ERROR_MASK  0xFE  // any bit other than IDLE is a hard error

// ---------------------------------------------------------------------------
// Data tokens (§7.3.3)
// ---------------------------------------------------------------------------
#define DATA_START_SINGLE 0xFE  // start token for CMD17/CMD24 and each CMD18 block
#define DATA_START_MULTI  0xFC  // start token for each CMD25 write block
#define DATA_STOP_TRAN    0xFD  // stop transmission token to end CMD25

// Data response token (card response to write data block, §7.3.3.1)
#define DATA_RESP_MASK     0x1F
#define DATA_RESP_ACCEPTED 0x05  // lower 5 bits = 0b00101 — data accepted
#define DATA_RESP_CRC_ERR  0x0B  // lower 5 bits = 0b01011 — CRC error
#define DATA_RESP_WR_ERR   0x0D  // lower 5 bits = 0b01101 — write error

// Error token bits (returned instead of DATA_START_SINGLE on read error, §7.3.3.2)
#define ERR_TOKEN_GENERAL  0x01
#define ERR_TOKEN_CC_ERR   0x02
#define ERR_TOKEN_ECC_FAIL 0x04
#define ERR_TOKEN_OOR      0x08  // out of range / card ECC failed

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
static bool       is_sdhc         = false;
static bool       sd_gpio_init_done = false;
static sd_error_t last_error      = SD_ERR_NONE;

// ---------------------------------------------------------------------------
// CRC functions
// ---------------------------------------------------------------------------

/*
 * crc7() — 7-bit CRC for SD command packets (§4.5 CRC).
 * Polynomial: x^7 + x^3 + 1 (0x09 in normal representation).
 * The caller appends the result as (crc7(...) << 1) | 0x01.
 * Bit-by-bit: commands are only 5 bytes so a lookup table is not worth it.
 */
static uint8_t crc7(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (int bit = 7; bit >= 0; bit--) {
            crc <<= 1;
            if (((b >> bit) & 1) ^ ((crc >> 7) & 1))
                crc ^= 0x09;
        }
    }
    return crc & 0x7F;
}

/*
 * CRC-16/CCITT lookup table — polynomial 0x1021.
 * 512 bytes of flash; reduces per-512-byte-block computation from ~4096 bit
 * operations to 512 table lookups (~8 µs vs. ~164 µs at 25 MHz SPI).
 */
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

/*
 * crc16_ccitt() — 16-bit CRC for 512-byte data blocks (§4.5 CRC).
 * Polynomial: x^16 + x^12 + x^5 + 1 (0x1021).
 * Returns the CRC as a big-endian uint16_t ready to be sent MSB-first.
 */
static uint16_t crc16_ccitt(const uint8_t *data, int len)
{
    uint16_t crc = 0;
    for (int i = 0; i < len; i++)
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    return crc;
}

// ---------------------------------------------------------------------------
// Low-level SPI helpers
// ---------------------------------------------------------------------------

static uint8_t spi_xfer(uint8_t data)
{
    uint8_t result;
    spi_write_read_blocking(SD_SPI, &data, &result, 1);
    return result;
}

/*
 * wait_ready() — poll MISO until card returns 0xFF (not busy).
 * Used after write commands (Nwr / Nac per §7.5.4 and §7.5.6).
 * CS is permanently asserted after sd_card_init(), so no select needed.
 */
static bool wait_ready(uint32_t ms)
{
    absolute_time_t deadline = make_timeout_time_ms(ms);
    while (!time_reached(deadline)) {
        if (spi_xfer(0xFF) == 0xFF) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// SD command engine (§7.3.1 — Command Format)
// ---------------------------------------------------------------------------

/*
 * sd_cmd() — send a 6-byte command and return the R1 response byte.
 *
 * Command packet layout (§7.3.1.1):
 *   byte 0: start bit(0) + tx bit(1) + cmd[5:0]
 *   byte 1-4: 32-bit argument, MSB first
 *   byte 5: CRC7[6:0] + end bit(1)
 *
 * If SD_CRC_ENABLED, CRC7 is computed over bytes 0-4.
 * If not, 0xFF is sent as a dummy CRC byte (cards accept this once CMD59(0)
 * leaves the default CRC-off state, or if CMD59 was never sent).
 *
 * Ncr (response latency) is at most 8 bytes per spec §7.5.1.
 */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t packet[6];
    packet[0] = 0x40 | cmd;
    packet[1] = (arg >> 24) & 0xFF;
    packet[2] = (arg >> 16) & 0xFF;
    packet[3] = (arg >>  8) & 0xFF;
    packet[4] =  arg        & 0xFF;

#if SD_CRC_ENABLED
    packet[5] = (crc7(packet, 5) << 1) | 0x01;
#else
    // Hardcode known CRCs for CMD0 and CMD8; 0xFF (ignore CRC) for the rest.
    // Cards will accept 0xFF when CRC mode is off (the default after power-on).
    if (cmd == CMD0)      packet[5] = 0x95;
    else if (cmd == CMD8) packet[5] = 0x87;
    else                  packet[5] = 0xFF;
#endif

    spi_write_blocking(SD_SPI, packet, 6);

    // Poll for R1 — MSB clear means valid response; Ncr max = 8 bytes (§7.5.1)
    uint8_t r = 0xFF;
    for (int i = 0; i < 8; i++) {
        r = spi_xfer(0xFF);
        if (!(r & 0x80)) break;
    }

    if (r & R1_CRC_ERROR)
        last_error = SD_ERR_CRC_CMD;

    return r;
}

/*
 * sd_read_r3r7() — read the 4 trailing bytes of an R3 or R7 response.
 * Must be called immediately after sd_cmd() returned R1 == 0x01 for
 * CMD58 (READ_OCR → R3) or CMD8 (SEND_IF_COND → R7).
 */
static void sd_read_r3r7(uint8_t out[4])
{
    for (int i = 0; i < 4; i++)
        out[i] = spi_xfer(0xFF);
}

// ---------------------------------------------------------------------------
// Public API — initialisation
// ---------------------------------------------------------------------------

void sd_init(void)
{
    if (sd_gpio_init_done) return;

    gpio_init(SD_MISO);
    gpio_init(SD_CS);
    gpio_init(SD_SCK);
    gpio_init(SD_MOSI);
    gpio_init(SD_DETECT);

    gpio_set_dir(SD_CS, GPIO_OUT);
    gpio_put(SD_CS, 1);         // CS idle-high
    gpio_set_dir(SD_DETECT, GPIO_IN);
    gpio_pull_up(SD_DETECT);   // active-low detect — pulled high when absent

    gpio_set_function(SD_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SD_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(SD_MOSI, GPIO_FUNC_SPI);

    sd_gpio_init_done = true;
}

bool sd_card_present(void)
{
    return !gpio_get(SD_DETECT); // active-low
}

bool sd_is_sdhc(void)
{
    return is_sdhc;
}

/*
 * sd_card_init() — full SD SPI-mode initialisation sequence (§7.2.1).
 *
 * CS is asserted (low) once after the 80 dummy clocks and held low for the
 * remainder of the session.  No select/deselect is performed between commands;
 * a single dummy byte (spi_xfer(0xFF)) is clocked after each response to
 * satisfy the Ncs inter-command gap required by §7.5.1.
 *
 * Steps:
 *  1. Check card present
 *  2. spi_init at 400 kHz + 10 ms power-up delay
 *  3. CS HIGH + 80 dummy clocks (≥74 required by spec)
 *  4. Assert CS LOW — held for the rest of the session
 *  5. CMD0  — GO_IDLE_STATE (reset; up to SD_CMD_RETRIES)
 *  6. CMD8  — SEND_IF_COND (SDv2 detection, 0x1AA pattern)
 *  7. CMD59 — CRC_ON_OFF(1) if SD_CRC_ENABLED
 *  8. CMD58 — READ_OCR (verify 3.3V support)
 *  9. ACMD41 loop — wait for card to leave idle (SD_INIT_TIMEOUT_MS)
 * 10. CMD58 again — read CCS bit to determine SDHC vs SDSC
 * 11. CMD16(512) — set block length for SDSC cards
 * 12. Switch SPI to SD_FAST_BAUD
 */
bool sd_card_init(void)
{
    last_error = SD_ERR_NONE;

    if (!sd_card_present()) {
        last_error = SD_ERR_NO_CARD;
        return false;
    }

    spi_init(SD_SPI, SD_INIT_BAUD);
    gpio_put(SD_CS, 1);              // CS HIGH for power-up phase
    busy_wait_us(10000);             // ≥1 ms power-up delay; 10 ms is conservative

    // ≥74 dummy clocks with CS deasserted to put card into native command state
    for (int i = 0; i < 10; i++) spi_xfer(0xFF); // 10 × 8 = 80 clocks

    gpio_put(SD_CS, 0);              // Assert CS — held low for the rest of the session
    busy_wait_us(1000);

    // CMD0 — reset to idle / enter SPI mode (expect R1 = 0x01)
    uint8_t r = 0;
    for (int attempt = 0; attempt < SD_CMD_RETRIES; attempt++) {
        r = sd_cmd(CMD0, 0);
        spi_xfer(0xFF);              // Ncs gap
        if (r == R1_IDLE) break;
        busy_wait_us(10000);
    }
    if (r != R1_IDLE) {
        last_error = SD_ERR_TIMEOUT;
        return false;
    }

    // CMD8 — SEND_IF_COND (§4.3.13): detect SDv2 by echo of 0x01AA
    // R7 response: [R1][voltage][echo_voltage][echo_check][echo_arg]
    bool is_v2 = false;
    r = sd_cmd(CMD8, 0x1AA); // VHS = 0x01 (2.7–3.6V), check pattern 0xAA
    if (r == R1_IDLE) {
        uint8_t r7[4];
        sd_read_r3r7(r7);
        spi_xfer(0xFF);              // Ncs gap
        // r7[2] bits[3:0] = voltage accepted (must be 0x01); r7[3] = echo
        if ((r7[2] & 0x0F) == 0x01 && r7[3] == 0xAA)
            is_v2 = true;
    } else {
        spi_xfer(0xFF);              // SDv1 or MMC — CMD8 is illegal, Ncs gap
    }

#if SD_CRC_ENABLED
    // CMD59(1) — enable CRC checking on the card side (§7.2.2)
    r = sd_cmd(CMD59, 1);
    spi_xfer(0xFF);                  // Ncs gap
    // Non-fatal: some cards don't support CMD59; continue regardless
#endif

    // CMD58 — read OCR; verify 3.3V operating range (bit 20, §5.1 Table 5-1)
    r = sd_cmd(CMD58, 0);
    uint8_t ocr[4] = {0};
    sd_read_r3r7(ocr);
    spi_xfer(0xFF);                  // Ncs gap
    if (r & R1_ERROR_MASK) {
        last_error = SD_ERR_CMD;
        return false;
    }
    // ocr[1] bit 4 = 3.2–3.3V; bit 5 = 3.3–3.4V (typical 3.3V cards set both)
    if (!(ocr[1] & 0x30)) {
        last_error = SD_ERR_CMD;
        return false;
    }

    // ACMD41 loop — SD_SEND_OP_COND: wait until card reports power-up complete
    // CMD55 (APP_CMD) must precede every ACMD (§4.3.9)
    absolute_time_t deadline = make_timeout_time_ms(SD_INIT_TIMEOUT_MS);
    do {
        r = sd_cmd(CMD55, 0);
        spi_xfer(0xFF);              // Ncs gap
        if (r & R1_ERROR_MASK) {
            last_error = SD_ERR_CMD;
            return false;
        }
        uint32_t hcs = is_v2 ? (1u << 30) : 0; // HCS bit — request SDHC if v2
        r = sd_cmd(ACMD41, hcs);
        spi_xfer(0xFF);              // Ncs gap
        if (r == 0) break; // idle bit clear = card ready
        busy_wait_us(1000);
    } while (!time_reached(deadline));

    if (r != 0) {
        last_error = SD_ERR_TIMEOUT;
        return false;
    }

    // CMD58 — re-read OCR to check CCS bit (bit 30) for SDHC vs SDSC
    r = sd_cmd(CMD58, 0);
    if (r != 0) {
        spi_xfer(0xFF);
        last_error = SD_ERR_CMD;
        return false;
    }
    sd_read_r3r7(ocr);
    spi_xfer(0xFF);                  // Ncs gap
    // ocr[0] bit 6 = CCS: 1 = SDHC/SDXC (block-addressed), 0 = SDSC (byte-addressed)
    is_sdhc = (ocr[0] & 0x40) != 0;

    // CMD16 — set block length to 512 bytes for SDSC (SDHC is always 512)
    if (!is_sdhc) {
        r = sd_cmd(CMD16, 512);
        spi_xfer(0xFF);              // Ncs gap
        if (r != 0) {
            last_error = SD_ERR_CMD;
            return false;
        }
    }

    spi_set_baudrate(SD_SPI, SD_FAST_BAUD);
    return true;
}

// ---------------------------------------------------------------------------
// Internal: wait for data start token with timeout
// Returns true if DATA_START_SINGLE (0xFE) was received.
// On error token (0x01–0x0F) sets last_error and returns false.
// On timeout sets SD_ERR_TIMEOUT and returns false.
// ---------------------------------------------------------------------------
static bool wait_for_data_token(uint32_t ms)
{
    absolute_time_t deadline = make_timeout_time_ms(ms);
    while (!time_reached(deadline)) {
        uint8_t tok = spi_xfer(0xFF);
        if (tok == DATA_START_SINGLE) return true;
        if (tok != 0xFF) {
            // Error token: bits 0-3 carry specific error bits (§7.3.3.2)
            if (tok & ERR_TOKEN_OOR)
                last_error = SD_ERR_PARAM;
            else
                last_error = SD_ERR_CARD;
            return false;
        }
    }
    last_error = SD_ERR_TIMEOUT;
    return false;
}

// ---------------------------------------------------------------------------
// Public API — single-block read (CMD17, §7.5.3)
// ---------------------------------------------------------------------------

/*
 * sd_read_block() — read one 512-byte sector with optional CRC validation.
 * Retries up to SD_READ_RETRIES times on transient CRC errors or timeouts.
 */
bool sd_read_block(uint32_t sector, uint8_t *buf)
{
    uint32_t addr = is_sdhc ? sector : sector * 512;

    for (int attempt = 0; attempt < SD_READ_RETRIES; attempt++) {
        last_error = SD_ERR_NONE;

        uint8_t r = sd_cmd(CMD17, addr);
        if (r & R1_ERROR_MASK) {
            spi_xfer(0xFF); // Ncs gap
            last_error = SD_ERR_CMD;
            continue;
        }

        if (!wait_for_data_token(SD_READ_TIMEOUT_MS)) {
            spi_xfer(0xFF); // Ncs gap
            continue; // last_error already set inside wait_for_data_token
        }

        // Read 512 data bytes
        memset(buf, 0xFF, 512);
        spi_write_read_blocking(SD_SPI, buf, buf, 512);

        // Read 2 CRC bytes (MSB first)
        uint8_t crc_hi = spi_xfer(0xFF);
        uint8_t crc_lo = spi_xfer(0xFF);
        spi_xfer(0xFF); // Ncs gap

#if SD_CRC_ENABLED
        uint16_t received_crc = ((uint16_t)crc_hi << 8) | crc_lo;
        uint16_t computed_crc = crc16_ccitt(buf, 512);
        if (received_crc != computed_crc) {
            last_error = SD_ERR_CRC_DATA;
            continue; // retry
        }
#else
        (void)crc_hi; (void)crc_lo;
#endif

        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Public API — single-block write (CMD24, §7.5.4)
// ---------------------------------------------------------------------------

/*
 * sd_write_block() — write one 512-byte sector with optional CRC.
 *
 * Write sequence:
 *   CMD24 → R1 → (Nwr dummy byte) → DATA_START_SINGLE → 512 bytes
 *   → CRC16 (2 bytes) → data response token → wait busy (Nac)
 */
bool sd_write_block(uint32_t sector, const uint8_t *buf)
{
    uint32_t addr = is_sdhc ? sector : sector * 512;
    last_error = SD_ERR_NONE;

    uint8_t r = sd_cmd(CMD24, addr);
    if (r & R1_ERROR_MASK) {
        spi_xfer(0xFF); // Ncs gap
        last_error = SD_ERR_CMD;
        return false;
    }

    spi_xfer(0xFF); // Nwr: one dummy byte before data token (§7.5.4)
    spi_xfer(DATA_START_SINGLE);
    spi_write_blocking(SD_SPI, buf, 512);

#if SD_CRC_ENABLED
    uint16_t crc = crc16_ccitt(buf, 512);
    uint8_t crc_bytes[2] = { (uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF) };
    spi_write_blocking(SD_SPI, crc_bytes, 2);
#else
    spi_xfer(0xFF); // dummy CRC MSB
    spi_xfer(0xFF); // dummy CRC LSB
#endif

    // Data response token (§7.3.3.1): lower 5 bits encode acceptance
    uint8_t resp = spi_xfer(0xFF) & DATA_RESP_MASK;
    spi_xfer(0xFF); // Ncs gap

    if (resp == DATA_RESP_CRC_ERR) { last_error = SD_ERR_CRC_DATA;    return false; }
    if (resp == DATA_RESP_WR_ERR)  { last_error = SD_ERR_WRITE_REJECT; return false; }
    if (resp != DATA_RESP_ACCEPTED){ last_error = SD_ERR_WRITE_REJECT; return false; }

    // Wait for card to finish internal programming (Nac, up to SD_WRITE_TIMEOUT_MS)
    bool ok = wait_ready(SD_WRITE_TIMEOUT_MS);
    spi_xfer(0xFF); // Ncs gap

    if (!ok) last_error = SD_ERR_TIMEOUT;
    return ok;
}

// ---------------------------------------------------------------------------
// Internal: send CMD12 while CS is already asserted (CMD18 context)
// ---------------------------------------------------------------------------

/*
 * sd_send_cmd12() — send STOP_TRANSMISSION without toggling CS.
 *
 * Must be called while CS is still asserted (i.e. from inside a CMD18
 * multi-block read).  sd_cmd() no longer touches CS (it stays permanently
 * asserted), but sd_send_cmd12 still exists as a separate function because
 * it must insert the CMD12-specific stuff byte before polling for R1.
 *
 * Per §7.5.6: one "stuff byte" is clocked after the 6-byte command packet
 * and before polling for the R1 response byte.
 */
static uint8_t sd_send_cmd12(void)
{
    uint8_t packet[6];
    packet[0] = 0x40 | CMD12;
    packet[1] = packet[2] = packet[3] = packet[4] = 0x00;
#if SD_CRC_ENABLED
    packet[5] = (crc7(packet, 5) << 1) | 0x01;
#else
    packet[5] = 0xFF;
#endif
    spi_write_blocking(SD_SPI, packet, 6);
    spi_xfer(0xFF); // stuff byte (§7.5.6) — sent before polling for R1
    uint8_t r = 0xFF;
    for (int i = 0; i < 8; i++) {
        r = spi_xfer(0xFF);
        if (!(r & 0x80)) break;
    }
    return r;
}

// ---------------------------------------------------------------------------
// Public API — multi-block read (CMD18, §7.5.3)
// ---------------------------------------------------------------------------

/*
 * sd_read_blocks() — read `count` contiguous 512-byte sectors into buf.
 * For count == 1, delegates to sd_read_block() (same retries + CRC path).
 * For count > 1, uses CMD18 (READ_MULTIPLE_BLOCK):
 *   CMD18 → R1 → [DATA_START_SINGLE → 512 bytes → CRC16] × count → CMD12
 *
 * CMD12 is sent via sd_send_cmd12() while CS stays asserted — never via
 * sd_cmd() — to avoid the CS glitch that ends CMD18 mode prematurely.
 */
bool sd_read_blocks(uint32_t sector, uint32_t count, uint8_t *buf)
{
    if (count == 1) return sd_read_block(sector, buf);

    uint32_t addr = is_sdhc ? sector : sector * 512;
    last_error = SD_ERR_NONE;

    uint8_t r = sd_cmd(CMD18, addr);
    if (r & R1_ERROR_MASK) {
        spi_xfer(0xFF); // Ncs gap
        last_error = SD_ERR_CMD;
        return false;
    }

    for (uint32_t i = 0; i < count; i++) {
        if (!wait_for_data_token(SD_READ_TIMEOUT_MS)) {
            // Abort: CS is still asserted — send CMD12 without deselecting first
            sd_send_cmd12();
            wait_ready(SD_READ_TIMEOUT_MS);
            spi_xfer(0xFF); // Ncs gap
            return false;
        }

        uint8_t *block = buf + (i * 512);
        memset(block, 0xFF, 512);
        spi_write_read_blocking(SD_SPI, block, block, 512);

        uint8_t crc_hi = spi_xfer(0xFF);
        uint8_t crc_lo = spi_xfer(0xFF);

#if SD_CRC_ENABLED
        uint16_t received_crc = ((uint16_t)crc_hi << 8) | crc_lo;
        uint16_t computed_crc = crc16_ccitt(block, 512);
        if (received_crc != computed_crc) {
            last_error = SD_ERR_CRC_DATA;
            // Abort: CS is still asserted
            sd_send_cmd12();
            wait_ready(SD_READ_TIMEOUT_MS);
            spi_xfer(0xFF); // Ncs gap
            return false;
        }
#else
        (void)crc_hi; (void)crc_lo;
#endif
    }

    // CMD12 — STOP_TRANSMISSION: CS is still asserted from CMD18
    // Must use sd_send_cmd12(), NOT sd_cmd(), to avoid the CS glitch.
    r = sd_send_cmd12();
    wait_ready(SD_READ_TIMEOUT_MS);
    spi_xfer(0xFF); // Ncs gap

    if (r & R1_ERROR_MASK) {
        last_error = SD_ERR_CMD;
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Public API — multi-block write (CMD25, §7.5.4)
// ---------------------------------------------------------------------------

/*
 * sd_write_blocks() — write `count` contiguous 512-byte sectors from buf.
 * For count == 1, delegates to sd_write_block().
 * For count > 1, uses CMD25 (WRITE_MULTIPLE_BLOCK):
 *   [ACMD23 pre-erase hint] → CMD25 → R1
 *   → [(Nwr) → DATA_START_MULTI → 512 bytes → CRC16 → data response
 *      → wait busy] × count
 *   → DATA_STOP_TRAN → (dummy) → wait busy
 *
 * ACMD23 is an optional pre-erase hint — failure is non-fatal (§4.3.14).
 */
bool sd_write_blocks(uint32_t sector, uint32_t count, const uint8_t *buf)
{
    if (count == 1) return sd_write_block(sector, buf);

    uint32_t addr = is_sdhc ? sector : sector * 512;
    last_error = SD_ERR_NONE;

    // ACMD23 — pre-erase hint (CMD55 + ACMD23); ignore errors (optional hint)
    uint8_t r = sd_cmd(CMD55, 0);
    spi_xfer(0xFF); // Ncs gap
    if (!(r & R1_ERROR_MASK)) {
        r = sd_cmd(ACMD23, count);
        spi_xfer(0xFF); // Ncs gap
        // Non-fatal — do not check r
    }

    // CMD25 — start multi-block write
    r = sd_cmd(CMD25, addr);
    if (r & R1_ERROR_MASK) {
        spi_xfer(0xFF); // Ncs gap
        last_error = SD_ERR_CMD;
        return false;
    }

    for (uint32_t i = 0; i < count; i++) {
        const uint8_t *block = buf + (i * 512);

        spi_xfer(0xFF); // Nwr: one dummy byte before data token
        spi_xfer(DATA_START_MULTI);
        spi_write_blocking(SD_SPI, block, 512);

#if SD_CRC_ENABLED
        uint16_t crc = crc16_ccitt(block, 512);
        uint8_t crc_bytes[2] = { (uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF) };
        spi_write_blocking(SD_SPI, crc_bytes, 2);
#else
        spi_xfer(0xFF);
        spi_xfer(0xFF);
#endif

        uint8_t resp = spi_xfer(0xFF) & DATA_RESP_MASK;
        if (resp != DATA_RESP_ACCEPTED) {
            if (resp == DATA_RESP_CRC_ERR)    last_error = SD_ERR_CRC_DATA;
            else if (resp == DATA_RESP_WR_ERR) last_error = SD_ERR_WRITE_REJECT;
            else                               last_error = SD_ERR_WRITE_REJECT;
            // Abort: send stop-tran token and wait for card to recover
            spi_xfer(DATA_STOP_TRAN);
            spi_xfer(0xFF);
            wait_ready(SD_WRITE_TIMEOUT_MS);
            spi_xfer(0xFF); // Ncs gap
            return false;
        }

        // Wait for block programming to complete (busy = MISO held low)
        if (!wait_ready(SD_WRITE_TIMEOUT_MS)) {
            last_error = SD_ERR_TIMEOUT;
            spi_xfer(DATA_STOP_TRAN);
            spi_xfer(0xFF);
            wait_ready(SD_WRITE_TIMEOUT_MS);
            spi_xfer(0xFF); // Ncs gap
            return false;
        }
    }

    // DATA_STOP_TRAN — terminate multi-block write sequence
    spi_xfer(DATA_STOP_TRAN);
    spi_xfer(0xFF); // dummy byte after stop token
    if (!wait_ready(SD_WRITE_TIMEOUT_MS)) {
        spi_xfer(0xFF); // Ncs gap
        last_error = SD_ERR_TIMEOUT;
        return false;
    }
    spi_xfer(0xFF); // Ncs gap
    return true;
}

// ---------------------------------------------------------------------------
// Public API — card capacity via CSD (CMD9, §5.3)
// ---------------------------------------------------------------------------

/*
 * sd_get_sector_count() — parse the CSD register to derive total sector count.
 *
 * CMD9 (SEND_CSD) returns a 16-byte register transmitted as a data block
 * (with DATA_START_SINGLE token + 2 CRC bytes, like a read command).
 *
 * CSD structure version is in bits [127:126] = csd[0] >> 6:
 *   0 = CSD v1 (SDSC cards)
 *   1 = CSD v2 (SDHC/SDXC cards)
 *
 * CSD v1 formula (§5.3.2):
 *   C_SIZE[11:0]   = csd[6][1:0] << 10 | csd[7] << 2 | csd[8][7:6]
 *   C_SIZE_MULT[2:0] = csd[9][7:5] (note: some sources cite different byte/bit)
 *   READ_BL_LEN[3:0] = csd[5][3:0]
 *   sectors = (C_SIZE + 1) << (C_SIZE_MULT + 2) × (2^READ_BL_LEN / 512)
 *           = (C_SIZE + 1) << (C_SIZE_MULT + READ_BL_LEN - 7)
 *
 * CSD v2 formula (§5.3.3):
 *   C_SIZE[21:0] = csd[7][5:0] << 16 | csd[8] << 8 | csd[9]
 *   sectors = (C_SIZE + 1) * 1024
 */
bool sd_get_sector_count(uint32_t *count)
{
    last_error = SD_ERR_NONE;

    uint8_t r = sd_cmd(CMD9, 0);
    if (r & R1_ERROR_MASK) {
        spi_xfer(0xFF); // Ncs gap
        last_error = SD_ERR_CMD;
        return false;
    }

    if (!wait_for_data_token(SD_READ_TIMEOUT_MS)) {
        spi_xfer(0xFF); // Ncs gap
        return false;
    }

    uint8_t csd[16];
    memset(csd, 0xFF, sizeof(csd));
    spi_write_read_blocking(SD_SPI, csd, csd, 16);

    // Read and optionally verify CRC
    uint8_t crc_hi = spi_xfer(0xFF);
    uint8_t crc_lo = spi_xfer(0xFF);
    spi_xfer(0xFF); // Ncs gap

#if SD_CRC_ENABLED
    uint16_t received_crc = ((uint16_t)crc_hi << 8) | crc_lo;
    uint16_t computed_crc = crc16_ccitt(csd, 16);
    if (received_crc != computed_crc) {
        last_error = SD_ERR_CRC_DATA;
        return false;
    }
#else
    (void)crc_hi; (void)crc_lo;
#endif

    uint8_t csd_ver = (csd[0] >> 6) & 0x03;

    if (csd_ver == 1) {
        // CSD v2: C_SIZE field at bytes [7][5:0], [8][7:0], [9][7:0]
        uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16)
                        | ((uint32_t) csd[8]          <<  8)
                        |  (uint32_t) csd[9];
        *count = (c_size + 1) * 1024;
    } else {
        // CSD v1: C_SIZE at [6][1:0]<<10 | [7]<<2 | [8][7:6]>>6
        uint32_t c_size = ((uint32_t)(csd[6] & 0x03) << 10)
                        | ((uint32_t) csd[7]          <<  2)
                        | ((uint32_t)(csd[8] & 0xC0)  >>  6);
        uint8_t c_size_mult  = (csd[9]  >> 5) & 0x07; // [9][7:5] per spec Table 5-16
        // Note: actual bit positions per spec:
        //   C_SIZE_MULT[2:1] at csd[9][7:6] and C_SIZE_MULT[0] at csd[10][7]
        // Simplified extraction that works for standard v1 layout:
        c_size_mult = (uint8_t)(((csd[9] & 0x03) << 1) | ((csd[10] >> 7) & 0x01));
        uint8_t read_bl_len  = csd[5] & 0x0F;
        *count = (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7u);
    }

    return true;
}

// ---------------------------------------------------------------------------
// Public API — error reporting
// ---------------------------------------------------------------------------

sd_error_t sd_last_error(void)
{
    return last_error;
}
