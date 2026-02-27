//
// sd_card.c - Production-ready SPI SD card driver for PicoCalc
//
// Implements SPI Mode (CPOL=0, CPHA=0) per Physical Layer Simplified Spec v9.00.
// Supports SDSC (v1 & v2) and SDHC/SDXC. Features:
//   - CRC7 on command packets, CRC16-CCITT on data blocks (SD_CRC_ENABLED)
//   - CMD18 multi-block read, CMD25 multi-block write
//   - CSD register parsing for card capacity
//

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "sd_card.h"
#include "crc.h"

// ---------------------------------------------------------------------------
// Command numbers (§4.7.4 Detailed Command Description)
// ---------------------------------------------------------------------------
#define CMD0   0    // GO_IDLE_STATE        — reset to idle / SPI mode
#define CMD8   8    // SEND_IF_COND         — voltage check (SDv2 detection)
#define CMD9   9    // SEND_CSD             — read card-specific data register
#define CMD12  12   // STOP_TRANSMISSION    — end CMD18 multi-block read
#define CMD16  16   // SET_BLOCKLEN         — set block length to 512
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
#define R1_START_BIT   0x80  // MSB clear = valid R1 byte; set = still waiting

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
#define ERR_TOKEN_GENERAL  0x01  // general/unknown card read error
#define ERR_TOKEN_CC_ERR   0x02  // card controller internal error
#define ERR_TOKEN_ECC_FAIL 0x04  // ECC applied but failed to correct the data
#define ERR_TOKEN_OOR      0x08  // command argument out of allowed range

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
static bool       is_sdhc           = false;
static bool       sd_gpio_init_done = false;


// ---------------------------------------------------------------------------
// Low-level SPI helpers
// ---------------------------------------------------------------------------

/*
 * wait_ready() — poll MISO until card returns 0xFF (not busy).
 * Used after write commands (Nwr / Nac per §7.5.4 and §7.5.6).
 * CS is permanently asserted after sd_card_init(), so no select needed.
 */
static sd_error_t wait_ready(uint32_t ms)
{
    uint8_t ff = 0xFF, r;
    absolute_time_t deadline = make_timeout_time_ms(ms);
    while (!time_reached(deadline)) {
        spi_write_read_blocking(SD_SPI, &ff, &r, 1);
        if (r == 0xFF) return SD_ERR_NONE;
    }
    return SD_ERR_TIMEOUT;
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
 * CMD12 requires one stuff byte clocked after the packet before R1 (§7.5.6).
 * Ncr (response latency) is at most 8 bytes per spec §7.5.1.
 */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t packet[6];
    packet[0] = 0x40 | (cmd & 0x3F);
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
    uint8_t ff = 0xFF;

    // CMD12 stuff byte (§7.5.6): one byte clocked before polling for R1
    if (cmd == CMD12) {
        spi_write_blocking(SD_SPI, &ff, 1);
    }

    // Poll for R1 — MSB clear means valid response; Ncr max = 8 bytes (§7.5.1)
    uint8_t r = 0xFF;
    for (int i = 0; i < 8; i++) {
        spi_write_read_blocking(SD_SPI, &ff, &r, 1);
        if (!(r & R1_START_BIT)) break;
    }

    return r;
}

// Map R1 response error bits to a specific sd_error_t (§7.3.2.1).
static sd_error_t r1_to_error(uint8_t r)
{
    if (r & R1_CRC_ERROR)                    return SD_ERR_CRC_CMD;
    if (r & (R1_PARAM_ERROR | R1_ADDR_ERROR)) return SD_ERR_OOR;
    return SD_ERR_CMD;
}

/*
 * sd_read_r3r7() — read the 4 trailing bytes of an R3 or R7 response.
 * Must be called immediately after sd_cmd() returned R1 == 0x01 for
 * CMD58 (READ_OCR → R3) or CMD8 (SEND_IF_COND → R7).
 */
static void sd_read_r3r7(uint8_t out[4])
{
    memset(out, 0xFF, 4);
    spi_write_read_blocking(SD_SPI, out, out, 4);
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

bool sd_is_sdhc(void)
{
    return is_sdhc;
}

bool sd_card_present(void)
{
    return !gpio_get(SD_DETECT); // active-low
}


/*
 * sd_card_init() — full SD SPI-mode initialisation sequence (§7.2.1).
 *
 * CS is asserted (low) once after the 80 dummy clocks and held low for the
 * remainder of the session.  A single dummy byte is clocked after each response
 * to satisfy the Ncs inter-command gap required by §7.5.1.
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
 * 11. CMD16(512) — set block length to 512 bytes
 * 12. Switch SPI to SD_FAST_BAUD
 */
sd_error_t sd_card_init(void)
{
    uint8_t ff = 0xFF;

    if (!sd_card_present())
        return SD_ERR_NO_CARD;

    spi_init(SD_SPI, SD_INIT_BAUD);
    gpio_put(SD_CS, 1);              // CS HIGH for power-up phase
    busy_wait_us(10000);             // ≥1 ms power-up delay; 10 ms is conservative

    // ≥74 dummy clocks with CS deasserted to put card into native command state
    uint8_t clocks[10];
    memset(clocks, 0xFF, sizeof(clocks));
    spi_write_blocking(SD_SPI, clocks, sizeof(clocks)); // 10 × 8 = 80 clocks

    gpio_put(SD_CS, 0);              // Assert CS — held low for the rest of the session
    busy_wait_us(1000);

    // CMD0 — reset to idle / enter SPI mode (expect R1 = 0x01)
    uint8_t r = 0;
    for (int attempt = 0; attempt < SD_CMD_RETRIES; attempt++) {
        r = sd_cmd(CMD0, 0);
        spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
        if (r == R1_IDLE) break;
        busy_wait_us(10000);
    }
    if (r != R1_IDLE)
        return SD_ERR_TIMEOUT;

    // CMD8 — SEND_IF_COND (§4.3.13): detect SDv2 by echo of 0x01AA
    // R7 response: [R1][voltage][echo_voltage][echo_check][echo_arg]
    bool is_v2 = false;
    r = sd_cmd(CMD8, 0x1AA); // VHS = 0x01 (2.7–3.6V), check pattern 0xAA
    if (r == R1_IDLE) {
        uint8_t r7[4];
        sd_read_r3r7(r7);
        spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
        // r7[2] bits[3:0] = voltage accepted (must be 0x01); r7[3] = echo
        if ((r7[2] & 0x0F) == 0x01 && r7[3] == 0xAA)
            is_v2 = true;
    } else {
        spi_write_blocking(SD_SPI, &ff, 1); // SDv1 or MMC — CMD8 is illegal, dummy cycle
    }

#if SD_CRC_ENABLED
    // CMD59(1) — enable CRC checking on the card side (§7.2.2)
    sd_cmd(CMD59, 1);
    spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
    // Non-fatal: some cards don't support CMD59; continue regardless
#endif

    // CMD58 — read OCR; verify 3.3V operating range (bit 20, §5.1 Table 5-1)
    r = sd_cmd(CMD58, 0);
    uint8_t ocr[4] = {0};
    sd_read_r3r7(ocr);
    spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
    if (r & R1_ERROR_MASK)
        return SD_ERR_CMD;
    // ocr[1] bit 4 = 3.2–3.3V; bit 5 = 3.3–3.4V (typical 3.3V cards set both)
    if (!(ocr[1] & 0x30))
        return SD_ERR_CMD;

    // ACMD41 loop — SD_SEND_OP_COND: wait until card reports power-up complete
    // CMD55 (APP_CMD) must precede every ACMD (§4.3.9)
    absolute_time_t deadline = make_timeout_time_ms(SD_INIT_TIMEOUT_MS);
    do {
        r = sd_cmd(CMD55, 0);
        spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
        if (r & R1_ERROR_MASK)
            return SD_ERR_CMD;
        uint32_t hcs = is_v2 ? (1u << 30) : 0; // HCS bit — request SDHC if v2
        r = sd_cmd(ACMD41, hcs);
        spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
        if (r == 0) break; // idle bit clear = card ready
        busy_wait_us(1000);
    } while (!time_reached(deadline));

    if (r != 0)
        return SD_ERR_TIMEOUT;

    // CMD58 — re-read OCR to check CCS bit (bit 30) for SDHC vs SDSC
    r = sd_cmd(CMD58, 0);
    sd_read_r3r7(ocr);
    spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
    if (r & R1_ERROR_MASK)
        return SD_ERR_CMD;
    // ocr[0] bit 6 = CCS: 1 = SDHC/SDXC (block-addressed), 0 = SDSC (byte-addressed)
    is_sdhc = (ocr[0] & 0x40) != 0;

    // CMD16 — fix block length to 512 bytes
    r = sd_cmd(CMD16, 512);
    spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
    if (r != 0)
        return SD_ERR_CMD;

    spi_set_baudrate(SD_SPI, SD_FAST_BAUD);
    return SD_ERR_NONE;
}

// ---------------------------------------------------------------------------
// Internal: wait for data start token with timeout (§7.3.3.2)
// Returns SD_ERR_NONE when DATA_START_SINGLE (0xFE) is received.
// Returns a specific error for error tokens or timeout.
// ---------------------------------------------------------------------------
static sd_error_t wait_for_data_token(uint32_t ms)
{
    uint8_t ff = 0xFF, tok;
    absolute_time_t deadline = make_timeout_time_ms(ms);
    while (!time_reached(deadline)) {
        spi_write_read_blocking(SD_SPI, &ff, &tok, 1);
        if (tok == DATA_START_SINGLE) return SD_ERR_NONE;
        if (tok != 0xFF) {
            // Error token: multiple bits may be set; check most specific first
            if (tok & ERR_TOKEN_OOR)      return SD_ERR_OOR;
            if (tok & ERR_TOKEN_ECC_FAIL) return SD_ERR_CRC_DATA;
            if (tok & ERR_TOKEN_CC_ERR)   return SD_ERR_CARD_CONTROLLER;
            if (tok & ERR_TOKEN_GENERAL)  return SD_ERR_GENERAL;
            return SD_ERR_DATA_TOKEN;
        }
    }
    return SD_ERR_TIMEOUT;
}

// ---------------------------------------------------------------------------
// Public API — single-block read (CMD17, §7.5.3)
// ---------------------------------------------------------------------------

/*
 * sd_read_block() — read one 512-byte sector with optional CRC validation.
 * Returns SD_ERR_NONE on success, or a specific error code on failure.
 * Retries are the responsibility of the caller.
 */
sd_error_t sd_read_block(uint32_t sector, uint8_t *buf)
{
    uint8_t ff = 0xFF;

    uint8_t r = sd_cmd(CMD17, sector);
    if (r & R1_ERROR_MASK) {
        spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
        return r1_to_error(r);
    }

    sd_error_t err = wait_for_data_token(SD_READ_TIMEOUT_MS);
    if (err != SD_ERR_NONE) {
        spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
        return err;
    }

    memset(buf, 0xFF, 512);
    spi_write_read_blocking(SD_SPI, buf, buf, 512);

    uint8_t crc[2] = {0xFF, 0xFF};
    spi_write_read_blocking(SD_SPI, crc, crc, 2);

#if SD_CRC_ENABLED
    if ((((uint16_t)crc[0] << 8) | crc[1]) != crc16_ccitt(buf, 512))
        return SD_ERR_CRC_DATA;
#endif

    return SD_ERR_NONE;
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
 * Returns SD_ERR_NONE on success, or a specific error code on failure.
 */
sd_error_t sd_write_block(uint32_t sector, const uint8_t *buf)
{
    uint8_t ff = 0xFF;

    uint8_t r = sd_cmd(CMD24, sector);
    if (r & R1_ERROR_MASK) {
        spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
        return r1_to_error(r);
    }

    spi_write_blocking(SD_SPI, &ff, 1); // Nwr: dummy byte before data token (§7.5.4)
    spi_write_blocking(SD_SPI, (const uint8_t[]){DATA_START_SINGLE}, 1);
    spi_write_blocking(SD_SPI, buf, 512);

#if SD_CRC_ENABLED
    uint16_t crc = crc16_ccitt(buf, 512);
    uint8_t crc_bytes[2] = { (uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF) };
    spi_write_blocking(SD_SPI, crc_bytes, 2);
#else
    uint8_t dummy_crc[2] = {0xFF, 0xFF};
    spi_write_blocking(SD_SPI, dummy_crc, 2);
#endif

    // Data response token (§7.3.3.1): lower 5 bits encode acceptance
    uint8_t resp;
    spi_write_read_blocking(SD_SPI, &ff, &resp, 1);
    resp &= DATA_RESP_MASK;
    spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle

    if (resp == DATA_RESP_CRC_ERR)     return SD_ERR_CRC_DATA;
    if (resp != DATA_RESP_ACCEPTED)    return SD_ERR_WRITE_REJECT;

    return wait_ready(SD_WRITE_TIMEOUT_MS);
}

// ---------------------------------------------------------------------------
// Public API — multi-block read (CMD18, §7.5.3)
// ---------------------------------------------------------------------------

/*
 * sd_read_blocks() — read `count` contiguous 512-byte sectors into buf.
 * For count == 1, delegates to sd_read_block().
 * For count > 1, uses CMD18 (READ_MULTIPLE_BLOCK):
 *   CMD18 → R1 → [DATA_START_SINGLE → 512 bytes → CRC16] × count → CMD12
 *
 * CMD12 (STOP_TRANSMISSION) is sent via sd_cmd(); the stuff byte required by
 * §7.5.6 is handled inside sd_cmd() for CMD12.
 * Returns SD_ERR_NONE on success, or a specific error code on failure.
 */
sd_error_t sd_read_blocks(uint32_t sector, uint32_t count, uint8_t *buf)
{
    if (count == 1)
        return sd_read_block(sector, buf);

    uint8_t ff = 0xFF;

    uint8_t r = sd_cmd(CMD18, sector);
    if (r & R1_ERROR_MASK) {
        spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
        return r1_to_error(r);
    }

    sd_error_t err = SD_ERR_NONE;
    for (uint32_t i = 0; i < count && err == SD_ERR_NONE; i++) {
        uint8_t *block = buf + (i * 512);

        err = wait_for_data_token(SD_READ_TIMEOUT_MS);
        if (err != SD_ERR_NONE)
            break;

        memset(block, 0xFF, 512);
        spi_write_read_blocking(SD_SPI, block, block, 512);

        uint8_t crc[2] = {0xFF, 0xFF};
        spi_write_read_blocking(SD_SPI, crc, crc, 2);

#if SD_CRC_ENABLED
        if ((((uint16_t)crc[0] << 8) | crc[1]) != crc16_ccitt(block, 512))
            err = SD_ERR_CRC_DATA;
#endif
    }

    // CMD12 — always send STOP_TRANSMISSION, whether the loop succeeded or aborted
    r = sd_cmd(CMD12, 0);
    sd_error_t ready_err = wait_ready(SD_READ_TIMEOUT_MS);
    spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle

    if (err != SD_ERR_NONE)        return err;
    if (r & R1_ERROR_MASK)         return r1_to_error(r);
    return ready_err;
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
sd_error_t sd_write_blocks(uint32_t sector, uint32_t count, const uint8_t *buf)
{
    if (count == 1)
        return sd_write_block(sector, buf);

    uint8_t ff = 0xFF;

    // ACMD23 — pre-erase hint (CMD55 + ACMD23); non-fatal if unsupported (§4.3.14)
    uint8_t r = sd_cmd(CMD55, 0);
    spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
    if (!(r & R1_ERROR_MASK)) {
        sd_cmd(ACMD23, count);
        spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
    }

    // CMD25 — start multi-block write
    r = sd_cmd(CMD25, sector);
    if (r & R1_ERROR_MASK) {
        spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
        return r1_to_error(r);
    }

    sd_error_t err = SD_ERR_NONE;
    for (uint32_t i = 0; i < count && err == SD_ERR_NONE; i++) {
        const uint8_t *block = buf + (i * 512);

        spi_write_blocking(SD_SPI, &ff, 1); // Nwr: dummy byte before data token
        spi_write_blocking(SD_SPI, (const uint8_t[]){DATA_START_MULTI}, 1);
        spi_write_blocking(SD_SPI, block, 512);

#if SD_CRC_ENABLED
        uint16_t crc = crc16_ccitt(block, 512);
        uint8_t crc_bytes[2] = { (uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF) };
        spi_write_blocking(SD_SPI, crc_bytes, 2);
#else
        uint8_t dummy_crc[2] = {0xFF, 0xFF};
        spi_write_blocking(SD_SPI, dummy_crc, 2);
#endif

        uint8_t resp;
        spi_write_read_blocking(SD_SPI, &ff, &resp, 1);
        resp &= DATA_RESP_MASK;
        if (resp == DATA_RESP_CRC_ERR)  { 
            err = SD_ERR_CRC_DATA;    
            break; 
        }
        if (resp != DATA_RESP_ACCEPTED) { 
            err = SD_ERR_WRITE_REJECT; 
            break; 
        }

        err = wait_ready(SD_WRITE_TIMEOUT_MS);
    }

    // DATA_STOP_TRAN — always terminate the multi-block write sequence
    spi_write_blocking(SD_SPI, (const uint8_t[]){DATA_STOP_TRAN}, 1);
    spi_write_blocking(SD_SPI, &ff, 1); // dummy byte after stop token
    return err != SD_ERR_NONE ? err : wait_ready(SD_WRITE_TIMEOUT_MS);
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
sd_error_t sd_get_sector_count(uint32_t *count)
{
    uint8_t ff = 0xFF;

    uint8_t r = sd_cmd(CMD9, 0);
    if (r & R1_ERROR_MASK) {
        spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
        return r1_to_error(r);
    }

    sd_error_t err = wait_for_data_token(SD_READ_TIMEOUT_MS);
    if (err != SD_ERR_NONE) {
        spi_write_blocking(SD_SPI, &ff, 1); // Dummy cycle
        return err;
    }

    uint8_t csd[16];
    memset(csd, 0xFF, sizeof(csd));
    spi_write_read_blocking(SD_SPI, csd, csd, 16);

    uint8_t crc[2] = {0xFF, 0xFF};
    spi_write_read_blocking(SD_SPI, crc, crc, 2);

#if SD_CRC_ENABLED
    if ((((uint16_t)crc[0] << 8) | crc[1]) != crc16_ccitt(csd, 16))
        return SD_ERR_CRC_DATA;
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

    return SD_ERR_NONE;
}
