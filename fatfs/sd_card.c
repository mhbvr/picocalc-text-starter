//
// sd_card.c - SPI SD card driver for PicoCalc
//
// Reimplemented from scratch inspired by carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico,
// simplified to C without DMA/CRC/threads.
//

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "sd_card.h"

// SD card commands
#define CMD0   0    // GO_IDLE_STATE
#define CMD8   8    // SEND_IF_COND
#define CMD16  16   // SET_BLOCKLEN
#define CMD17  17   // READ_SINGLE_BLOCK
#define CMD24  24   // WRITE_BLOCK
#define CMD55  55   // APP_CMD
#define CMD58  58   // READ_OCR
#define ACMD41 41   // SD_SEND_OP_COND

#define DATA_START_TOKEN 0xFE

// Global state
static bool is_sdhc = false;
static bool sd_gpio_init_done = false;

// ---------------------------------------------------------------------------
// Low-level SPI helpers
// ---------------------------------------------------------------------------

static uint8_t spi_xfer(uint8_t data)
{
    uint8_t result;
    spi_write_read_blocking(SD_SPI, &data, &result, 1);
    return result;
}

static void cs_select(void)
{
    gpio_put(SD_CS, 0);
}

static void cs_deselect(void)
{
    gpio_put(SD_CS, 1);
    spi_xfer(0xFF); // 1 dummy byte after deselect
}

// Poll until MISO = 0xFF (card ready) or timeout (ms)
static bool wait_ready(uint32_t ms)
{
    absolute_time_t deadline = make_timeout_time_ms(ms);
    while (!time_reached(deadline)) {
        if (spi_xfer(0xFF) == 0xFF) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// SD command
// ---------------------------------------------------------------------------

static uint8_t sd_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t packet[6];
    packet[0] = 0x40 | cmd;
    packet[1] = (arg >> 24) & 0xFF;
    packet[2] = (arg >> 16) & 0xFF;
    packet[3] = (arg >>  8) & 0xFF;
    packet[4] =  arg        & 0xFF;
    // Hardcoded CRC for the two commands that need it; 0xFF (CRC off) for rest
    if (cmd == CMD0)       packet[5] = 0x95;
    else if (cmd == CMD8)  packet[5] = 0x87;
    else                   packet[5] = 0xFF;

    cs_select();
    spi_write_blocking(SD_SPI, packet, 6);

    // Wait for R1 response (MSB clear means valid)
    uint8_t r = 0xFF;
    for (int i = 0; i < 16; i++) {
        r = spi_xfer(0xFF);
        if (!(r & 0x80)) break;
    }
    return r;
}

// ---------------------------------------------------------------------------
// Public API
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
    gpio_put(SD_CS, 1);
    gpio_set_dir(SD_DETECT, GPIO_IN);
    gpio_pull_up(SD_DETECT);

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

bool sd_card_init(void)
{
    if (!sd_card_present()) return false;

    // Start at init baud rate
    spi_init(SD_SPI, SD_INIT_BAUD);
    gpio_put(SD_CS, 1);
    busy_wait_us(10000);

    // 80 dummy clocks with CS high
    for (int i = 0; i < 10; i++) spi_xfer(0xFF);

    busy_wait_us(10000);

    // CMD0 - go idle (up to 10 retries)
    uint8_t r = 0;
    for (int attempt = 0; attempt < 10; attempt++) {
        r = sd_cmd(CMD0, 0);
        cs_deselect();
        if (r == 0x01) break;
        busy_wait_us(10000);
    }
    if (r != 0x01) return false;

    // CMD8 - detect SDv2 vs v1
    bool is_v2 = false;
    r = sd_cmd(CMD8, 0x1AA);
    if (r == 0x01) {
        // Read R7 tail (4 bytes)
        uint8_t r7[4];
        for (int i = 0; i < 4; i++) r7[i] = spi_xfer(0xFF);
        cs_deselect();
        // Check voltage accepted and echo pattern
        if ((r7[2] & 0x0F) == 0x01 && r7[3] == 0xAA) {
            is_v2 = true;
        }
    } else {
        cs_deselect();
    }

    // CMD58 - check OCR / verify 3.3V support (drain response, continue regardless)
    r = sd_cmd(CMD58, 0);
    for (int i = 0; i < 4; i++) spi_xfer(0xFF);
    cs_deselect();

    // ACMD41 loop - wait for card to leave idle (up to 1 second)
    absolute_time_t deadline = make_timeout_time_ms(1000);
    do {
        r = sd_cmd(CMD55, 0);
        cs_deselect();
        if (r > 1) return false;

        uint32_t hcs = is_v2 ? 0x40000000 : 0;
        r = sd_cmd(ACMD41, hcs);
        cs_deselect();
        if (r == 0) break;
        busy_wait_us(1000);
    } while (!time_reached(deadline));

    if (r != 0) return false;

    // CMD58 again - check CCS bit to determine SDHC
    r = sd_cmd(CMD58, 0);
    if (r != 0) { cs_deselect(); return false; }
    uint8_t ocr[4];
    for (int i = 0; i < 4; i++) ocr[i] = spi_xfer(0xFF);
    cs_deselect();
    is_sdhc = (ocr[0] & 0x40) != 0;

    // CMD16 - set block size to 512 for SDSC
    if (!is_sdhc) {
        r = sd_cmd(CMD16, 512);
        cs_deselect();
        if (r != 0) return false;
    }

    // Switch to fast baud
    spi_set_baudrate(SD_SPI, SD_FAST_BAUD);

    return true;
}

bool sd_read_block(uint32_t sector, uint8_t *buf)
{
    uint32_t addr = is_sdhc ? sector : sector * 512;

    uint8_t r = sd_cmd(CMD17, addr);
    if (r != 0) { cs_deselect(); return false; }

    // Wait for data token 0xFE (timeout 100 ms)
    absolute_time_t deadline = make_timeout_time_ms(100);
    do {
        r = spi_xfer(0xFF);
        if (r == DATA_START_TOKEN) break;
        if (time_reached(deadline)) { cs_deselect(); return false; }
    } while (r == 0xFF);

    if (r != DATA_START_TOKEN) { cs_deselect(); return false; }

    // Read 512 bytes
    memset(buf, 0xFF, 512);
    spi_write_read_blocking(SD_SPI, buf, buf, 512);

    // Skip 2 CRC bytes
    spi_xfer(0xFF);
    spi_xfer(0xFF);

    cs_deselect();
    return true;
}

bool sd_write_block(uint32_t sector, const uint8_t *buf)
{
    uint32_t addr = is_sdhc ? sector : sector * 512;

    uint8_t r = sd_cmd(CMD24, addr);
    if (r != 0) { cs_deselect(); return false; }

    // Send start token
    spi_xfer(DATA_START_TOKEN);

    // Send 512 bytes of data
    spi_write_blocking(SD_SPI, buf, 512);

    // Send dummy CRC (2 bytes)
    spi_xfer(0xFF);
    spi_xfer(0xFF);

    // Check data-response token; lower nibble 0x05 = accepted
    r = spi_xfer(0xFF) & 0x1F;
    cs_deselect();
    if (r != 0x05) return false;

    // Wait for card to finish programming
    cs_select();
    bool ok = wait_ready(500);
    cs_deselect();

    return ok;
}
