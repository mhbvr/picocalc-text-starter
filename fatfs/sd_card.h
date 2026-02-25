#pragma once
#include <stdbool.h>
#include <stdint.h>

// GPIO / SPI configuration (same pins as old sdcard.h)
#define SD_SPI       spi0
#define SD_MISO      16
#define SD_CS        17
#define SD_SCK       18
#define SD_MOSI      19
#define SD_DETECT    22
#define SD_INIT_BAUD 400000    // 400 kHz for init
#define SD_FAST_BAUD 25000000  // 25 MHz for data

void     sd_init(void);                                       // GPIO/SPI setup
bool     sd_card_present(void);                               // card detect pin
bool     sd_is_sdhc(void);                                    // SDHC vs SDSC
bool     sd_card_init(void);                                  // full init sequence
bool     sd_read_block(uint32_t sector, uint8_t *buf);        // single-block read
bool     sd_write_block(uint32_t sector, const uint8_t *buf); // single-block write
