#pragma once
#include <stdint.h>

// CRC7 for SD command packets (polynomial x^7 + x^3 + 1).
// Caller appends result as (crc7(...) << 1) | 0x01.
uint8_t crc7(const uint8_t *data, int len);

// CRC-16/CCITT for 512-byte data blocks (polynomial x^16 + x^12 + x^5 + 1).
// Returns the CRC as a big-endian uint16_t ready to be sent MSB-first.
uint16_t crc16_ccitt(const uint8_t *data, int len);
