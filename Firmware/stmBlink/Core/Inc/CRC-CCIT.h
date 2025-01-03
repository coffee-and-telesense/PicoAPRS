/*
 * CRC-CCIT.h
 *
 *  Created on: Aug 2, 2024
 *      Author: arie
 */

// CRC-CCIT Implementation based on work by Francesco Sacchi
#ifndef CRC_CCIT_H
#define CRC_CCIT_H

#include <stdint.h>

#define CRC_CCIT_INIT_VAL ((uint16_t)0xFFFF)

extern const uint16_t crc_ccit_table[256];

inline uint16_t update_crc_ccit(uint8_t c, uint16_t prev_crc) {
	return (prev_crc >> 8) ^ crc_ccit_table[(prev_crc ^ c) & 0xff];
}

#endif
