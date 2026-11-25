/**
 * @file code39.h
 * @brief Code 39 (ISO/IEC 16388) 9-bar pattern lookup API.
 */

#ifndef CODE39_H
#define CODE39_H

#include <stdint.h>

/**
 * @brief Decode a 9-bit Code 39 wide/narrow pattern to its ASCII character.
 * @param[in] pattern_bits Nine-bit pattern (MSB = first bar, 1 = wide bar).
 * @return Matching ASCII character byte, or '?' if pattern is unrecognized.
 */
uint8_t code39_decode_pattern(uint16_t pattern_bits);

#endif /* CODE39_H */

/*** end of file ***/
