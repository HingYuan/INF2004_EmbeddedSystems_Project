/**
 * @file code39.c
 * @brief Code 39 (ISO/IEC 16388) 9-bar pattern lookup table implementation.
 *
 * NOTE: Reference standard ISO/IEC 16388 (Code 39 barcode symbology). Each
 * entry encodes 9 elements (5 bars, 4 spaces) where bit 8 is the leftmost
 * element and 1 denotes a wide element.
 */

#include "code39.h"

#include "../robot_tasks.h"

#define CODE39_UNKNOWN_CHAR             ((uint8_t)'?')
#define CODE39_ENTRY_COUNT              (44u)

typedef struct ROBOT_TASKS_PACKED
{
    uint16_t pattern   : 9;
    uint16_t character : 7;
} code39_entry_t;

ROBOT_TASKS_STATIC_ASSERT(code39_entry_size, (2u == sizeof(code39_entry_t)));

static const code39_entry_t g_code39_dict[CODE39_ENTRY_COUNT] =
{
    /* Letters A-Z */
    { 0x109u, (uint16_t)'A' },
    { 0x049u, (uint16_t)'B' },
    { 0x148u, (uint16_t)'C' },
    { 0x019u, (uint16_t)'D' },
    { 0x118u, (uint16_t)'E' },
    { 0x058u, (uint16_t)'F' },
    { 0x00Du, (uint16_t)'G' },
    { 0x10Cu, (uint16_t)'H' },
    { 0x04Cu, (uint16_t)'I' },
    { 0x01Cu, (uint16_t)'J' },
    { 0x103u, (uint16_t)'K' },
    { 0x043u, (uint16_t)'L' },
    { 0x142u, (uint16_t)'M' },
    { 0x013u, (uint16_t)'N' },
    { 0x112u, (uint16_t)'O' },
    { 0x052u, (uint16_t)'P' },
    { 0x007u, (uint16_t)'Q' },
    { 0x106u, (uint16_t)'R' },
    { 0x046u, (uint16_t)'S' },
    { 0x016u, (uint16_t)'T' },
    { 0x181u, (uint16_t)'U' },
    { 0x0C1u, (uint16_t)'V' },
    { 0x1C0u, (uint16_t)'W' },
    { 0x091u, (uint16_t)'X' },
    { 0x190u, (uint16_t)'Y' },
    { 0x0D0u, (uint16_t)'Z' },

    /* Digits 0-9 */
    { 0x034u, (uint16_t)'0' },
    { 0x121u, (uint16_t)'1' },
    { 0x061u, (uint16_t)'2' },
    { 0x160u, (uint16_t)'3' },
    { 0x031u, (uint16_t)'4' },
    { 0x130u, (uint16_t)'5' },
    { 0x070u, (uint16_t)'6' },
    { 0x025u, (uint16_t)'7' },
    { 0x124u, (uint16_t)'8' },
    { 0x064u, (uint16_t)'9' },

    /* Symbols and start/stop asterisk */
    { 0x085u, (uint16_t)'-' },
    { 0x184u, (uint16_t)'.' },
    { 0x0C4u, (uint16_t)' ' },
    { 0x0A8u, (uint16_t)'$' },
    { 0x0A2u, (uint16_t)'/' },
    { 0x08Au, (uint16_t)'+' },
    { 0x02Au, (uint16_t)'%' },
    { 0x094u, (uint16_t)'*' }
};

/**
 * @brief Decode a 9-bit Code 39 wide/narrow pattern to its ASCII character.
 * @param[in] pattern_bits Nine-bit pattern (MSB = first bar, 1 = wide bar).
 * @return Matching ASCII character byte, or '?' if pattern is unrecognized.
 */
uint8_t code39_decode_pattern (uint16_t pattern_bits)
{
    uint8_t decoded_char = CODE39_UNKNOWN_CHAR;

    for (uint32_t entry_idx = 0u; entry_idx < CODE39_ENTRY_COUNT; entry_idx++)
    {
        if (g_code39_dict[entry_idx].pattern == pattern_bits)
        {
            /* Range-safe: Code 39 ASCII characters are 7-bit (0..127). */
            decoded_char = (uint8_t)g_code39_dict[entry_idx].character;
            break;
        }
    }

    return decoded_char;
}

/*** end of file ***/
