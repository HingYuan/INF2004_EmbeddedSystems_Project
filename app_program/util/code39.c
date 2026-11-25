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

#define CODE39_UNKNOWN_CHAR             (0x3Fu)
#define CODE39_ENTRY_COUNT              (44u)

typedef struct ROBOT_TASKS_PACKED
{
    uint16_t pattern   : 9;
    uint16_t character : 7;
} code39_entry_t;

ROBOT_TASKS_STATIC_ASSERT(code39_entry_size, (2u == sizeof(code39_entry_t)));

static const code39_entry_t g_code39_dict[CODE39_ENTRY_COUNT] =
{
    /* Encode uppercase ASCII letters A through Z (0x41..0x5A). */
    { 0x109u, 0x41u },
    { 0x049u, 0x42u },
    { 0x148u, 0x43u },
    { 0x019u, 0x44u },
    { 0x118u, 0x45u },
    { 0x058u, 0x46u },
    { 0x00Du, 0x47u },
    { 0x10Cu, 0x48u },
    { 0x04Cu, 0x49u },
    { 0x01Cu, 0x4Au },
    { 0x103u, 0x4Bu },
    { 0x043u, 0x4Cu },
    { 0x142u, 0x4Du },
    { 0x013u, 0x4Eu },
    { 0x112u, 0x4Fu },
    { 0x052u, 0x50u },
    { 0x007u, 0x51u },
    { 0x106u, 0x52u },
    { 0x046u, 0x53u },
    { 0x016u, 0x54u },
    { 0x181u, 0x55u },
    { 0x0C1u, 0x56u },
    { 0x1C0u, 0x57u },
    { 0x091u, 0x58u },
    { 0x190u, 0x59u },
    { 0x0D0u, 0x5Au },

    /* Encode ASCII decimal digits 0 through 9 (0x30..0x39). */
    { 0x034u, 0x30u },
    { 0x121u, 0x31u },
    { 0x061u, 0x32u },
    { 0x160u, 0x33u },
    { 0x031u, 0x34u },
    { 0x130u, 0x35u },
    { 0x070u, 0x36u },
    { 0x025u, 0x37u },
    { 0x124u, 0x38u },
    { 0x064u, 0x39u },

    /* Encode Code 39 symbols '-', '.', ' ', '$', '/', '+', '%', and '*'. */
    { 0x085u, 0x2Du },
    { 0x184u, 0x2Eu },
    { 0x0C4u, 0x20u },
    { 0x0A8u, 0x24u },
    { 0x0A2u, 0x2Fu },
    { 0x08Au, 0x2Bu },
    { 0x02Au, 0x25u },
    { 0x094u, 0x2Au }
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
