#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include <tk/tkernel.h>
#include <tk/typedef.h>

/*
 * Compression baseline (2026-11-22; professor's tk/typedef.h types).
 *
 * Field / valid domain                       Stored bits
 * motor power -100..100 / independent ticks 0..255     8 / 8 per wheel
 * motor state 0..6 / IMU state 0..3 / event 0..3       3 / 2 / 2
 * IR state 0..4 / bar colour 0..1 / ASCII 0..127       3 / 1 / 7
 * barcode duration (wrapping us delta)/100, 0..42949672     26
 * sonar distance 0..400 / status 0..6 / angle -45..45   9 / 3 / 7 signed
 * profile width 0..800 / offset including gap 0..411  10 / 9
 * pitch/roll -180..180 degrees, resolution 0.1       12 signed each
 * heading 0..359 / height 0..68 mm, resolution 0.1     9 / 10
 * Navigator mode 0..10 / bypass phase IDs 0..27        4 / 5
 * route direction 0..5 / route index 0..5             3 / 3
 * bypass side -1..1 / flags / confirmation 0..2        2 signed / 1 / 2
 * side/forward/leg travel incl. coast (guarded)       11 / 12 / 12
 * planned width 0..4220 mm                            13
 * stall counters saturate at 75 / encoder filter      7 each / 2 per channel
 * shared faults/control flags                        13 in one aligned UW
 *
 * Full 32-bit wrapping timers, sequence/encoder counters and signed odometry
 * retain their domains; shortening them would change wrap/freshness behaviour.
 * Native OS/driver arguments and arithmetic intermediates keep their API width.
 * IMU filter calculations remain float: only messages/display data use fixed
 * point. Both tilt axes, lift/overturn, hump and compass features remain.
 * Packed struct layout is internal to this GCC firmware, NOT the MQTT wire ABI.
 * MQTT uses explicit shifts/masks; see Network/mqtt_client.h.
 *
 * ARM GCC 13.3.0, Cortex-M0+, -O2 -DNDEBUG, SMP/MQTT/WEB enabled:
 * Type                        Before -> After bytes
 * MOTOR_COMMAND                    4 -> 4
 * BAR_RECORD                       8 -> 4
 * OBSTACLE_PROFILE               120 -> 26
 * SONAR_PROBE_RESULT              20 -> 7
 * SONAR_DISTANCE_MSG              12 -> 2
 * IMU_TELEMETRY_MSG               20 -> 6
 * NAVIGATOR_MESSAGE              124 -> 27
 * TELEMETRY_SNAPSHOT              68 -> 32
 * BYPASS_CONTEXT                 204 -> 61
 * IMU_CONFIRM                    12 -> 9
 * C_RING_BUFFER                  32 -> 22
 *
 * Static .data + .bss object sizes, same compiler/profile:
 * robot_tasks 9656->2312, Motor 94->52, Navigator 68->32, IR 0->0,
 * IMU 8->8, Ultrasonic 4->0, Bypass_helpers 0->0, MQTT 1692->289,
 * web_server 1584->2584 (larger API buffer fits all 66 mapping entries).
 * Total for those nine modules: 13106->5277 bytes, saving 7829 bytes.
 * Excludes linker alignment, kernel/drivers, heap and reserved task stacks.
 * Queue storage 9640->2288 bytes; capacities are now truly 20/64/10 messages,
 * including the kernel's word-rounded payload and four-byte record header.
 * Navigator payload copying falls 124->27 bytes/message (78.2% fewer bytes).
 * MQTT telemetry is 35 binary bytes; command 5, status 10, mapping 2+ceil(10N/8).
 *
 * Verification: all 13 application C units compile in 8 ARM configurations
 * (SMP 0/1, network 0/1, debug/release). Host C checks use the actual helpers:
 * all 65536 speed pairs, sonar 0..400 at every ray/status, 131072 encoder
 * sequences, malformed/fragmented binary frames, full mapping/dashboard JSON,
 * timer wrap, route/ring bounds, and existing IMU/bypass debug self-checks.
 * 20000 randomized mirrored width calculations match the pre-change algorithm.
 * This is compiler/host evidence, not on-car timing, flash or motion acceptance.
 *
 * Width reporting extension (2026-11-24): four 12-bit edge limits and two valid
 * bits add 7 bytes to OBSTACLE_PROFILE (33), NAVIGATOR_MESSAGE (34) and
 * BYPASS_CONTEXT (68). The 64-message queue grows by 512 bytes after rounding.
 * The result uses 10 width bits + 2 status bits. TELEMETRY_SNAPSHOT remains
 * 32 bytes; MQTT uses two reserved bits and remains 35 bytes. Motion fields
 * and their ranges are unchanged. The size/performance figures above describe
 * the earlier compression baseline, before this extension.
 *
 * Concurrency: common interrupt-preserving lock guards every shared packed
 * telemetry/map write and snapshot read. Shared flags use locked read/modify/
 * write and an aligned word read. Private state has one owner; encoder filters
 * have the ISR as sole writer (startup self-check/reset runs before IRQ use).
 */

/* Shared application flags: one aligned word, updated only through the API.
 * No-echo and link loss are diagnostic bits, not motor stop requests.
 * Bits 0..2 retain the existing motor fault values used in serial diagnostics.
 */
#define ROBOT_FAULT_STALL       (1UL << 0)
#define ROBOT_FAULT_ENCODER     (1UL << 1)
#define ROBOT_FAULT_ESTOP       (1UL << 2)
#define ROBOT_FAULT_SONAR       (1UL << 3)
#define ROBOT_FAULT_SERVO       (1UL << 4)
#define ROBOT_FAULT_IMU_READ    (1UL << 5)
#define ROBOT_FAULT_IMU_TILT    (1UL << 6)
#define ROBOT_STATUS_NO_ECHO    (1UL << 7)
#define ROBOT_FAULT_WIFI        (1UL << 8)
#define ROBOT_FAULT_MQTT        (1UL << 9)
#define ROBOT_STOP_ACK_REQUEST  (1UL << 10)
#define ROBOT_ENCODER_DEGRADED  (1UL << 11)
#define ROBOT_ESTOP_REQUEST     (1UL << 12)
#define ROBOT_FLAG_BITS         13U

IMPORT UINT robot_data_lock(void);
IMPORT void robot_data_unlock(UINT saved);
IMPORT UW robot_flags_get(void);
IMPORT void robot_flags_update(UW set, UW clear);
IMPORT void robot_flag_assign(UW mask, BOOL enabled);

/* Explicit little-endian bit streams for sonar rays, maps and MQTT.
 * Callers allocate ceil((offset + width)/8) bytes; width is 1..32.
 */
static inline UW robot_bits_get(const UB *bytes, UINT offset, UINT width)
{
    UW value = 0;
    for (UINT i = 0; i < width; ++i)
        value |= (UW)((bytes[(offset + i) / 8U] >> ((offset + i) % 8U)) & 1U) << i;
    return value;
}

static inline void robot_bits_set(UB *bytes, UINT offset, UINT width, UW value)
{
    for (UINT i = 0; i < width; ++i) {
        UINT bit = (offset + i) % 8U;
        UB *byte = &bytes[(offset + i) / 8U];
        *byte = (UB)((*byte & ~(1U << bit)) | (((value >> i) & 1U) << bit));
    }
}

static inline W robot_sign_extend(UW value, UINT width)
{
    UW sign = 1UL << (width - 1U);
    return (W)((value ^ sign) - sign);
}

static inline H robot_fixed10(float value, H minimum, H maximum)
{
    float scaled = value * 10.0f;
    if (!(scaled >= minimum)) return minimum;
    if (scaled >= maximum) return maximum;
    return (H)(scaled < 0.0f ? scaled - 0.5f : scaled + 0.5f);
}

typedef void (*PERIPHERAL_TASK)(INT stacd, void *exinf);
typedef struct {
    PERIPHERAL_TASK task;
    INT priority; /* OS API value; not an application state. */
    INT stacd;
    UW assprc;
    const char *name;
} TASK_START;

IMPORT ID barcode_mbfid;
IMPORT ID navigator_mbfid;
IMPORT ID motor_mbfid;
IMPORT ID ranger_flgid;

IMPORT UW time_us(void);
IMPORT void delay_us(UW delay);
IMPORT ER robot_tasks_start(void);

#endif
