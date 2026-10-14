/* Feetech STS servos (the STS3215 of the SO-101) on a half-duplex bus: protocol 0 of their SDK, values
 * little-endian. The registers are LeRobot's (lerobot/motors/feetech/tables.py). */
#ifndef SO101_STS_H
#define SO101_STS_H

#include <stdint.h>
#include "so101ik.h"

enum {
    STS_MIN_LIMIT = 9, STS_MAX_LIMIT = 11, STS_MAX_TORQUE = 16, STS_HOMING_OFFSET = 31,
    STS_TORQUE_ENABLE = 40, STS_ACCELERATION = 41, STS_GOAL_POSITION = 42, STS_TORQUE_LIMIT = 48, STS_LOCK = 55,
    STS_PRESENT_POSITION = 56, STS_PRESENT_LOAD = 60, STS_PRESENT_CURRENT = 69,
};

int so101_sts_ping(so101_hal *h, uint8_t id);
int so101_sts_read(so101_hal *h, uint8_t id, uint8_t addr, uint8_t *data, int n);
int so101_sts_write(so101_hal *h, uint8_t id, uint8_t addr, const uint8_t *data, int n);        /* waits for the status */
int so101_sts_write1(so101_hal *h, uint8_t id, uint8_t addr, int value);
int so101_sts_write2(so101_hal *h, uint8_t id, uint8_t addr, int value);
int so101_sts_read2(so101_hal *h, uint8_t id, uint8_t addr, int *value);
/* n bytes at addr of every servo in ids, in one packet; no answer */
int so101_sts_sync_write(so101_hal *h, uint8_t addr, int n, const uint8_t *ids, int count, const uint8_t *data);
/* n bytes at addr of every servo in ids: data holds count * n bytes */
int so101_sts_sync_read(so101_hal *h, uint8_t addr, int n, const uint8_t *ids, int count, uint8_t *data);

static inline int so101_sts_u16(const uint8_t *p) { return p[0] | (p[1] << 8); }
/* sign and magnitude, the sign at bit */
static inline int so101_sts_signed(int v, int bit) { return (v >> bit) & 1 ? -(v & ((1 << bit) - 1)) : v & ((1 << bit) - 1); }

#endif
