/* Six simulated STS servos behind so101_hal (so101ik.h, so101_sim): the packets of src/sts.c answered from a
 * control table per servo, the servos moved between packets on a clock of their own. */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "so101ik.h"

#define KP 20.0            /* steps/s per step from the goal */
#define VMAX 2300.0        /* steps/s */
#define BYTE_US 10         /* 1 Mbaud */

static int u16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static void put16(uint8_t *p, int v) { p[0] = (uint8_t)(v & 0xFF), p[1] = (uint8_t)((v >> 8) & 0xFF); }
static int sign_mag(int v, int bit) { return v < 0 ? (1 << bit) | -v : v; }

static void degrees(const so101_sim *s, const double *raw, so101_real deg[SO101_NJ]) {
    for (int j = 0; j < SO101_NJ; j++) {
        double lo = s->cal[j].range_min, hi = s->cal[j].range_max;
        deg[j] = (so101_real)(j == SO101_GRIP ? (raw[j] - lo) / (hi - lo) * 100 : (raw[j] - (lo + hi) / 2) * 360 / 4095);
    }
}

/* The servos from s->now to t, a millisecond at a time, and their readings into the tables. */
static void advance(so101_sim *s, uint64_t t) {
    while (s->now < t) {
        uint64_t step = t - s->now < 1000 ? t - s->now : 1000;
        double dt = step / 1e6, next[SO101_NJ];
        for (int j = 0; j < SO101_NJ; j++) {
            next[j] = s->pos[j];
            s->held[j] = 0;
            if (!s->reg[j][40]) continue;                    /* Torque_Enable */
            double err = (double)u16(s->reg[j] + 42) - s->pos[j];
            if (fabs(err) <= SO101_SIM_DEAD) continue;
            double v = KP * err;
            v = v > VMAX ? VMAX : v < -VMAX ? -VMAX : v;
            next[j] += v * dt;
            if (s->lo_block[j] >= 0 && next[j] < s->lo_block[j]) next[j] = s->lo_block[j], s->held[j] = 1;
            if (s->hi_block[j] >= 0 && next[j] > s->hi_block[j]) next[j] = s->hi_block[j], s->held[j] = 1;
        }
        so101_real deg[SO101_NJ], tip[3], was[3];
        degrees(s, next, deg);
        so101_tip(&s->m, deg, tip, NULL);
        degrees(s, s->pos, deg);
        so101_tip(&s->m, deg, was, NULL);
        if (tip[2] < s->floor_z && tip[2] < was[2]) {        /* the floor holds the whole arm back */
            for (int j = 0; j < SO101_NJ; j++) s->held[j] |= next[j] != s->pos[j];
        } else {
            memcpy(s->pos, next, sizeof next);
        }
        s->now += step;
    }
    for (int j = 0; j < SO101_NJ; j++) {
        int err = u16(s->reg[j] + 42) - (int)lround(s->pos[j]);
        int load = s->reg[j][40] && abs(err) > 2 ? err * (s->held[j] ? 8 : 1) : 0;   /* tenths of a percent */
        load = load > 1000 ? 1000 : load < -1000 ? -1000 : load;
        put16(s->reg[j] + 56, (int)lround(s->pos[j]));       /* Present_Position */
        put16(s->reg[j] + 60, sign_mag(load, 10));           /* Present_Load */
        put16(s->reg[j] + 69, abs(load) / 2);                /* Present_Current, 6.5 mA steps */
    }
}

static void answer(so101_sim *s, int id, const uint8_t *data, int n) {
    if (s->rx_n + n + 6 > (int)sizeof s->rx) return;
    uint8_t *b = s->rx + s->rx_n;
    b[0] = b[1] = 0xFF;
    b[2] = (uint8_t)id;
    b[3] = (uint8_t)(n + 2);
    b[4] = 0;
    unsigned sum = b[2] + b[3];
    for (int i = 0; i < n; i++) sum += b[5 + i] = data[i];
    b[5 + n] = (uint8_t)~sum;
    s->rx_n += n + 6;
}

static int servo(const so101_sim *s, int id) {
    for (int j = 0; j < SO101_NJ; j++)
        if (s->cal[j].id == id) return j;
    return -1;
}

static int sim_write(void *c, const uint8_t *b, int n) {
    so101_sim *s = c;
    s->now += BYTE_US * (uint64_t)n;
    advance(s, s->now);
    if (n < 6) return n;
    int id = b[2], inst = b[4], np = b[3] - 2, j = servo(s, id);
    const uint8_t *p = b + 5;
    if (inst == 0x01 && j >= 0) answer(s, id, NULL, 0);                              /* ping */
    else if (inst == 0x02 && j >= 0) answer(s, id, s->reg[j] + p[0], p[1]);          /* read */
    else if (inst == 0x03 && j >= 0) {                                               /* write */
        memcpy(s->reg[j] + p[0], p + 1, np - 1);
        answer(s, id, NULL, 0);
    } else if (inst == 0x82) {                                                       /* sync read */
        for (int i = 2; i < np; i++)
            if ((j = servo(s, p[i])) >= 0) answer(s, p[i], s->reg[j] + p[0], p[1]);
    } else if (inst == 0x83) {                                                       /* sync write */
        for (int k = 2; k + p[1] < np; k += p[1] + 1)
            if ((j = servo(s, p[k])) >= 0) memcpy(s->reg[j] + p[0], p + k + 1, p[1]);
    }
    return n;
}

static int sim_read(void *c, uint8_t *b, int n, uint32_t timeout) {
    so101_sim *s = c;
    int k = s->rx_n - s->rx_at < n ? s->rx_n - s->rx_at : n;
    if (k <= 0) {
        advance(s, s->now + timeout);
        return 0;
    }
    memcpy(b, s->rx + s->rx_at, k);
    s->rx_at += k;
    s->now += BYTE_US * (uint64_t)k;
    if (s->rx_at == s->rx_n) s->rx_at = s->rx_n = 0;
    return k;
}

static void sim_drain(void *c) {
    so101_sim *s = c;
    s->rx_at = s->rx_n = 0;
}
static uint64_t sim_now(void *c) { return ((so101_sim *)c)->now; }
static void sim_sleep(void *c, uint32_t us) {
    so101_sim *s = c;
    advance(s, s->now + us);
}

void so101_sim_init(so101_sim *s, const so101_servo_cal cal[SO101_NJ], const so101_real deg[SO101_NJ]) {
    memset(s, 0, sizeof *s);
    memcpy(s->cal, cal ? cal : SO101_DEFAULT_CAL, sizeof s->cal);
    so101_model_init(&s->m);
    so101_model_ranges(&s->m, s->cal);
    s->floor_z = -1e9;
    for (int j = 0; j < SO101_NJ; j++) {
        const so101_servo_cal *k = &s->cal[j];
        put16(s->reg[j] + 9, k->range_min);                  /* Min_Position_Limit */
        put16(s->reg[j] + 11, k->range_max);                 /* Max_Position_Limit */
        put16(s->reg[j] + 31, sign_mag(k->homing_offset, 11));
        put16(s->reg[j] + 48, 1000);                         /* Torque_Limit as after a power cut */
        double lo = k->range_min, hi = k->range_max;
        s->pos[j] = j == SO101_GRIP ? lo + deg[j] / 100 * (hi - lo) : deg[j] * 4095 / 360 + (lo + hi) / 2;
        s->lo_block[j] = s->hi_block[j] = -1;
    }
    advance(s, 0);
}

void so101_hal_sim(so101_hal *hal, so101_sim *s) {
    so101_hal h = {sim_write, sim_read, sim_drain, sim_now, sim_sleep, NULL, NULL, NULL, s};
    *hal = h;
}
