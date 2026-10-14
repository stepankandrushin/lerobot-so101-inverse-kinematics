/* Fast runs, driven: a run planned whole (src/path.c) replayed tick by tick, the servos at full torque and
 * without their own acceleration ramp.
 *
 * With the usual Torque_Limit 500 a servo turns at half its speed (165 instead of 290 degrees/s), and its own
 * ramp (Acceleration 254) adds a delay that grows with the speed; at 1000 and 0 a streamed goal is followed
 * as a lag of 60 ms behind 22 ms (agents/measurements.md, Fast runs). So the goal sent at time t is where
 * the joint should be at t + dead, plus lag times its speed there, plus the sag compensation the slow moves
 * use. so101_arm_fast puts the two settings on for a run and back after it; a run killed outright leaves
 * them, and so101_arm_torque_on writes them back. */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "so101ik_internal.h"
#include "sts.h"

#define OFF_FOR 0.15       /* s a joint may be err_max from where it should be */
#define FULL 99            /* % load */
#define FULL_FOR 0.4       /* s */

static const char *SHORT[SO101_NJ] = {"pan", "lift", "elbow", "wflex", "roll", "grip"};

static void say(so101_arm *a, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void say(so101_arm *a, const char *fmt, ...) {
    if (!a->hal.log) return;
    char b[400];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    a->hal.log(a->hal.ctx, b);
}

static int fail(so101_arm *a, int code, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
static int fail(so101_arm *a, int code, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(a->err, sizeof a->err, fmt, ap);
    va_end(ap);
    return code;
}

/* calibrated degrees (the gripper 0-100) as a servo's steps, LeRobot's DEGREES and RANGE_0_100 */
static int to_raw(const so101_servo_cal *c, int j, so101_real v) {
    so101_real lo = c->range_min, hi = c->range_max;
    if (j == SO101_GRIP) {
        v = v < 0 ? 0 : v > 100 ? 100 : v;
        return (int)(v / 100 * (hi - lo) + lo);
    }
    so101_real r = v * 4095 / 360 + (lo + hi) / 2;
    return r < 0 ? 0 : r > 4095 ? 4095 : (int)r;
}

static int write_goals(so101_arm *a, const so101_real deg[SO101_NJ]) {
    uint8_t id[SO101_NJ], d[SO101_NJ * 2];
    for (int j = 0; j < SO101_NJ; j++) {
        int raw = to_raw(&a->cal[j], j, deg[j]);
        id[j] = (uint8_t)a->cal[j].id;
        d[2 * j] = (uint8_t)(raw & 0xFF);
        d[2 * j + 1] = (uint8_t)((raw >> 8) & 0x7F);
    }
    return so101_sts_sync_write(&a->hal, STS_GOAL_POSITION, 2, id, SO101_NJ, d) ? fail(a, SO101_EIO, "goal write failed")
                                                                               : SO101_OK;
}

static int freeze(so101_arm *a, int code, const char *why) {
    so101_real pos[SO101_NJ];
    if (so101_arm_read(a, pos, NULL, NULL) == SO101_OK) {
        memcpy(a->goal, pos, sizeof pos);
        write_goals(a, pos);
    }
    say(a, "STOP in %s: %s; holding where it stands", a->step, why);
    return fail(a, code, "%s", why);
}

int so101_arm_fast(so101_arm *a, const so101_fast *f, int on) {
    int limit = on ? f->torque_limit : a->torque_limit, acc = on ? f->acceleration : a->acceleration;
    for (int j = 0; j < SO101_NA; j++) {
        uint8_t id = (uint8_t)a->cal[j].id;
        int rc = SO101_EIO, rc2 = SO101_EIO;
        for (int k = 0; k < 4 && rc; k++) rc = so101_sts_write2(&a->hal, id, STS_TORQUE_LIMIT, limit);
        for (int k = 0; k < 4 && rc2; k++) rc2 = so101_sts_write1(&a->hal, id, STS_ACCELERATION, acc);
        if (rc || rc2) return fail(a, SO101_EIO, "Torque_Limit and Acceleration did not take on %s", SHORT[j]);
    }
    say(a, on ? "fast: Torque_Limit %d and Acceleration %d on the arm joints" : "back to Torque_Limit %d and Acceleration %d",
        limit, acc);
    return SO101_OK;
}

/* joint j of the ticks q at tick x (a fraction), the ends held */
static so101_real at_tick(const so101_real (*q)[SO101_NJ], int n, so101_real x, int j) {
    if (x <= 0) return q[0][j];
    if (x >= n - 1) return q[n - 1][j];
    int i = (int)x;
    return q[i][j] + (x - i) * (q[i + 1][j] - q[i][j]);
}

/* its change per tick there, as numpy's gradient takes it */
static so101_real rate_tick(const so101_real (*q)[SO101_NJ], int n, int i, int j) {
    if (i <= 0) return q[1][j] - q[0][j];
    if (i >= n - 1) return q[n - 1][j] - q[n - 2][j];
    return (q[i + 1][j] - q[i - 1][j]) / 2;
}

static so101_real rate_at(const so101_real (*q)[SO101_NJ], int n, so101_real x, int j) {
    if (x <= 0) return rate_tick(q, n, 0, j);
    if (x >= n - 1) return rate_tick(q, n, n - 1, j);
    int i = (int)x;
    return rate_tick(q, n, i, j) + (x - i) * (rate_tick(q, n, i + 1, j) - rate_tick(q, n, i, j));
}

int so101_arm_play(so101_arm *a, const so101_fast *f, const so101_real (*q)[SO101_NJ], int n, const so101_real (*tip)[3],
                   const so101_real (*fix)[SO101_NA], so101_real rest, so101_real err_max, so101_real (*pos)[SO101_NJ],
                   so101_real (*load)[SO101_NJ]) {
    if (!a->torque) return fail(a, SO101_ESTATE, "the torque is off");
    if (n < 2) return fail(a, SO101_EPATH, "a run of %d ticks", n);
    so101_real worst = 0;
    for (int j = 0; j < SO101_NA; j++)
        if (RFABS(q[0][j] - a->goal[j]) > worst) worst = RFABS(q[0][j] - a->goal[j]);
    if (worst > 5) return fail(a, SO101_ESTATE, "%s: the run starts %.1f degrees from where the arm stands", a->step, (double)worst);
    a->have_pose = a->have_target = 0;
    double dt = (double)f->dt, end = (n - 1) * dt + (double)rest, off = -1, full = -1;
    so101_real ahead = f->dead / f->dt;
    say(a, "--- %s: %d ticks, %.2f s", a->step, n, (n - 1) * dt);
    int filled = -1;
    so101_real p[SO101_NJ], ld[SO101_NJ], ma[SO101_NJ];
    uint64_t start = a->hal.now_us(a->hal.ctx);
    for (int k = 0;; k++) {
        double t = (double)(a->hal.now_us(a->hal.ctx) - start) / 1e6;
        if (a->hal.stop && a->hal.stop(a->hal.ctx)) return freeze(a, SO101_ESTOP, "stop asked for");
        so101_real x = (so101_real)(t / dt), want[SO101_NJ], goal[SO101_NJ];
        for (int j = 0; j < SO101_NJ; j++) want[j] = goal[j] = at_tick(q, n, x, j);
        for (int j = 0; j < SO101_NA; j++)
            goal[j] = at_tick(q, n, x + ahead, j) + f->lag * rate_at(q, n, x + ahead, j) / f->dt;
        if (a->sag > 0) {
            so101_pose pp;
            so101_pose_eval(&a->model, want, &pp);
            for (int j = SO101_LIFT; j <= SO101_ROLL; j++) goal[j] += a->sag * pp.torque[j];
        }
        for (int j = 0; j < SO101_NA; j++) {
            if (fix) {
                so101_real xi = x < 0 ? 0 : x > n - 1 ? (so101_real)(n - 1) : x;
                int i = (int)xi < n - 1 ? (int)xi : n - 2;
                goal[j] += fix[i][j] + (xi - i) * (fix[i + 1][j] - fix[i][j]);
            }
            goal[j] = goal[j] < a->model.lo[j] ? a->model.lo[j] : goal[j] > a->model.hi[j] ? a->model.hi[j] : goal[j];
        }
        int rc = write_goals(a, goal);
        memcpy(a->goal, want, sizeof want);
        if (!rc) rc = so101_arm_read(a, p, ld, ma);
        if (rc) return freeze(a, rc, a->err);
        if (a->hal.tick) {
            so101_tick tk = {(double)(a->hal.now_us(a->hal.ctx) - a->t0) / 1e6, a->step};
            memcpy(tk.goal, want, sizeof tk.goal);
            memcpy(tk.pos, p, sizeof tk.pos);
            memcpy(tk.load, ld, sizeof tk.load);
            memcpy(tk.ma, ma, sizeof tk.ma);
            so101_tip(&a->model, p, tk.tip, NULL);
            for (int c = 0; c < 3; c++) tk.target[c] = (so101_real)NAN;
            if (tip) {
                so101_real xi = x < 0 ? 0 : x > n - 1 ? (so101_real)(n - 1) : x;
                int i = (int)xi < n - 1 ? (int)xi : n - 2;
                for (int c = 0; c < 3; c++) tk.target[c] = tip[i][c] + (xi - i) * (tip[i + 1][c] - tip[i][c]);
            }
            a->hal.tick(a->hal.ctx, &tk);
        }
        /* what was read, at the tick nearest its time */
        int i = (int)(x + (so101_real)0.5);
        if (i > n - 1) i = n - 1;
        while (filled < i) {
            filled++;
            if (pos) memcpy(pos[filled], p, sizeof p);
            if (load) memcpy(load[filled], ld, sizeof ld);
        }
        int far = -1, heavy = -1;
        so101_real most = 0, most_load = 0;
        for (int j = 0; j < SO101_NA; j++) {
            if (RFABS(want[j] - p[j]) > most) most = RFABS(want[j] - p[j]), far = j;
            if (RFABS(ld[j]) > most_load) most_load = RFABS(ld[j]), heavy = j;
        }
        if (most > err_max) {
            if (off < 0) off = t;
            if (t - off >= OFF_FOR) {
                char why[120];
                snprintf(why, sizeof why, "%s is %+.1f degrees from where it should be, load %+.0f %%", SHORT[far],
                         (double)(want[far] - p[far]), (double)ld[far]);
                return freeze(a, SO101_ESTALL, why);
            }
        } else off = -1;
        if (most_load >= FULL) {
            if (full < 0) full = t;
            if (t - full >= FULL_FOR) {
                char why[120];
                snprintf(why, sizeof why, "%s at full load for %.1f s", SHORT[heavy], FULL_FOR);
                return freeze(a, SO101_ELOAD, why);
            }
        } else full = -1;
        if (t >= end) break;
        /* the next tick on its deadline: a late one sends the goal of its own time, and the ones after catch up */
        uint64_t next = start + (uint64_t)((k + 1) * dt * 1e6), now = a->hal.now_us(a->hal.ctx);
        if (next > now) a->hal.sleep_us(a->hal.ctx, (uint32_t)(next - now));
    }
    while (filled < n - 1) {
        filled++;
        if (pos) memcpy(pos[filled], p, sizeof p);
        if (load) memcpy(load[filled], ld, sizeof ld);
    }
    memcpy(a->goal, q[n - 1], sizeof a->goal);
    return SO101_OK;
}
