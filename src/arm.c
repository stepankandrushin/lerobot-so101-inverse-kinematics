/* The follower arm driven in calibrated degrees.
 *
 * The servos keep LeRobot's calibration (Homing_Offset and limits as in the calibration given, which
 * so101_arm_check compares; nothing is written to EEPROM). At torque-on the two RAM settings the moves were
 * tuned with are written where a power cut has reset them (Acceleration 254, Torque_Limit no higher than
 * 500), every goal is set to where its joint stands, and nothing may move. Goals go out every tick_us on
 * a minimum-jerk profile, each sag degrees per N m of the model's holding torque past the wanted angle.
 * A joint more than lag_max behind its goal with under 0.5 degrees of progress in 0.3 s, or above
 * load_max for 0.3 s, or hal.stop(), stops the move: every goal goes to where its joint stands and the
 * call returns the error. A straight tip line goes through the least-loaded pose near the one before
 * every 2 mm (so101_ik_near). */
#include <stdio.h>
#include <string.h>
#include "so101ik_internal.h"
#include "sts.h"

/* The start pose: folded, lift and elbow against their stops, the gripper on the table in front of the
 * base. A calibration's ranges end at the stops, so the fold is found from them: lift and elbow this
 * far inside their range ends, the wrist flexed down this far. */
#define FOLD_LIFT ((so101_real)2)
#define FOLD_ELBOW ((so101_real)3)
#define FOLD_FLEX ((so101_real)76.5)
#define FOLDED_WITHIN ((so101_real)10)   /* degrees from the fold that count as folded at torque-on */

static const char *SHORT[SO101_NJ] = {"pan", "lift", "elbow", "wflex", "roll", "grip"};

#define TABLE (a->model.table_z)
#define MAXLINE 512            /* poses of one line, 2 mm apart: a metre */
#define WINDOW 0.3             /* s: the stall and overload rules' time */

static void say(so101_arm *a, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
#include <stdarg.h>
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

static double secs(so101_arm *a) { return (double)(a->hal.now_us(a->hal.ctx) - a->t0) / 1e6; }

void so101_arm_init(so101_arm *a, const so101_hal *hal, const so101_servo_cal cal[SO101_NJ]) {
    memset(a, 0, sizeof *a);
    a->hal = *hal;
    memcpy(a->cal, cal ? cal : SO101_DEFAULT_CAL, sizeof a->cal);
    so101_model_init(&a->model);
    so101_model_ranges(&a->model, a->cal);
    a->tick_us = 20000;
    a->lag_max = 7;
    a->load_max = 45;
    a->sag = (so101_real)3.7;
    a->acceleration = 254;
    a->torque_limit = 500;
    a->t0 = hal->now_us(hal->ctx);
    const so101_real *lo = a->model.lo, *hi = a->model.hi;
    so101_real flex = FOLD_FLEX < hi[SO101_FLEX] - 3 ? FOLD_FLEX : hi[SO101_FLEX] - 3;
    so101_real fold[SO101_NJ] = {0, lo[SO101_LIFT] + FOLD_LIFT, hi[SO101_ELBOW] - FOLD_ELBOW, flex, 0, 2};
    memcpy(a->start, fold, sizeof a->start);
    memcpy(a->goal, a->start, sizeof a->goal);
    strcpy(a->step, "connect");
}

/* The start pose with the wrist straight: unfolding goes through it, and folding. */
static void ready_pose(const so101_arm *a, so101_real out[SO101_NJ]) {
    memcpy(out, a->start, sizeof a->start);
    out[SO101_FLEX] = 0;
}

/* ---- calibrated degrees and the servos' steps (LeRobot's DEGREES and RANGE_0_100) ------------- */

static so101_real from_raw(const so101_servo_cal *c, int j, int raw) {
    so101_real lo = c->range_min, hi = c->range_max;
    if (j == SO101_GRIP) {
        so101_real v = raw < lo ? lo : raw > hi ? hi : raw;
        return (v - lo) / (hi - lo) * 100;
    }
    return (raw - (lo + hi) / 2) * 360 / (so101_real)4095;
}

static int to_raw(const so101_servo_cal *c, int j, so101_real v) {
    so101_real lo = c->range_min, hi = c->range_max;
    if (j == SO101_GRIP) {
        v = v < 0 ? 0 : v > 100 ? 100 : v;
        return (int)(v / 100 * (hi - lo) + lo);
    }
    so101_real r = v * 4095 / 360 + (lo + hi) / 2;
    if (r < 0) r = 0;
    if (r > 4095) r = 4095;
    return (int)r;
}

static void ids(const so101_arm *a, uint8_t out[SO101_NJ]) {
    for (int j = 0; j < SO101_NJ; j++) out[j] = (uint8_t)a->cal[j].id;
}

static int retry_sync_read(so101_arm *a, uint8_t addr, int n, uint8_t *data) {
    uint8_t id[SO101_NJ];
    ids(a, id);
    int rc = SO101_EIO;
    for (int k = 0; k < 4 && rc; k++) rc = so101_sts_sync_read(&a->hal, addr, n, id, SO101_NJ, data);
    return rc ? fail(a, SO101_EIO, "sync read of %d bytes at %d: no answer", n, addr) : SO101_OK;
}

int so101_arm_read(so101_arm *a, so101_real pos[SO101_NJ], so101_real load[SO101_NJ], so101_real ma[SO101_NJ]) {
    uint8_t d[SO101_NJ * 15];      /* Present_Position (56) to Present_Current (69, 70) */
    int rc = retry_sync_read(a, STS_PRESENT_POSITION, 15, d);
    if (rc) return rc;
    for (int j = 0; j < SO101_NJ; j++) {
        const uint8_t *p = d + 15 * j;
        if (pos) pos[j] = from_raw(&a->cal[j], j, so101_sts_signed(so101_sts_u16(p), 15));
        if (load) load[j] = (so101_real)so101_sts_signed(so101_sts_u16(p + 4), 10) / 10;
        if (ma) ma[j] = (so101_real)so101_sts_u16(p + 13) * (so101_real)6.5;
    }
    return SO101_OK;
}

static int read_raw_positions(so101_arm *a, uint8_t addr, int raw[SO101_NJ]) {
    uint8_t d[SO101_NJ * 2];
    int rc = retry_sync_read(a, addr, 2, d);
    if (rc) return rc;
    for (int j = 0; j < SO101_NJ; j++) raw[j] = so101_sts_signed(so101_sts_u16(d + 2 * j), 15);
    return SO101_OK;
}

static int write_raw_goals(so101_arm *a, const int raw[SO101_NJ]) {
    uint8_t id[SO101_NJ], d[SO101_NJ * 2];
    ids(a, id);
    for (int j = 0; j < SO101_NJ; j++) {
        d[2 * j] = (uint8_t)(raw[j] & 0xFF);
        d[2 * j + 1] = (uint8_t)((raw[j] >> 8) & 0x7F);
    }
    return so101_sts_sync_write(&a->hal, STS_GOAL_POSITION, 2, id, SO101_NJ, d) ? fail(a, SO101_EIO, "goal write failed") : SO101_OK;
}

/* The goals for the wanted angles: each loaded joint's sag degrees per N m of holding torque further. */
static int send_goals(so101_arm *a, const so101_real want[SO101_NJ]) {
    so101_real g[SO101_NJ];
    memcpy(g, want, sizeof g);
    if (a->sag > 0) {
        so101_pose p;
        so101_pose_eval(&a->model, want, &p);
        for (int j = SO101_LIFT; j <= SO101_ROLL; j++) g[j] += a->sag * p.torque[j];
    }
    int raw[SO101_NJ];
    for (int j = 0; j < SO101_NJ; j++) raw[j] = to_raw(&a->cal[j], j, g[j]);
    return write_raw_goals(a, raw);
}

int so101_arm_check(so101_arm *a) {
    for (int j = 0; j < SO101_NJ; j++) {
        uint8_t id = (uint8_t)a->cal[j].id;
        int ok = 0, ho = 0, lo = 0, hi = 0;
        for (int k = 0; k < 3 && !ok; k++)
            ok = !so101_sts_read2(&a->hal, id, STS_HOMING_OFFSET, &ho) && !so101_sts_read2(&a->hal, id, STS_MIN_LIMIT, &lo)
                 && !so101_sts_read2(&a->hal, id, STS_MAX_LIMIT, &hi);
        if (!ok) return fail(a, SO101_EIO, "servo %d (%s) does not answer", id, SHORT[j]);
        ho = so101_sts_signed(ho, 11);
        if (ho != a->cal[j].homing_offset || lo != a->cal[j].range_min || hi != a->cal[j].range_max)
            return fail(a, SO101_ECAL, "servo %d (%s): Homing_Offset %d, limits %d..%d, the calibration says %d, %d..%d: "
                        "restore the calibration first; nothing moved", id, SHORT[j], ho, lo, hi,
                        a->cal[j].homing_offset, a->cal[j].range_min, a->cal[j].range_max);
    }
    return SO101_OK;
}

static int write_both(so101_arm *a, uint8_t id, uint8_t reg, int len, int value) {
    int rc = SO101_EIO;
    for (int k = 0; k < 4 && rc; k++) rc = len == 1 ? so101_sts_write1(&a->hal, id, reg, value) : so101_sts_write2(&a->hal, id, reg, value);
    return rc;
}

int so101_arm_torque_off(so101_arm *a) {
    int bad = 0;
    for (int j = 0; j < SO101_NJ; j++) {
        bad |= write_both(a, (uint8_t)a->cal[j].id, STS_TORQUE_ENABLE, 1, 0);
        bad |= write_both(a, (uint8_t)a->cal[j].id, STS_LOCK, 1, 0);
    }
    a->torque = 0;
    say(a, "torque off");
    return bad ? fail(a, SO101_EIO, "torque off: a servo did not answer") : SO101_OK;
}

int so101_arm_torque_on(so101_arm *a) {
    char wrote[160] = "";
    const struct { uint8_t reg; int len, want, down_only; const char *name; } regs[2] = {
        {STS_ACCELERATION, 1, a->acceleration, 0, "Acceleration"}, {STS_TORQUE_LIMIT, 2, a->torque_limit, 1, "Torque_Limit"}};
    for (int r = 0; r < 2; r++) {
        for (int j = 0; j < SO101_NJ; j++) {
            uint8_t id = (uint8_t)a->cal[j].id, d[2] = {0, 0};
            int was = -1;
            for (int k = 0; k < 4 && was < 0; k++)
                if (!so101_sts_read(&a->hal, id, regs[r].reg, d, regs[r].len)) was = regs[r].len == 1 ? d[0] : so101_sts_u16(d);
            if (was < 0) return fail(a, SO101_EIO, "servo %d (%s) does not answer", id, SHORT[j]);
            if (regs[r].down_only ? was <= regs[r].want : was == regs[r].want) continue;
            if (write_both(a, id, regs[r].reg, regs[r].len, regs[r].want))
                return fail(a, SO101_EIO, "%s did not take on %s", regs[r].name, SHORT[j]);
            size_t n = strlen(wrote);
            snprintf(wrote + n, sizeof wrote - n, " %s %s %d", regs[r].name, SHORT[j], was);
        }
    }
    if (*wrote) say(a, "the servos' RAM was not as these moves need it (a power cut resets it), now written:%s", wrote);
    int raw[SO101_NJ], back[SO101_NJ], rc;
    if ((rc = read_raw_positions(a, STS_PRESENT_POSITION, raw))) return rc;
    if ((rc = write_raw_goals(a, raw))) return rc;
    if ((rc = read_raw_positions(a, STS_GOAL_POSITION, back))) return rc;
    if (memcmp(raw, back, sizeof raw)) {
        so101_arm_torque_off(a);
        return fail(a, SO101_EIO, "Goal_Position did not take; torque off");
    }
    for (int j = 0; j < SO101_NJ; j++) {
        if (write_both(a, (uint8_t)a->cal[j].id, STS_TORQUE_ENABLE, 1, 1) || write_both(a, (uint8_t)a->cal[j].id, STS_LOCK, 1, 1)) {
            so101_arm_torque_off(a);
            return fail(a, SO101_EIO, "torque on: %s did not answer; torque off", SHORT[j]);
        }
    }
    a->torque = 1;
    a->hal.sleep_us(a->hal.ctx, 500000);
    int now[SO101_NJ];
    if ((rc = read_raw_positions(a, STS_PRESENT_POSITION, now))) return rc;
    for (int j = 0; j < SO101_NJ; j++)
        if (now[j] - raw[j] > 30 || raw[j] - now[j] > 30) {
            so101_arm_torque_off(a);
            return fail(a, SO101_EMOVED, "%s moved %d steps at torque-on; torque off", SHORT[j], now[j] - raw[j]);
        }
    if ((rc = so101_arm_read(a, a->goal, NULL, NULL))) return rc;
    say(a, "torque on at pan %+.1f lift %+.1f elbow %+.1f wflex %+.1f roll %+.1f grip %.1f", (double)a->goal[0],
        (double)a->goal[1], (double)a->goal[2], (double)a->goal[3], (double)a->goal[4], (double)a->goal[5]);
    /* Folded now: this is the start pose home() comes back to. Otherwise the fold from the ranges, with
     * the wrist roll and the gripper as they are. */
    so101_real dl = a->goal[SO101_LIFT] - (a->model.lo[SO101_LIFT] + FOLD_LIFT);
    so101_real de = a->goal[SO101_ELBOW] - (a->model.hi[SO101_ELBOW] - FOLD_ELBOW);
    a->folded = dl < FOLDED_WITHIN && dl > -FOLDED_WITHIN && de < FOLDED_WITHIN && de > -FOLDED_WITHIN;
    if (a->folded) {
        memcpy(a->start, a->goal, sizeof a->start);
    } else {
        a->start[SO101_ROLL] = a->goal[SO101_ROLL];
        a->start[SO101_GRIP] = a->goal[SO101_GRIP];
    }
    return SO101_OK;
}

/* ---- moves ----------------------------------------------------------------------------------- */

static so101_real minjerk(so101_real t) {
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    return t * t * t * (10 - 15 * t + 6 * t * t);
}

static int freeze(so101_arm *a, int code, const char *why) {
    so101_real pos[SO101_NJ];
    if (so101_arm_read(a, pos, NULL, NULL) == SO101_OK) {
        memcpy(a->goal, pos, sizeof pos);
        int raw[SO101_NJ];
        for (int j = 0; j < SO101_NJ; j++) raw[j] = to_raw(&a->cal[j], j, pos[j]);
        write_raw_goals(a, raw);
    }
    say(a, "STOP in %s: %s; holding where it stands", a->step, why);
    return fail(a, code, "%s", why);
}

typedef void (*path_fn)(so101_arm *a, void *ctx, so101_real s, so101_real goal[SO101_NJ]);

#define HIST 64

/* Send path(s) for s on a minimum-jerk profile over duration seconds, then hold for settle. */
static int follow(so101_arm *a, path_fn path, void *ctx, double duration, double settle) {
    double th[HIST];
    so101_real ph[HIST][SO101_NJ], high[SO101_NJ];
    int h0 = 0, hn = 0;
    for (int j = 0; j < SO101_NJ; j++) high[j] = -1;
    uint64_t start = a->hal.now_us(a->hal.ctx), next = start;
    for (;;) {
        uint64_t now = a->hal.now_us(a->hal.ctx);
        double t = (double)(now - start) / 1e6;
        if (a->hal.stop && a->hal.stop(a->hal.ctx)) return freeze(a, SO101_ESTOP, "stop asked for");
        int done = t >= duration + settle;
        path(a, ctx, minjerk((so101_real)(t / duration)), a->goal);
        int rc = send_goals(a, a->goal);
        so101_real pos[SO101_NJ], load[SO101_NJ], ma[SO101_NJ];
        if (!rc) rc = so101_arm_read(a, pos, load, ma);
        if (rc) return freeze(a, rc, a->err);
        if (a->hal.tick) {
            so101_tick k = {secs(a), a->step};
            memcpy(k.goal, a->goal, sizeof k.goal);
            memcpy(k.pos, pos, sizeof k.pos);
            memcpy(k.load, load, sizeof k.load);
            memcpy(k.ma, ma, sizeof k.ma);
            so101_tip(&a->model, pos, k.tip, NULL);
            for (int i = 0; i < 3; i++) k.target[i] = a->have_target ? a->target[i] : (so101_real)NAN;
            a->hal.tick(a->hal.ctx, &k);
        }
        /* positions of the last WINDOW seconds */
        if (hn == HIST) h0 = (h0 + 1) % HIST, hn--;
        int k = (h0 + hn) % HIST;
        th[k] = t;
        memcpy(ph[k], pos, sizeof pos);
        hn++;
        while (hn > 1 && t - th[(h0 + 1) % HIST] >= WINDOW) h0 = (h0 + 1) % HIST, hn--;
        for (int j = 0; j < SO101_NJ; j++) {
            so101_real lag = a->goal[j] - pos[j];
            if (j != SO101_GRIP && RFABS(lag) > a->lag_max && t - th[h0] >= WINDOW && RFABS(pos[j] - ph[h0][j]) < (so101_real)0.5) {
                char why[120];
                snprintf(why, sizeof why, "%s stalled %+.1f degrees behind its goal, load %+.1f %%", SHORT[j], (double)lag,
                         (double)load[j]);
                return freeze(a, SO101_ESTALL, why);
            }
            if (RFABS(load[j]) > a->load_max) {
                if (high[j] < 0) high[j] = (so101_real)t;
                if (t - high[j] >= WINDOW) {
                    char why[120];
                    snprintf(why, sizeof why, "%s load %+.1f %% for %.1f s", SHORT[j], (double)load[j], WINDOW);
                    return freeze(a, SO101_ELOAD, why);
                }
            } else high[j] = -1;
        }
        if (done) return SO101_OK;
        /* the next tick on its own deadline, so a late wake-up does not push back the ones after it */
        next += a->tick_us;
        uint64_t after = a->hal.now_us(a->hal.ctx);
        if (next > after) a->hal.sleep_us(a->hal.ctx, (uint32_t)(next - after));
        else next = after;
    }
}

/* What the straight joint path from p to q runs into at 30 points: a joint past its range, an axis near
 * the table (so101_problems, a degree beyond the ranges), or a part below the table (so101_collides with no gap). */
static int clear_path(so101_arm *a, const so101_real p[SO101_NJ], const so101_real q[SO101_NJ], char *why, size_t n) {
    unsigned all = 0;
    int hit = 0;
    for (int i = 0; i < 30; i++) {
        so101_real s = (so101_real)i / 29, d[SO101_NJ];
        for (int j = 0; j < SO101_NJ; j++) d[j] = p[j] + s * (q[j] - p[j]);
        so101_pose pose;
        so101_pose_eval(&a->model, d, &pose);
        all |= so101_problems(&a->model, &pose, -1);
        if (!hit) hit = so101_collides(&a->model, d, 0);
    }
    if (!all && !hit) return 0;
    size_t k = 0;
    why[0] = 0;
    for (unsigned b = 1; b <= SO101_P_TIP_BASE; b <<= 1)
        if (all & b) k += snprintf(why + k, k < n ? n - k : 0, "%s%s", k ? "; " : "", so101_problem_name(b));
    if (hit && k < n) snprintf(why + k, n - k, "%s%s touch", k ? "; " : "", so101_collision_name(hit));
    return 1;
}

typedef struct { so101_real from[SO101_NJ], to[SO101_NJ]; } joints_t;

static void joints_path(so101_arm *a, void *ctx, so101_real s, so101_real goal[SO101_NJ]) {
    const joints_t *J = ctx;
    for (int j = 0; j < SO101_NJ; j++) goal[j] = J->from[j] + s * (J->to[j] - J->from[j]);
}

int so101_arm_move_joints(so101_arm *a, const so101_real target[SO101_NJ], so101_real speed, int check) {
    if (!a->torque) return fail(a, SO101_ESTATE, "the torque is off");
    joints_t J;
    memcpy(J.from, a->goal, sizeof J.from);
    memcpy(J.to, target, sizeof J.to);
    char why[160];
    if (check && clear_path(a, J.from, J.to, why, sizeof why))
        return fail(a, SO101_EPATH, "%s: the joint path runs into %s", a->step, why);
    so101_real dist = 0;
    char line[200];
    size_t k = 0;
    for (int j = 0; j < SO101_NJ; j++) {
        so101_real d = RFABS(J.to[j] - J.from[j]);
        if (d > dist) dist = d;
        if (d > (so101_real)0.05 && k < sizeof line)
            k += snprintf(line + k, sizeof line - k, " %s %+.1f>%+.1f", SHORT[j], (double)J.from[j], (double)J.to[j]);
    }
    say(a, "--- %s:%s", a->step, k ? line : " nothing to move");
    double duration = (double)(1.875 * dist / speed);
    return follow(a, joints_path, &J, duration > 0.3 ? duration : 0.3, 0.3);
}

int so101_arm_move_to(so101_arm *a, const so101_real xyz[3], const so101_real *pitch, so101_real speed) {
    so101_pose p;
    int rc = pitch ? so101_ik_pitch(&a->model, xyz, *pitch, a->goal[SO101_ROLL], a->goal[SO101_GRIP], NULL, &p)
                   : so101_ik(&a->model, xyz, a->goal[SO101_ROLL], a->goal[SO101_GRIP], &p);
    if (rc) return fail(a, rc, "no allowed pose reaches %.1f %.1f %.1f cm", (double)(100 * xyz[0]), (double)(100 * xyz[1]),
                        (double)(100 * xyz[2]));
    snprintf(a->step, sizeof a->step, "to %.1f %.1f %.1f pitch %+.1f", (double)(100 * p.tip[0]), (double)(100 * p.tip[1]),
             (double)(100 * p.tip[2]), (double)p.pitch);
    memcpy(a->target, p.tip, sizeof a->target);
    a->have_target = 1;
    rc = so101_arm_move_joints(a, p.deg, speed, 1);
    if (rc == SO101_OK) {
        a->pose = p;
        a->have_pose = 1;
    } else {
        a->have_pose = 0;
    }
    return rc;
}

/* ---- straight lines -------------------------------------------------------------------------- */

typedef struct { int n; so101_real (*q)[SO101_NJ]; so101_real (*tip)[3]; } through_t;
static so101_real LINE_Q[MAXLINE + 1][SO101_NJ], LINE_TIP[MAXLINE + 1][3];

static void through_path(so101_arm *a, void *ctx, so101_real s, so101_real goal[SO101_NJ]) {
    const through_t *T = ctx;
    int i = (int)(s * T->n);
    if (i > T->n - 1) i = T->n - 1;
    so101_real f = s * T->n - i;
    for (int j = 0; j < SO101_NJ; j++) goal[j] = T->q[i][j] + f * (T->q[i + 1][j] - T->q[i][j]);
    for (int k = 0; k < 3; k++) a->target[k] = T->tip[i][k] + f * (T->tip[i + 1][k] - T->tip[i][k]);
    a->have_target = 1;
}

/* Follow poses 0..n, linearly between them, on one minimum-jerk profile over duration. */
static int through(so101_arm *a, int n, double duration, const so101_pose *last) {
    so101_real jump = 0;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < 4; j++) {
            so101_real d = RFABS(LINE_Q[i + 1][j] - LINE_Q[i][j]);
            if (d > jump) jump = d;
        }
    if (jump > 3) return fail(a, SO101_EPATH, "%s: the poses jump %.1f degrees between neighbours", a->step, (double)jump);
    say(a, "--- %s: %d poses, pitch %+.1f > %+.1f, cost %.3f > %.3f", a->step, n, (double)a->pose.pitch,
        (double)last->pitch, (double)a->pose.cost, (double)last->cost);
    through_t T = {n, LINE_Q, LINE_TIP};
    int rc = follow(a, through_path, &T, duration > 0.3 ? duration : 0.3, 0.3);
    if (rc == SO101_OK) a->pose = *last;
    else a->have_pose = 0;
    return rc;
}

int so101_arm_line_to(so101_arm *a, const so101_real xyz[3], so101_real speed) {
    if (!a->torque) return fail(a, SO101_ESTATE, "the torque is off");
    if (!a->have_pose) return fail(a, SO101_ESTATE, "a line goes on from a move_to");
    const so101_real *s = a->pose.tip;
    so101_real d[3] = {xyz[0] - s[0], xyz[1] - s[1], xyz[2] - s[2]};
    so101_real len = RSQRT(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    int n = (int)(len / (so101_real)0.002 + (so101_real)0.999);
    if (n < 1) n = 1;
    if (n > MAXLINE) return fail(a, SO101_EPATH, "a line of %.0f cm is longer than %d poses", (double)(100 * len), MAXLINE);
    so101_pose p = a->pose;
    memcpy(LINE_Q[0], p.deg, sizeof LINE_Q[0]);
    memcpy(LINE_TIP[0], p.tip, sizeof LINE_TIP[0]);
    for (int k = 1; k <= n; k++) {
        so101_real c[3] = {s[0] + d[0] * k / n, s[1] + d[1] * k / n, s[2] + d[2] * k / n};
        so101_pose q;
        if (so101_ik_near(&a->model, c, &p, &q))
            return fail(a, SO101_EUNREACHABLE, "no allowed pose reaches %.1f %.1f %.1f cm on the way", (double)(100 * c[0]),
                        (double)(100 * c[1]), (double)(100 * c[2]));
        p = q;
        memcpy(LINE_Q[k], p.deg, sizeof LINE_Q[k]);
        memcpy(LINE_TIP[k], p.tip, sizeof LINE_TIP[k]);
    }
    snprintf(a->step, sizeof a->step, "line to %.1f %.1f %.1f", (double)(100 * xyz[0]), (double)(100 * xyz[1]),
             (double)(100 * xyz[2]));
    return through(a, n, (double)(1.875 * len / speed), &p);
}

int so101_arm_move_by(so101_arm *a, const so101_real dxyz[3], so101_real speed) {
    if (!a->have_pose) return fail(a, SO101_ESTATE, "a move by goes on from a move_to");
    so101_real to[3] = {a->pose.tip[0] + dxyz[0], a->pose.tip[1] + dxyz[1], a->pose.tip[2] + dxyz[2]};
    return so101_arm_line_to(a, to, speed);
}

/* ---- the start pose -------------------------------------------------------------------------- */

int so101_arm_unfold(so101_arm *a) {
    a->have_target = a->have_pose = 0;
    so101_real wrist[SO101_NJ], ready[SO101_NJ];
    ready_pose(a, ready);
    memcpy(wrist, a->goal, sizeof wrist);
    wrist[SO101_FLEX] = ready[SO101_FLEX];
    strcpy(a->step, "wrist up");
    int rc = so101_arm_move_joints(a, wrist, 40, 0);
    if (rc) return rc;
    strcpy(a->step, "ready");
    return so101_arm_move_joints(a, ready, 40, 1);
}

/* The tip straight up to height z, without the table checks: each 1 mm step the least change of lift,
 * elbow and wrist flex that raises it (damped least squares on their Jacobian). */
static int rise(so101_arm *a, so101_real z, so101_real speed) {
    so101_real deg[SO101_NJ], start[3], tip[3];
    memcpy(deg, a->goal, sizeof deg);
    so101_tip(&a->model, deg, start, NULL);
    memcpy(LINE_Q[0], deg, sizeof LINE_Q[0]);
    memcpy(LINE_TIP[0], start, sizeof LINE_TIP[0]);
    int n = 0, edge = 0;
    memcpy(tip, start, sizeof tip);
    while (tip[2] < z && n < MAXLINE) {
        so101_real J[3][3], want[3] = {start[0] - tip[0], start[1] - tip[1], (so101_real)0.001};
        for (int c = 0; c < 3; c++) {
            so101_real d2[SO101_NJ], t2[3];
            memcpy(d2, deg, sizeof d2);
            d2[SO101_LIFT + c] += (so101_real)0.01;
            so101_tip(&a->model, d2, t2, NULL);
            for (int r = 0; r < 3; r++) J[r][c] = (t2[r] - tip[r]) / (so101_real)0.01;
        }
        /* step = J^T (J J^T + l I)^-1 want */
        so101_real A[3][3], tr = 0;
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) {
                A[r][c] = 0;
                for (int k = 0; k < 3; k++) A[r][c] += J[r][k] * J[c][k];
            }
        for (int r = 0; r < 3; r++) tr += A[r][r];
        for (int r = 0; r < 3; r++) A[r][r] += (so101_real)1e-6 * tr;
        so101_real det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
                      + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
        if (RFABS(det) < (so101_real)1e-30) break;
        so101_real inv[3][3] = {{A[1][1] * A[2][2] - A[1][2] * A[2][1], A[0][2] * A[2][1] - A[0][1] * A[2][2], A[0][1] * A[1][2] - A[0][2] * A[1][1]},
                             {A[1][2] * A[2][0] - A[1][0] * A[2][2], A[0][0] * A[2][2] - A[0][2] * A[2][0], A[0][2] * A[1][0] - A[0][0] * A[1][2]},
                             {A[1][0] * A[2][1] - A[1][1] * A[2][0], A[0][1] * A[2][0] - A[0][0] * A[2][1], A[0][0] * A[1][1] - A[0][1] * A[1][0]}};
        so101_real y[3];
        for (int r = 0; r < 3; r++) y[r] = (inv[r][0] * want[0] + inv[r][1] * want[1] + inv[r][2] * want[2]) / det;
        for (int c = 0; c < 3; c++) deg[SO101_LIFT + c] += J[0][c] * y[0] + J[1][c] * y[1] + J[2][c] * y[2];
        for (int c = 0; c < 3; c++)
            if (deg[SO101_LIFT + c] < a->model.lo[SO101_LIFT + c] || deg[SO101_LIFT + c] > a->model.hi[SO101_LIFT + c]) edge = 1;
        if (edge) break;
        so101_tip(&a->model, deg, tip, NULL);
        n++;
        memcpy(LINE_Q[n], deg, sizeof LINE_Q[n]);
        memcpy(LINE_TIP[n], tip, sizeof LINE_TIP[n]);
    }
    if (n > 0) {
        snprintf(a->step, sizeof a->step, "up to %.0f cm above the table", (double)(100 * (tip[2] - TABLE)));
        so101_pose last;
        so101_pose_eval(&a->model, LINE_Q[n], &last);
        a->pose = last;
        int rc = through(a, n, (double)(1.875 * (tip[2] - start[2]) / speed), &last);
        if (rc) return rc;
    }
    if (edge) return fail(a, SO101_EPATH, "rising, a joint reached its range end at %.1f cm", (double)(100 * (tip[2] - TABLE)));
    return SO101_OK;
}

int so101_arm_home(so101_arm *a) {
    if (!a->torque) return fail(a, SO101_ESTATE, "the torque is off");
    a->have_target = 0;
    so101_real tip[3];
    so101_tip(&a->model, a->goal, tip, NULL);
    int rc;
    if (tip[2] < TABLE + (so101_real)0.05 && (rc = rise(a, TABLE + (so101_real)0.06, (so101_real)0.01))) return rc;
    char why[160];
    so101_real ready[SO101_NJ];
    ready_pose(a, ready);
    if (clear_path(a, a->goal, ready, why, sizeof why)) {
        so101_tip(&a->model, a->goal, tip, NULL);
        so101_real up[3] = {tip[0], tip[1], tip[2] > TABLE + (so101_real)0.15 ? tip[2] : TABLE + (so101_real)0.15};
        if ((rc = so101_arm_move_to(a, up, NULL, 30))) return rc;
    }
    a->have_target = a->have_pose = 0;
    strcpy(a->step, "ready");
    if ((rc = so101_arm_move_joints(a, ready, 40, 1))) return rc;
    strcpy(a->step, "start pose");
    if ((rc = so101_arm_move_joints(a, a->start, 30, 0))) return rc;
    a->hal.sleep_us(a->hal.ctx, 300000);
    return so101_arm_torque_off(a);
}
