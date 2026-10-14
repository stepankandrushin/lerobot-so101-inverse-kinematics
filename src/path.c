/* Fast runs, planned: a path of the tip worked out whole before the arm moves, so that the arm then only
 * replays numbers (src/fast.c) as fast as the servos go.
 *
 * 1. so101_path_poses: the joints at every point of the path (2 mm apart), each pose solved from the one
 *    before, at a pitch given for each point or in the least-loaded pose near the one before.
 *    so101_path_fastest searches the pitch, the same smooth function of the place along a closed path on
 *    every lap, for the shortest lap.
 * 2. so101_path_time: the fastest timing of that joint path with every joint under vmax and amax.
 * 3. so101_path_sample: the run as joints every dt, its speed along the path smoothed over 40 ms.
 * 4. so101_run_check: the run against the model as it will be driven.
 * 5. so101_path_learn: what the joints were off in one replay corrects the goals of the next.
 * so101_unfold_path is the way from the folded start pose to a run's first point at the same limits, and
 * backwards the way home; so101_unfold_clear checks it with the joints a few degrees off.
 *
 * The limits are measured on 12 V STS3215 servos at Torque_Limit 1000 and Acceleration 0
 * (agents/measurements.md, Fast runs): a goal step drives a joint at 283-304 degrees/s, 254 against gravity,
 * and a joint follows a goal streamed at 100 Hz as a first-order lag of 60 ms behind a dead time of 22 ms. */
#include <stdio.h>
#include <string.h>
#include "so101ik_internal.h"

#define BIG ((so101_real)1e30)
#define JUMP ((so101_real)3)        /* degrees a joint may move between two neighbouring points of a path */
#define CLOSE ((so101_real)0.05)    /* degrees: a closed path's last pose against its first */

static const char *NAMES[SO101_NJ] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};

void so101_fast_init(so101_fast *f) {
    f->vmax = 220;
    f->amax = 1500;
    f->top = 295;
    f->dt = (so101_real)0.01;
    f->lag = (so101_real)0.060;
    f->dead = (so101_real)0.022;
    f->smooth = (so101_real)0.04;
    f->gap = (so101_real)0.02;
    f->err_max = 12;
    f->tube = 6;
    f->torque_limit = 1000;
    f->acceleration = 0;
}

/* ---- poses along a path ------------------------------------------------------------------------- */

int so101_path_poses(so101_model *m, const so101_real (*xyz)[3], int n, const so101_real *pitch, so101_real roll,
                     so101_real grip, int branch, so101_real gap, so101_real (*q)[SO101_NJ], int *at) {
    if (n < 1) return SO101_EPATH;
    so101_real d0 = xyz[n - 1][0] - xyz[0][0], d1 = xyz[n - 1][1] - xyz[0][1], d2 = xyz[n - 1][2] - xyz[0][2];
    int closed = n > 2 && d0 * d0 + d1 * d1 + d2 * d2 < (so101_real)1e-12;
    so101_pose p, prev;
    /* In the least-loaded poses a closed path may end in another pose than it began with (the least-loaded
     * pose at the first point is not always the one a path comes back to): a second lap then starts from
     * where the first ended. */
    for (int lap = 0; lap < (closed && !pitch ? 2 : 1); lap++) {
        for (int i = lap ? 1 : 0; i < n; i++) {
            if (at) *at = i;
            int rc;
            if (i == 0)
                rc = pitch ? so101_ik_branch(m, xyz[0], pitch[0], roll, grip, branch < 0 ? -1 : 1, &p)
                           : so101_ik(m, xyz[0], roll, grip, &p);
            else
                rc = pitch ? so101_ik_pitch(m, xyz[i], pitch[i], roll, grip, &prev, &p) : so101_ik_near(m, xyz[i], &prev, &p);
            if (rc) return rc;
            if (i > 0)
                for (int j = 0; j < 4; j++)
                    if (RFABS(p.deg[j] - prev.deg[j]) > JUMP) return SO101_EPATH;
            if ((i % 5 == 0 || i == n - 1) && gap > 0 && so101_collides(m, p.deg, gap)) return SO101_ECOLLIDES;
            memcpy(q[i], p.deg, sizeof q[i]);
            prev = p;
        }
        if (!closed) return SO101_OK;
        so101_real off = 0;
        for (int j = 0; j < 4; j++)
            if (RFABS(q[n - 1][j] - q[0][j]) > off) off = RFABS(q[n - 1][j] - q[0][j]);
        if (off <= CLOSE) {
            memcpy(q[n - 1], q[0], sizeof q[0]);
            return SO101_OK;
        }
        memcpy(q[0], q[n - 1], sizeof q[0]);
    }
    if (at) *at = n - 1;
    return SO101_EPATH;
}

/* ---- timing ------------------------------------------------------------------------------------- */

/* point i of a path gone round laps times: a closed path's last point is its first */
static const so101_real *pt(const so101_real (*q)[SO101_NJ], int n, int laps, int i) {
    return laps > 1 ? q[i % (n - 1)] : q[i];
}

/* how much the squared path speed may change along segment seg, beside point p where it is x[p] */
static so101_real room(const so101_real (*q)[SO101_NJ], int n, int laps, int N, const so101_fast *f, const so101_real *x,
                       int p, int seg) {
    const so101_real *a = pt(q, n, laps, seg), *b = pt(q, n, laps, seg + 1), *c = pt(q, n, laps, p);
    so101_real r = BIG;
    for (int j = 0; j < SO101_NA; j++) {
        so101_real step = RFABS(b[j] - a[j]);
        if (step <= (so101_real)1e-12) continue;
        so101_real bend = p > 0 && p < N - 1 ? RFABS(pt(q, n, laps, p + 1)[j] - 2 * c[j] + pt(q, n, laps, p - 1)[j]) : 0;
        so101_real v = (f->amax - bend * x[p]) / step;
        if (v < r) r = v;
    }
    return r > 0 ? 2 * r : 0;
}

/* Time-optimal path parameterisation on the grid of path points. With x the squared path speed at a point
 * (points/s, squared), a joint's speed on a segment is its step there times the root of x, and its
 * acceleration has two parts: the change of its step at a point times x (the path bends), and its step
 * times the path acceleration, half the change of x from one point to the next. So each point has a
 * largest x (the speed limit, and the bend alone at the acceleration limit: a corner is passed slowly),
 * and each segment a largest change of x (what the bend at its end leaves of the acceleration limit). A
 * backward pass lowers every x to what can still brake for what comes after, a forward pass to what can be
 * reached from what came before. The two parts of the acceleration are added by size, so the result is on
 * the safe side of the exact optimum (Pham and Pham's TOPP-RA, 2018, finds that). */
so101_real so101_path_time(const so101_real (*q)[SO101_NJ], int n, int laps, const so101_fast *f, so101_real *t) {
    int N = laps > 1 ? (n - 1) * laps + 1 : n;
    so101_real *x = t;      /* the squared path speed at each point, until the times replace it */
    for (int i = 0; i < N; i++) {
        const so101_real *c = pt(q, n, laps, i);
        so101_real best = (so101_real)1e12;
        for (int j = 0; j < SO101_NA; j++) {
            so101_real before = i > 0 ? RFABS(c[j] - pt(q, n, laps, i - 1)[j]) : 0;
            so101_real after = i < N - 1 ? RFABS(pt(q, n, laps, i + 1)[j] - c[j]) : 0;
            so101_real wide = before > after ? before : after;
            if (wide > 0) {
                so101_real v = f->vmax / wide;
                if (v * v < best) best = v * v;
            }
            if (i > 0 && i < N - 1) {
                so101_real bend = RFABS(pt(q, n, laps, i + 1)[j] - 2 * c[j] + pt(q, n, laps, i - 1)[j]);
                if (bend > 0 && f->amax / bend < best) best = f->amax / bend;
            }
        }
        x[i] = best;
    }
    x[0] = x[N - 1] = 0;
    for (int i = N - 2; i >= 0; i--) {
        so101_real r = x[i + 1] + room(q, n, laps, N, f, x, i + 1, i);
        if (r < x[i]) x[i] = r;
    }
    for (int i = 0; i < N - 1; i++) {
        so101_real r = x[i] + room(q, n, laps, N, f, x, i, i);
        if (r < x[i + 1]) x[i + 1] = r;
    }
    so101_real vp = RSQRT(x[0]);
    t[0] = 0;
    for (int i = 1; i < N; i++) {
        so101_real v = RSQRT(x[i]), s = vp + v;
        t[i] = t[i - 1] + 2 / (s > (so101_real)1e-3 ? s : (so101_real)1e-3);
        vp = v;
    }
    return t[N - 1];
}

/* the place along the path (a point index and a fraction) at time tau of the timing t, clamped at the ends;
 * *k: where the last call found it */
static so101_real place(const so101_real *t, int N, so101_real tau, int *k) {
    if (tau <= 0) return 0;
    if (tau >= t[N - 1]) return (so101_real)(N - 1);
    int i = *k;
    while (i > 0 && t[i] > tau) i--;
    while (i < N - 2 && t[i + 1] <= tau) i++;
    *k = i;
    so101_real d = t[i + 1] - t[i];
    return i + (d > 0 ? (tau - t[i]) / d : 0);
}

#define TAPS 20     /* the smoothing window's taps */

/* The place along the path as a function of time is smoothed with a Hann window over f->smooth: the joints
 * stay on the path, only the timing is smoothed, so that no acceleration starts at once. The run begins
 * half a window early and ends half a window late, so it starts and ends exactly at rest on its ends. */
int so101_path_sample(const so101_real (*q)[SO101_NJ], int n, int laps, const so101_real *t, so101_real speed,
                      const so101_fast *f, so101_real (*out)[SO101_NJ], so101_real *at, int cap) {
    int N = laps > 1 ? (n - 1) * laps + 1 : n;
    if (N < 2) return -1;
    so101_real T = t[N - 1] / speed, h = f->smooth / 2, fine = f->smooth / TAPS;
    int K = (int)((T + 2 * h) / f->dt + (so101_real)0.999999) + 1;
    if (K > cap) return -1;
    so101_real w[TAPS], sum = 0;
    for (int j = 0; j < TAPS; j++) sum += w[j] = (so101_real)0.5 - (so101_real)0.5 * RCOS(2 * SO101_PI * (j + 1) / (TAPS + 1));
    int hint = 0;
    for (int k = 0; k < K; k++) {
        so101_real tau = k * f->dt - h, s = 0;
        for (int j = 0; j < TAPS; j++)
            s += w[j] * place(t, N, (tau + (j - (TAPS - 1) / (so101_real)2) * fine) * speed, &hint);
        s /= sum;
        if (s < 0) s = 0;
        if (s > N - 1) s = (so101_real)(N - 1);
        int i = (int)s;
        if (i > N - 2) i = N - 2;
        so101_real fr = s - i;
        const so101_real *a = pt(q, n, laps, i), *b = pt(q, n, laps, i + 1);
        for (int j = 0; j < SO101_NJ; j++) out[k][j] = a[j] + fr * (b[j] - a[j]);
        if (at) at[k] = s;
    }
    return K;
}

/* ---- the fastest pitch -------------------------------------------------------------------------- */

#define HARMONICS 3
#define NC (1 + 2 * HARMONICS)
#define EVALS 3000          /* laps tried at most */

static void profile(const so101_real c[NC], int n, so101_real *pitch) {
    for (int i = 0; i < n; i++) {
        so101_real s = 2 * SO101_PI * i / (n - 1), v = c[0];
        for (int k = 1; k <= HARMONICS; k++) v += c[2 * k - 1] * RCOS(k * s) + c[2 * k] * RSIN(k * s);
        pitch[i] = v;
    }
}

typedef struct {
    so101_model *m;
    const so101_real (*xyz)[3];
    int n;
    so101_real roll, grip, best;
    const so101_fast *f;
    so101_real *tp, (*tq)[SO101_NJ], *tt;     /* a trial's pitches, joints and times */
    so101_real *pitch, (*q)[SO101_NJ];        /* the best so far */
    int evals;
} search_t;

/* The lap at the pitches of c on each elbow branch; 1 if one beats the best so far, which it then is. A lap
 * is timed between two others, as a run of several laps goes round it. */
static int try_lap(search_t *S, const so101_real c[NC]) {
    int better = 0, n = S->n;
    profile(c, n, S->tp);
    for (int b = 1; b >= -1; b -= 2) {
        S->evals++;
        if (so101_path_poses(S->m, S->xyz, n, S->tp, S->roll, S->grip, b, S->f->gap, S->tq, NULL)) continue;
        so101_path_time((const so101_real (*)[SO101_NJ])S->tq, n, 3, S->f, S->tt);
        so101_real lap = BIG;
        for (int k = 0; k < 3; k++) {
            so101_real d = S->tt[(k + 1) * (n - 1)] - S->tt[k * (n - 1)];
            if (d < lap) lap = d;
        }
        if (lap < S->best - (so101_real)1e-4) {
            S->best = lap;
            memcpy(S->pitch, S->tp, sizeof(so101_real) * n);
            memcpy(S->q, S->tq, sizeof(so101_real) * SO101_NJ * n);
            better = 1;
        }
    }
    return better;
}

int so101_path_fastest(so101_model *m, const so101_real (*xyz)[3], int n, so101_real roll, so101_real grip,
                       const so101_fast *f, so101_real *pitch, so101_real (*q)[SO101_NJ], so101_real *lap, so101_real *work) {
    if (n < 3) return SO101_EPATH;
    search_t S = {m, xyz, n, roll, grip, BIG, f, work, (so101_real (*)[SO101_NJ])(work + n), work + n + SO101_NJ * n,
                  pitch, q, 0};
    so101_real c[NC] = {0}, best[NC] = {0};
    for (int k = -5; k <= 5; k++) {
        c[0] = (so101_real)(15 * k);
        if (try_lap(&S, c)) memcpy(best, c, sizeof c);
    }
    if (S.best >= BIG) return SO101_EUNREACHABLE;
    memcpy(c, best, sizeof c);
    for (so101_real step = 16; step >= 2 && S.evals < EVALS;) {
        int better = 0;
        for (int i = 0; i < NC; i++)
            for (int sign = 1; sign >= -1; sign -= 2) {
                so101_real trial[NC];
                memcpy(trial, c, sizeof c);
                trial[i] += sign * step;
                if (try_lap(&S, trial)) {
                    memcpy(c, trial, sizeof c);
                    better = 1;
                    break;
                }
            }
        if (!better) step /= 2;
    }
    if (lap) *lap = S.best;
    return SO101_OK;
}

/* ---- checks ------------------------------------------------------------------------------------- */

int so101_run_check(const so101_model *m, const so101_real (*q)[SO101_NJ], int n, const so101_fast *f, char *why,
                    int nwhy) {
    for (int k = 0; k < n; k++) {
        double t = (double)(k * f->dt);
        for (int j = 0; j < SO101_NA; j++) {
            if (q[k][j] < m->lo[j] + 2 || q[k][j] > m->hi[j] - 2) {
                snprintf(why, nwhy, "%s at %.1f degrees at %.2f s, outside its range %.1f..%.1f less 2", NAMES[j],
                         (double)q[k][j], t, (double)m->lo[j], (double)m->hi[j]);
                return 1;
            }
            if (k > 0 && RFABS(q[k][j] - q[k - 1][j]) > f->top * f->dt) {
                snprintf(why, nwhy, "%s would turn at %.0f degrees/s at %.2f s", NAMES[j],
                         (double)(RFABS(q[k][j] - q[k - 1][j]) / f->dt), t);
                return 1;
            }
        }
        int c = k % 5 == 0 || k == n - 1 ? so101_collides(m, q[k], f->gap) : 0;
        if (c) {
            snprintf(why, nwhy, "%s closer than %.0f mm at %.2f s", so101_collision_name(c), (double)(1000 * f->gap), t);
            return 1;
        }
    }
    return 0;
}

/* ---- learning ----------------------------------------------------------------------------------- */

#define W 8      /* the smoothing's ticks either way */

/* joint j's error at tick i, smoothed over 25 ms (a Gaussian), the ends held */
static so101_real err_at(const so101_real (*q)[SO101_NJ], const so101_real (*pos)[SO101_NJ], int n, const so101_real *g,
                         int i, int j) {
    so101_real s = 0;
    for (int k = -W; k <= W; k++) {
        int c = i + k < 0 ? 0 : i + k > n - 1 ? n - 1 : i + k;
        s += g[k + W] * (q[c][j] - pos[c][j]);
    }
    return s;
}

/* its rate per tick, as numpy's gradient takes it */
static so101_real rate_at(const so101_real (*q)[SO101_NJ], const so101_real (*pos)[SO101_NJ], int n, const so101_real *g,
                          int i, int j) {
    if (i <= 0) return err_at(q, pos, n, g, 1, j) - err_at(q, pos, n, g, 0, j);
    if (i >= n - 1) return err_at(q, pos, n, g, n - 1, j) - err_at(q, pos, n, g, n - 2, j);
    return (err_at(q, pos, n, g, i + 1, j) - err_at(q, pos, n, g, i - 1, j)) / 2;
}

/* Each joint's error, where it should have been less where it was, smoothed both ways in time (so nothing
 * is shifted), and put through the inverse of how it follows a goal: the error dead later plus lag times
 * its rate. A goal changed by that much moves the joint by the error. */
void so101_path_learn(const so101_real (*q)[SO101_NJ], const so101_real (*pos)[SO101_NJ], int n, const so101_fast *f,
                      so101_real gain, so101_real (*fix)[SO101_NA]) {
    if (n < 2) return;
    so101_real g[2 * W + 1], sum = 0;
    for (int k = -W; k <= W; k++) {
        so101_real z = k * f->dt / (so101_real)0.025;
        sum += g[k + W] = (so101_real)exp(-0.5 * (double)(z * z));
    }
    for (int k = 0; k <= 2 * W; k++) g[k] /= sum;
    so101_real ahead = f->dead / f->dt;
    for (int k = 0; k < n; k++) {
        so101_real x = k + ahead;
        if (x > n - 1) x = (so101_real)(n - 1);
        int i = (int)x;
        if (i > n - 2) i = n - 2;
        so101_real fr = x - i;
        for (int j = 0; j < SO101_NA; j++) {
            so101_real e = err_at(q, pos, n, g, i, j) + fr * (err_at(q, pos, n, g, i + 1, j) - err_at(q, pos, n, g, i, j));
            so101_real r = rate_at(q, pos, n, g, i, j) + fr * (rate_at(q, pos, n, g, i + 1, j) - rate_at(q, pos, n, g, i, j));
            fix[k][j] += gain * (e + f->lag * r / f->dt);
        }
    }
}

/* ---- from the start pose and back --------------------------------------------------------------- */

#define LEAD ((so101_real)0.25)    /* of the way: what wrist flex and elbow have done when lift and pan start */

/* In the folded start pose the gripper hangs just above the table, and the lift ten degrees forward with
 * the wrist still down puts it on the table: so the wrist and the elbow lead. */
static void unfold_at(const so101_real a[SO101_NJ], const so101_real b[SO101_NJ], so101_real s, so101_real out[SO101_NJ]) {
    so101_real u = (s - LEAD) / (1 - LEAD);
    u = u < 0 ? 0 : u > 1 ? 1 : u;
    so101_real late = u * u * (3 - 2 * u);
    for (int j = 0; j < SO101_NJ; j++) out[j] = a[j] + (j <= SO101_LIFT ? late : s) * (b[j] - a[j]);
}

static int unfold_points(const so101_real a[SO101_NJ], const so101_real b[SO101_NJ]) {
    so101_real most = 0;
    for (int j = 0; j < SO101_NJ; j++)
        if (RFABS(b[j] - a[j]) > most) most = RFABS(b[j] - a[j]);
    int k = (int)(most / (so101_real)0.5);
    return (k > 2 ? k : 2) + 1;
}

int so101_unfold_path(const so101_real a[SO101_NJ], const so101_real b[SO101_NJ], so101_real (*q)[SO101_NJ], int cap) {
    int N = unfold_points(a, b);
    if (N > cap) return -1;
    for (int i = 0; i < N; i++) unfold_at(a, b, (so101_real)i / (N - 1), q[i]);
    return N;
}

int so101_unfold_clear(const so101_model *m, const so101_real a[SO101_NJ], const so101_real b[SO101_NJ], so101_real tube) {
    int N = unfold_points(a, b);
    for (int i = 0; i < 25; i++) {
        int k = (int)((double)i * (N - 1) / 24);
        so101_real row[SO101_NJ];
        unfold_at(a, b, (so101_real)k / (N - 1), row);
        for (int c = 0; c < 9; c++) {
            so101_real v[SO101_NJ];
            memcpy(v, row, sizeof v);
            for (int d = 0; c && d < 3; d++) {
                int j = SO101_LIFT + d;
                so101_real lo = a[j] < b[j] ? a[j] : b[j], hi = a[j] < b[j] ? b[j] : a[j];
                v[j] += ((c - 1) >> d & 1) ? tube : -tube;
                v[j] = v[j] < lo ? lo : v[j] > hi ? hi : v[j];
            }
            if (so101_collides(m, v, 0)) return 0;
        }
    }
    return 1;
}
