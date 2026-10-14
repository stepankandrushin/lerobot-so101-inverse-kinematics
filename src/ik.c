/* Inverse kinematics.
 *
 * With the wrist roll fixed, a tip position leaves the arm one degree of freedom, the gripper's pitch.
 * Lift, elbow and wrist flex turn about parallel horizontal axes and the pan about a vertical one, so
 * for a pitch and an elbow branch the pose has a closed form: the pan from the tip's direction and its
 * lateral offset, the wrist flex from the pitch, lift and elbow as a two-link arm to the wrist. The
 * plane's geometry and the moments of the links about each axis are taken once per roll and gripper
 * from the full model (so101_planar), so a pose and its gravity load cost one sine and cosine and two
 * atan2s. Newton's method on the full model finishes the chosen pose (the URDF's axes are parallel to
 * 1e-5 rad, which leaves micrometres).
 *
 * The poses that reach a point form closed curves in joint space:
 * along one the pitch rises on one elbow branch to where the elbow is straight or folded and falls back
 * on the other. The wrist's squared distance from the lift axis is a sinusoid in the pitch, so where
 * the two links reach it, and so the curves, are known in closed form: a loop from pitch a to b on
 * branch +1 and back on -1, or a whole turn on each branch when they reach it at every pitch. so101_ik()
 * walks each loop every pitch_step degrees and more densely where the joints move fast (a step is at
 * most JSTEP degrees of joint space), goes downhill from every local minimum of the allowed poses, and
 * takes the best that passes the full checks; when that one collides, the best sample that does not, and
 * downhill from it with every check. */
#include <string.h>
#include "so101ik_internal.h"

#define BIG ((so101_real)1e30)
#define HIGH (1u << 16)    /* plan_t.bad: past the far end of a range */
/* how far past +-1 the elbow's cosine may round where the branches meet (the loops' ends) */
#ifdef SO101_FLOAT
#define CTOL ((so101_real)3e-5)
#else
#define CTOL ((so101_real)1e-9)
#endif

static inline so101_real wrapd(so101_real a) {    /* degrees into [-180, 180), for a within a few turns */
    while (a >= 180) a -= 360;
    while (a < -180) a += 360;
    return a;
}

/* rotation about +y by t, on (x, z): c = cos t, s = sin t */
static inline void rot(so101_real c, so101_real s, const so101_real v[2], so101_real o[2]) {
    o[0] = v[0] * c + v[1] * s;
    o[1] = -v[0] * s + v[1] * c;
}

static void planar_prepare(so101_model *m, so101_real roll, so101_real grip) {
    so101_planar *P = &m->pl;
    if (P->valid && P->roll == roll && P->grip == grip && P->payload == m->payload
        && !memcmp(P->off, m->offsets, sizeof P->off))
        return;
    so101_real deg[SO101_NJ] = {-m->offsets[0], -m->offsets[1], -m->offsets[2], -m->offsets[3], roll, grip};
    so101_tf F[8];
    so101_frames(m, deg, F);
    P->s[0] = F[1].R[8] > 0 ? 1 : -1;
    for (int i = 1; i < 4; i++) P->s[i] = F[i + 1].R[5] > 0 ? 1 : -1;
    P->pan_x = F[1].p[0];
    P->pan_y = F[1].p[1];
    const so101_real *o1 = F[2].p, *o2 = F[3].p, *o3 = F[4].p, *t = F[7].p, *rl = F[5].p;
    P->o1[0] = o1[0]; P->o1[1] = o1[2];
    P->a[0] = o2[0] - o1[0]; P->a[1] = o2[2] - o1[2];
    P->b[0] = o3[0] - o2[0]; P->b[1] = o3[2] - o2[2];
    P->c3[0] = t[0] - o3[0]; P->c3[1] = t[2] - o3[2];
    P->roll_pt[0] = rl[0] - o3[0]; P->roll_pt[1] = rl[2] - o3[2];
    P->tip_y = t[1];
    P->roll_y = rl[1];
    P->la = RSQRT(P->a[0] * P->a[0] + P->a[1] * P->a[1]);
    P->lb = RSQRT(P->b[0] * P->b[0] + P->b[1] * P->b[1]);
    P->delta0 = RATAN2(P->b[1], P->b[0]) - RATAN2(P->a[1], P->a[0]);
    P->cd0 = RCOS(P->delta0);
    P->sd0 = RSIN(P->delta0);
    P->lc3 = RSQRT(P->c3[0] * P->c3[0] + P->c3[1] * P->c3[1]);
    P->beta_c = RATAN2(P->c3[1], P->c3[0]);
    P->d_ang = RATAN2(F[7].R[8], F[7].R[2]);
    /* moments (kg m) of the links beyond each flex axis about it, at zero */
    so101_real c[SO101_NL][3];
    for (int k = 1; k < SO101_NL; k++)
        for (int i = 0; i < 3; i++)
            c[k][i] = F[k].R[3 * i] * SO101_COM[k][0] + F[k].R[3 * i + 1] * SO101_COM[k][1] + F[k].R[3 * i + 2] * SO101_COM[k][2]
                      + F[k].p[i];
    P->mu1[0] = SO101_MASS[2] * (c[2][0] - o1[0]); P->mu1[1] = SO101_MASS[2] * (c[2][2] - o1[2]);
    P->mu2[0] = SO101_MASS[3] * (c[3][0] - o2[0]); P->mu2[1] = SO101_MASS[3] * (c[3][2] - o2[2]);
    P->mu3[0] = m->payload * (t[0] - o3[0]); P->mu3[1] = m->payload * (t[2] - o3[2]);
    P->m3 = m->payload;
    for (int k = 4; k < SO101_NL; k++) {
        P->mu3[0] += SO101_MASS[k] * (c[k][0] - o3[0]);
        P->mu3[1] += SO101_MASS[k] * (c[k][2] - o3[2]);
        P->m3 += SO101_MASS[k];
    }
    P->m2 = SO101_MASS[3] + P->m3;
    /* the roll axis and the moment of gripper and jaw about it */
    for (int i = 0; i < 3; i++) {
        P->ra[i] = F[5].R[3 * i + 2];
        P->rr[i] = SO101_MASS[5] * (c[5][i] - rl[i]) + SO101_MASS[6] * (c[6][i] - rl[i]) + m->payload * (t[i] - rl[i]);
    }
    P->kc = 1 / (2 * P->la * P->lb);
    P->l2 = P->la * P->la + P->lb * P->lb;
    for (int j = 0; j < 4; j++) P->kcomf[j] = 2 / (m->hi[j] - m->lo[j]);
    P->roll = roll;
    P->grip = grip;
    P->payload = m->payload;
    memcpy(P->off, m->offsets, sizeof P->off);
    P->valid = 1;
}

/* What a tip position fixes whatever the pitch: the pan, and the wrist's way to the lift axis less the
 * gripper's part, w = W0 - R(t3) c3. */
typedef struct {
    so101_real xyz[3];
    int ok;                    /* off the pan axis by more than the tip's lateral offset */
    so101_real pan;            /* calibrated degrees */
    so101_real u2;             /* squared horizontal distance from the pan axis, less the lateral offset's */
    so101_real W0[2], K, M, bw;
    unsigned bad;           /* the tip's own problems: near the table, in the base */
} target_t;

static void target(const so101_model *m, const so101_real xyz[3], target_t *tg) {
    const so101_planar *P = &m->pl;
    memcpy(tg->xyz, xyz, sizeof tg->xyz);
    so101_real dx = xyz[0] - P->pan_x, dy = xyz[1] - P->pan_y, v = P->tip_y - P->pan_y;
    tg->u2 = dx * dx + dy * dy - v * v;
    tg->ok = tg->u2 > (so101_real)1e-12;
    tg->bad = 0;
    if (xyz[2] < m->table_z + m->tip_clearance) tg->bad |= SO101_P_TIP_LOW;
    so101_real bx = xyz[0] - SO101_PAN_AXIS_X, by = xyz[1] - SO101_PAN_AXIS_Y;
    if (m->base_rule && xyz[2] < (so101_real)0.13 && bx * bx + by * by < (so101_real)(0.085 * 0.085)) tg->bad |= SO101_P_TIP_BASE;
    if (!tg->ok) return;
    so101_real u = RSQRT(tg->u2);
    tg->pan = wrapd((RATAN2(dy, dx) - RATAN2(v, u)) / P->s[0] / DEG - m->offsets[0]);
    tg->W0[0] = P->pan_x + u - P->o1[0];
    tg->W0[1] = xyz[2] - P->o1[1];
    so101_real lw = RSQRT(tg->W0[0] * tg->W0[0] + tg->W0[1] * tg->W0[1]);
    tg->K = lw * lw + P->lc3 * P->lc3;
    tg->M = lw * P->lc3;
    tg->bw = RATAN2(tg->W0[1], tg->W0[0]);
}

typedef struct {
    so101_real deg[4];     /* pan, lift, elbow, wrist flex */
    so101_real score, cost;
    unsigned bad;       /* so101_problems bits, from the plane */
} plan_t;

/* The pose at pitch phi (degrees) on branch sigma (+1, -1) in the plane: 0 if the wrist is out of reach. */
static int planar_pose(const so101_model *m, const target_t *tg, so101_real phi, int sigma, so101_real margin, plan_t *o) {
    const so101_planar *P = &m->pl;
    if (!tg->ok) return 0;
    so101_real t3 = P->s[3] > 0 ? P->d_ang - phi * DEG : P->d_ang - SO101_PI + phi * DEG;
    so101_real c3 = RCOS(t3), s3 = RSIN(t3), tc[2], w[2];
    rot(c3, s3, P->c3, tc);
    w[0] = tg->W0[0] - tc[0];
    w[1] = tg->W0[1] - tc[1];
    so101_real ww = w[0] * w[0] + w[1] * w[1];
    so101_real C = (ww - P->l2) * P->kc;
    if (C > 1 || C < -1) {
        if (C > 1 + CTOL || C < -1 - CTOL) return 0;
        C = C > 0 ? 1 : -1;      /* where the branches meet */
    }
    so101_real sg = RSQRT(1 - C * C), gam = so101_atan2(sg, C);
    so101_real cd = P->cd0 * C + sigma * P->sd0 * sg, sd = P->sd0 * C - sigma * P->cd0 * sg;   /* delta0 - sigma gam */
    so101_real delta = P->delta0 - sigma * gam, rb[2], vv[2];
    rot(cd, sd, P->b, rb);
    vv[0] = P->a[0] + rb[0];
    vv[1] = P->a[1] + rb[1];
    so101_real dot = w[0] * vv[0] + w[1] * vv[1], cr = w[0] * vv[1] - w[1] * vv[0];
    so101_real t1 = so101_atan2(cr, dot), inv = 1 / RSQRT(dot * dot + cr * cr);
    so101_real c1 = dot * inv, s1 = cr * inv, c2 = c1 * cd - s1 * sd, s2 = s1 * cd + c1 * sd;
    const so101_real R2D = (so101_real)(180 / 3.14159265358979323846);
    o->deg[0] = tg->pan;
    o->deg[1] = wrapd(t1 * P->s[1] * R2D - m->offsets[1]);
    o->deg[2] = wrapd(delta * P->s[2] * R2D - m->offsets[2]);
    o->deg[3] = wrapd((t3 - t1 - delta) * P->s[3] * R2D - m->offsets[3]);
    /* points and torques */
    so101_real ra_[2], rb_[2], q[2];
    rot(c1, s1, P->a, ra_);
    rot(c2, s2, P->b, rb_);
    so101_real e[2] = {P->o1[0] + ra_[0], P->o1[1] + ra_[1]};     /* elbow axis */
    so101_real wr[2] = {e[0] + rb_[0], e[1] + rb_[1]};           /* wrist axis */
    rot(c3, s3, P->roll_pt, q);
    so101_real rl[2] = {wr[0] + q[0], wr[1] + q[1]};             /* roll axis */
    so101_real m1x = P->mu1[0] * c1 + P->mu1[1] * s1, m2x = P->mu2[0] * c2 + P->mu2[1] * s2;
    so101_real m3x = P->mu3[0] * c3 + P->mu3[1] * s3;
    so101_real tf = -G * P->s[3] * m3x;
    so101_real te = -G * P->s[2] * (m2x + P->m3 * rb_[0] + m3x);
    so101_real tl = -G * P->s[1] * (m1x + P->m2 * ra_[0] + m2x + P->m3 * rb_[0] + m3x);
    so101_real ax = P->ra[0] * c3 + P->ra[2] * s3, rx = P->rr[0] * c3 + P->rr[2] * s3;
    so101_real tr = G * (ax * P->rr[1] - P->ra[1] * rx);
    o->cost = tl * tl + te * te + tf * tf + tr * tr;
    so101_real cf = 0;
    for (int j = 1; j < 4; j++) {
        so101_real k = o->deg[j] * P->kcomf[j];
        cf += k * k;
    }
    o->score = o->cost + m->comfort * (so101_real)(1.0 / 3) * cf;
    /* the rules; a range's far end sets a second bit (HIGH << j), so two poses past opposite ends differ */
    unsigned bad = tg->bad;
    for (int j = 0; j < 4; j++) {
        if (o->deg[j] < m->lo[j] + margin) bad |= 1u << j;
        else if (o->deg[j] > m->hi[j] - margin) bad |= (1u << j) | (HIGH << j);
    }
    so101_real low = m->table_z + m->clearance + (margin > m->margin ? (so101_real)1e-6 : 0);
    if (e[1] < low) bad |= SO101_P_ELBOW_LOW;
    if (wr[1] < low) bad |= SO101_P_WRIST_LOW;
    if (rl[1] < low) bad |= SO101_P_ROLL_LOW;
    if (m->base_rule) {
        so101_real hx = rl[0] - P->pan_x, hy = P->roll_y - P->pan_y;
        if (rl[1] < (so101_real)0.13 && hx * hx + hy * hy < (so101_real)(0.085 * 0.085)) bad |= SO101_P_ROLL_BASE;
    }
    o->bad = bad;
    return 1;
}

/* the elbow branch a pose is on */
static int branch_of(const so101_model *m, const so101_real deg[SO101_NJ]) {
    const so101_planar *P = &m->pl;
    so101_real delta = P->s[2] * (deg[SO101_ELBOW] + m->offsets[SO101_ELBOW]) * DEG;
    so101_real rel = wrapd((P->delta0 - delta) / DEG);
    return rel >= 0 ? 1 : -1;
}

/* ---- the full model: Newton to (xyz, pitch) --------------------------------------------------- */

static int solve4(so101_real A[4][4], so101_real b[4]) {
    for (int c = 0; c < 4; c++) {
        int p = c;
        for (int r = c + 1; r < 4; r++)
            if (RFABS(A[r][c]) > RFABS(A[p][c])) p = r;
        if (RFABS(A[p][c]) < (so101_real)1e-12) return 0;
        if (p != c) {
            for (int k = 0; k < 4; k++) { so101_real t = A[c][k]; A[c][k] = A[p][k]; A[p][k] = t; }
            so101_real t = b[c]; b[c] = b[p]; b[p] = t;
        }
        for (int r = c + 1; r < 4; r++) {
            so101_real f = A[r][c] / A[c][c];
            for (int k = c; k < 4; k++) A[r][k] -= f * A[c][k];
            b[r] -= f * b[c];
        }
    }
    for (int c = 3; c >= 0; c--) {
        for (int k = c + 1; k < 4; k++) b[c] -= A[c][k] * b[k];
        b[c] /= A[c][c];
    }
    return 1;
}

static inline void cross(const so101_real *a, const so101_real *b, so101_real *o) {
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}

/* deg[0..3] onto the pose that puts the tip at xyz with pitch phi; roll and gripper in deg[4..5].
 * Fills out (the full model's pose) and returns 1, or 0 if it does not converge. */
static int polish(const so101_model *m, const so101_real xyz[3], so101_real phi, so101_real deg[SO101_NJ], so101_pose *out) {
#ifdef SO101_FLOAT
    const so101_real tol = (so101_real)2e-3;
#else
    const so101_real tol = (so101_real)1e-7;
#endif
    for (int it = 0; it < 12; it++) {
        so101_tf F[8];
        so101_frames(m, deg, F);
        const so101_real *T = F[7].p, *R = F[7].R, *W = F[4].R;
        so101_real d[3] = {R[2], R[5], R[8]}, n[3] = {W[2], W[5], W[8]};
        so101_real A = d[2], B = d[0] * n[1] - d[1] * n[0];
        so101_real r[4] = {1000 * (T[0] - xyz[0]), 1000 * (T[1] - xyz[1]), 1000 * (T[2] - xyz[2]),
                        so101_wrap(RATAN2(A, B) / DEG - phi)};
        so101_real worst = 0;
        for (int i = 0; i < 4; i++) worst = RFABS(r[i]) > worst ? RFABS(r[i]) : worst;
        if (worst < tol) {
            so101_pose_from_frames(m, deg, F, out);
            return 1;
        }
        so101_real J[4][4], rhs[4];
        for (int j = 0; j < 4; j++) {
            const so101_real *f = F[j + 1].R, *o = F[j + 1].p;
            so101_real e[3] = {f[2], f[5], f[8]}, rel[3] = {T[0] - o[0], T[1] - o[1], T[2] - o[2]}, v[3], dd[3], dn[3];
            cross(e, rel, v);
            cross(e, d, dd);
            cross(e, n, dn);
            so101_real dA = dd[2], dB = dd[0] * n[1] + d[0] * dn[1] - dd[1] * n[0] - d[1] * dn[0];
            for (int i = 0; i < 3; i++) J[i][j] = 1000 * v[i] * DEG;           /* mm a degree */
            J[3][j] = (B * dA - A * dB) / (A * A + B * B);                     /* degree a degree */
        }
        for (int i = 0; i < 4; i++) rhs[i] = -r[i];
        if (!solve4(J, rhs)) return 0;
        so101_real big = 0;
        for (int j = 0; j < 4; j++) big = RFABS(rhs[j]) > big ? RFABS(rhs[j]) : big;
        so101_real k = big > 20 ? 20 / big : 1;
        for (int j = 0; j < 4; j++) deg[j] = so101_wrap(deg[j] + k * rhs[j]);
    }
    return 0;
}

/* The full pose at pitch phi on branch sigma: the plane's, then Newton. */
static int full_pose(so101_model *m, const target_t *tg, so101_real phi, int sigma, so101_real roll, so101_real grip,
                     so101_pose *out) {
    plan_t pp;
    if (!planar_pose(m, tg, phi, sigma, m->margin, &pp)) return 0;
    so101_real deg[SO101_NJ] = {pp.deg[0], pp.deg[1], pp.deg[2], pp.deg[3], roll, grip};
    return polish(m, tg->xyz, phi, deg, out);
}

static int allowed(so101_model *m, const so101_pose *p) {
    if (so101_problems(m, p, m->margin)) return 0;
    return !m->collisions || !so101_collides(m, p->deg, m->gap);
}

/* ---- the search ------------------------------------------------------------------------------ */

typedef struct { so101_real a, b; int full; } loop_t;     /* from a to b and back; full: a turn on branch a */

#define MAXLOOPS 2           /* find_loops gives one or two */
#define MAXSAMP 512
#define SLACK ((so101_real)1e-3)   /* degrees: the plane's ranges this much inside the full model's */
/* Between two samples that break different rules (or ranges at different ends) an allowed stretch may
 * lie: there, and where a joint moves more than JSTEP degrees between two samples, the step is halved
 * up to DEPTH times. */
#define JSTEP ((so101_real)20)
#define DEPTH 5

typedef struct {
    const so101_pose *near;   /* along a path: the pose before, whose change costs smooth, */
    so101_real window;        /* and the search starts within window degrees of pitch of it */
} rank_t;

static so101_real rank_of(const so101_model *m, const plan_t *p, const rank_t *rk) {
    so101_real r = p->score;
    if (rk && rk->near) {
        so101_real s = 0;
        for (int j = 0; j < 4; j++) {
            so101_real d = p->deg[j] - rk->near->deg[j];
            s += d * d;
        }
        r += m->smooth * s;
    }
    return r;
}

static so101_real period(const loop_t *L) { return L->full ? 360 : 2 * (L->b - L->a); }

static void at(const loop_t *L, so101_real s, int *sigma, so101_real *phi) {
    so101_real P = period(L);
    if (s < 0 || s >= P) {
        s = RFMOD(s, P);
        if (s < 0) s += P;
    }
    if (L->full) {
        *sigma = (int)L->a;
        *phi = -180 + s;
        return;
    }
    so101_real len = L->b - L->a;
    if (s <= len) {
        *sigma = 1;
        *phi = L->a + s;
    } else {
        *sigma = -1;
        *phi = L->b - (s - len);
    }
}

/* |w|^2 = K - 2 M cos(psi), psi = t3 + bw - beta_c: within reach where (la - lb)^2 <= |w|^2 <= (la + lb)^2 */
static int find_loops(const so101_model *m, const target_t *tg, loop_t *L) {
    const so101_planar *P = &m->pl;
    if (!tg->ok) return 0;
    so101_real lo2 = (P->la - P->lb) * (P->la - P->lb), hi2 = (P->la + P->lb) * (P->la + P->lb);
    if (tg->M < (so101_real)1e-12) {
        if (tg->K < lo2 || tg->K > hi2) return 0;
        L[0] = (loop_t){1, 0, 1};
        L[1] = (loop_t){-1, 0, 1};
        return 2;
    }
    so101_real cl = (tg->K - hi2) / (2 * tg->M), ch = (tg->K - lo2) / (2 * tg->M);
    if (cl > 1 || ch < -1) return 0;
    so101_real A = ch >= 1 ? 0 : RACOS(ch), B = cl <= -1 ? SO101_PI : RACOS(cl);
    so101_real iv[2][2];
    int n = 0;
    if (A == 0 && B == SO101_PI) {
        L[0] = (loop_t){1, 0, 1};
        L[1] = (loop_t){-1, 0, 1};
        return 2;
    }
    if (A == 0) { iv[0][0] = -B; iv[0][1] = B; n = 1; }
    else if (B == SO101_PI) { iv[0][0] = A; iv[0][1] = 2 * SO101_PI - A; n = 1; }
    else { iv[0][0] = A; iv[0][1] = B; iv[1][0] = -B; iv[1][1] = -A; n = 2; }
    for (int i = 0; i < n; i++) {
        so101_real ph[2];
        for (int k = 0; k < 2; k++) {
            so101_real t3 = iv[i][k] - tg->bw + P->beta_c;
            ph[k] = (P->s[3] > 0 ? P->d_ang - t3 : t3 - P->d_ang + SO101_PI) / DEG;
        }
        so101_real a = ph[0] < ph[1] ? ph[0] : ph[1], b = ph[0] < ph[1] ? ph[1] : ph[0];
        L[i] = (loop_t){a, b, 0};
    }
    return n;
}

typedef struct { so101_real s, r, deg[4]; int ok; unsigned bad; } samp_t;   /* ok: within reach, deg holds the pose */

static void sample(const so101_model *m, const target_t *tg, const loop_t *L, so101_real s, const rank_t *rk, samp_t *o) {
    int sg;
    so101_real ph;
    plan_t p;
    at(L, s, &sg, &ph);
    o->s = s;
    o->r = BIG;
    o->ok = 0;
    if (rk && rk->near && RFABS(wrapd(ph - rk->near->pitch)) > rk->window) return;
    o->ok = planar_pose(m, tg, ph, sg, m->margin + SLACK, &p);
    if (!o->ok) return;
    memcpy(o->deg, p.deg, sizeof o->deg);
    o->bad = p.bad;
    if (!p.bad) o->r = rank_of(m, &p, rk);
}

static so101_real jdist2(const samp_t *a, const samp_t *b) {    /* squared, degrees */
    so101_real s = 0;
    for (int j = 0; j < 4; j++) {
        so101_real d = a->deg[j] - b->deg[j];
        d = d >= 180 ? d - 360 : d < -180 ? d + 360 : d;
        s += d * d;
    }
    return s;
}

typedef struct { samp_t *v; int n, max; } samps_t;

/* the samples strictly between a and b, in order, where the joints move more than JSTEP between them */
static void between(const so101_model *m, const target_t *tg, const loop_t *L, const rank_t *rk, const samp_t *a,
                    const samp_t *b, int depth, samps_t *S) {
    if (depth >= DEPTH || !a->ok || !b->ok || S->n >= S->max - 1) return;
    if (!((a->r >= BIG && b->r >= BIG && a->bad != b->bad) || jdist2(a, b) > JSTEP * JSTEP)) return;
    samp_t mid;
    sample(m, tg, L, (a->s + b->s) / 2, rk, &mid);
    between(m, tg, L, rk, a, &mid, depth + 1, S);
    if (S->n < S->max) S->v[S->n++] = mid;
    between(m, tg, L, rk, &mid, b, depth + 1, S);
}

static so101_real rank_s(const so101_model *m, const target_t *tg, const loop_t *L, so101_real s, const rank_t *rk) {
    int sg;
    so101_real ph;
    plan_t p;
    at(L, s, &sg, &ph);
    if (!planar_pose(m, tg, ph, sg, m->margin + SLACK, &p) || p.bad) return BIG;
    return rank_of(m, &p, rk);
}

/* Downhill along a loop from s among the allowed poses, in the plane: the step halves where neither way
 * is better, down to 1e-4 degrees. */
static so101_real downhill(const so101_model *m, const target_t *tg, const loop_t *L, so101_real s, so101_real step,
                        const rank_t *rk, so101_real *best) {
    while (step > (so101_real)1e-4) {
        int moved = 0;
        for (int sgn = 1; sgn >= -1 && !moved; sgn -= 2) {
            so101_real r = rank_s(m, tg, L, s + sgn * step, rk);
            if (r < *best) {
                *best = r;
                s += sgn * step;
                moved = 1;
            }
        }
        if (!moved) step /= 2;
    }
    return s;
}

static int full_at(so101_model *m, const target_t *tg, const loop_t *L, so101_real s, so101_real roll, so101_real grip,
                   so101_pose *out) {
    int sg;
    so101_real ph;
    at(L, s, &sg, &ph);
    return full_pose(m, tg, ph, sg, roll, grip, out) && allowed(m, out);
}

/* Downhill along a loop from s, a pose that passes every check (out holds it), with every check at every
 * step: to the least rank or the edge of what is allowed. */
static void descend(so101_model *m, const target_t *tg, const loop_t *L, so101_real s, so101_real step, so101_real roll,
                    so101_real grip, const rank_t *rk, so101_pose *out) {
    so101_pose p;
    so101_real best = rank_s(m, tg, L, s, rk);
    while (step > (so101_real)1e-3) {
        int moved = 0;
        for (int sgn = 1; sgn >= -1 && !moved; sgn -= 2) {
            so101_real r = rank_s(m, tg, L, s + sgn * step, rk);
            if (r < best && full_at(m, tg, L, s + sgn * step, roll, grip, &p)) {
                best = r;
                s += sgn * step;
                *out = p;
                moved = 1;
            }
        }
        if (!moved) step /= 2;
    }
}

static samp_t SAMP[MAXLOOPS][MAXSAMP];

typedef struct { int loop; so101_real s0, s, r; } cand_t;   /* a local minimum: its sample, refined, its rank */
#define MAXCAND 32
#define SCAN 160                 /* steps out of a colliding stretch, each way, */
#define SCAN_STEP ((so101_real)0.25)   /* of this many degrees of pitch */

static int search(so101_model *m, const so101_real xyz[3], so101_real roll, so101_real grip, so101_real step, const rank_t *rk,
                  so101_pose *out) {
    target_t tg;
    target(m, xyz, &tg);
    loop_t L[MAXLOOPS];
    int nl = find_loops(m, &tg, L);
    if (!nl) return SO101_EUNREACHABLE;
    int ns[MAXLOOPS], any = 0;
    for (int l = 0; l < nl; l++) {
        so101_real P = period(&L[l]);
        int n = L[l].full ? (int)(P / step + (so101_real)0.999) : 2 * (int)(P / 2 / step + (so101_real)0.999);
        if (n < 2) n = 2;
        so101_real ds = P / n;
        samps_t S = {SAMP[l], 0, MAXSAMP};
        samp_t first, prev, cur;
        sample(m, &tg, &L[l], 0, rk, &first);
        prev = first;
        for (int i = 1; i <= n && S.n < S.max; i++) {
            if (i < n) sample(m, &tg, &L[l], i * ds, rk, &cur);
            else cur = first, cur.s = P;
            S.v[S.n++] = prev;
            between(m, &tg, &L[l], rk, &prev, &cur, 0, &S);
            prev = cur;
        }
        ns[l] = S.n;
        for (int i = 0; i < S.n; i++) any |= S.v[i].r < BIG;
    }
    if (!any) return SO101_EUNREACHABLE;
    /* Every local minimum along the loops, refined; the best, if it passes every check, is the answer. */
    cand_t c[MAXCAND];
    int nc = 0;
    for (int l = 0; l < nl; l++) {
        samp_t *v = SAMP[l];
        int n = ns[l];
        so101_real P = period(&L[l]);
        for (int i = 0; i < n && nc < MAXCAND; i++) {
            const samp_t *pv = &v[(i + n - 1) % n], *nx = &v[(i + 1) % n];
            if (v[i].r >= BIG || v[i].r > pv->r || v[i].r >= nx->r) continue;
            so101_real gap_p = RFMOD(v[i].s - pv->s + P, P), gap_n = RFMOD(nx->s - v[i].s + P, P);
            c[nc] = (cand_t){l, v[i].s, v[i].s, v[i].r};
            c[nc].s = downhill(m, &tg, &L[l], v[i].s, (gap_p < gap_n ? gap_p : gap_n) / 2, rk, &c[nc].r);
            nc++;
        }
    }
    int bi = -1;
    for (int i = 0; i < nc; i++)
        if (bi < 0 || c[i].r < c[bi].r) bi = i;
    if (bi >= 0 && full_at(m, &tg, &L[c[bi].loop], c[bi].s, roll, grip, out)) return SO101_OK;
    /* It does not pass: out of the colliding stretch around it each way in small steps, to the edge
     * of what is allowed, and downhill from there with every check; and the other minima.
     * The best of what passes. */
    so101_pose p;
    so101_real best = BIG;
    for (int i = 0; i < nc; i++) {
        const loop_t *Li = &L[c[i].loop];
        if (i == bi) {
            for (int dir = 1; dir >= -1; dir -= 2) {
                so101_real s = c[i].s;
                for (int k = 0; k < SCAN; k++) {
                    s += dir * SCAN_STEP;
                    so101_real r = rank_s(m, &tg, Li, s, rk);
                    if (r >= BIG) break;                 /* out of what the plane allows */
                    if (r >= best) break;                /* no better than what passed */
                    if (full_at(m, &tg, Li, s, roll, grip, &p)) {
                        descend(m, &tg, Li, s, SCAN_STEP / 2, roll, grip, rk, &p);
                        r = rank_s(m, &tg, Li, s, rk);
                        so101_real rp = p.score;
                        if (rk && rk->near) rp = r;
                        if (rp < best) best = rp, *out = p;
                        break;
                    }
                }
            }
        } else if (c[i].r < best) {
            if (full_at(m, &tg, Li, c[i].s, roll, grip, &p)) {
                so101_real rp = rk && rk->near ? c[i].r : p.score;
                if (rp < best) best = rp, *out = p;
            } else if (full_at(m, &tg, Li, c[i].s0, roll, grip, &p)) {
                descend(m, &tg, Li, c[i].s0, step / 2, roll, grip, rk, &p);
                so101_real rp = p.score;
                if (rp < best) best = rp, *out = p;
            }
        }
    }
    if (best < BIG) return SO101_OK;
    /* Nothing near a minimum passes: the best sample that does, and downhill from it. */
    for (int guard = 0; guard < MAXLOOPS * MAXSAMP; guard++) {
        int gl = -1, gi = -1;
        for (int l = 0; l < nl; l++)
            for (int i = 0; i < ns[l]; i++)
                if (SAMP[l][i].r < BIG && (gl < 0 || SAMP[l][i].r < SAMP[gl][gi].r)) gl = l, gi = i;
        if (gl < 0) break;
        SAMP[gl][gi].r = BIG;
        if (full_at(m, &tg, &L[gl], SAMP[gl][gi].s, roll, grip, out)) {
            descend(m, &tg, &L[gl], SAMP[gl][gi].s, step / 2, roll, grip, rk, out);
            return SO101_OK;
        }
    }
    return SO101_ECOLLIDES;
}

int so101_ik(so101_model *m, const so101_real xyz[3], so101_real roll, so101_real grip, so101_pose *out) {
    planar_prepare(m, roll, grip);
    so101_real step = m->pitch_step > (so101_real)0.25 ? m->pitch_step : (so101_real)0.25;
    return search(m, xyz, roll, grip, step, NULL, out);
}

int so101_ik_near(so101_model *m, const so101_real xyz[3], const so101_pose *prev, so101_pose *out) {
    planar_prepare(m, prev->deg[SO101_ROLL], prev->deg[SO101_GRIP]);
    rank_t rk = {prev, m->near_window};
    return search(m, xyz, prev->deg[SO101_ROLL], prev->deg[SO101_GRIP], 2, &rk, out);
}

int so101_ik_pitch(so101_model *m, const so101_real xyz[3], so101_real pitch, so101_real roll, so101_real grip, const so101_pose *near,
                so101_pose *out) {
    if (near) {
        roll = near->deg[SO101_ROLL];
        grip = near->deg[SO101_GRIP];
    }
    planar_prepare(m, roll, grip);
    target_t tg;
    target(m, xyz, &tg);
    int s = near ? branch_of(m, near->deg) : 1;
    int sig[2] = {s, -s};
    for (int i = 0; i < 2; i++)
        if (full_pose(m, &tg, pitch, sig[i], roll, grip, out) && allowed(m, out)) return SO101_OK;
    return SO101_EUNREACHABLE;
}

int so101_ik_branch(so101_model *m, const so101_real xyz[3], so101_real pitch, so101_real roll, so101_real grip, int sigma,
                    so101_pose *out) {
    planar_prepare(m, roll, grip);
    target_t tg;
    target(m, xyz, &tg);
    return full_pose(m, &tg, pitch, sigma, roll, grip, out) && allowed(m, out) ? SO101_OK : SO101_EUNREACHABLE;
}

/* ---- a tip frame from the joints the arm has (LeRobot's RobotKinematics.inverse_kinematics) ---- */

static void log3(const so101_real M[9], so101_real w[3]) {
    so101_real tr = M[0] + M[4] + M[8], c = (tr - 1) / 2;
    so101_real v[3] = {M[7] - M[5], M[2] - M[6], M[3] - M[1]};
    if (c > 1) c = 1;
    if (c < -1) c = -1;
    so101_real th = RACOS(c), s = RSIN(th);
    if (th < (so101_real)1e-6) {
        for (int i = 0; i < 3; i++) w[i] = v[i] / 2;
        return;
    }
    if (s > (so101_real)1e-6) {
        for (int i = 0; i < 3; i++) w[i] = v[i] * th / (2 * s);
        return;
    }
    /* near pi: the axis from the diagonal */
    so101_real ax[3];
    for (int i = 0; i < 3; i++) {
        so101_real d = (M[4 * i] + 1) / 2;
        ax[i] = RSQRT(d > 0 ? d : 0);
    }
    if (ax[0] >= ax[1] && ax[0] >= ax[2]) {
        ax[1] = (M[1] + M[3]) / (4 * ax[0]);
        ax[2] = (M[2] + M[6]) / (4 * ax[0]);
    } else if (ax[1] >= ax[2]) {
        ax[0] = (M[1] + M[3]) / (4 * ax[1]);
        ax[2] = (M[5] + M[7]) / (4 * ax[1]);
    } else {
        ax[0] = (M[2] + M[6]) / (4 * ax[2]);
        ax[1] = (M[5] + M[7]) / (4 * ax[2]);
    }
    for (int i = 0; i < 3; i++) w[i] = ax[i] * th;
}

/* Solve the n x n symmetric positive system H x = g in place (Cholesky); 0 if it is not. */
static int chol(so101_real *H, so101_real *g, int n) {
    for (int j = 0; j < n; j++) {
        so101_real d = H[j * n + j];
        for (int k = 0; k < j; k++) d -= H[j * n + k] * H[j * n + k];
        if (d <= 0) return 0;
        d = RSQRT(d);
        H[j * n + j] = d;
        for (int i = j + 1; i < n; i++) {
            so101_real s = H[i * n + j];
            for (int k = 0; k < j; k++) s -= H[i * n + k] * H[j * n + k];
            H[i * n + j] = s / d;
        }
    }
    for (int i = 0; i < n; i++) {
        so101_real s = g[i];
        for (int k = 0; k < i; k++) s -= H[i * n + k] * g[k];
        g[i] = s / H[i * n + i];
    }
    for (int i = n - 1; i >= 0; i--) {
        so101_real s = g[i];
        for (int k = i + 1; k < n; k++) s -= H[k * n + i] * g[k];
        g[i] = s / H[i * n + i];
    }
    return 1;
}

/* the weighted residual's cost at the tip frame F[7], and its two parts */
static so101_real local_cost(const so101_tf F[8], const so101_real Rt[9], const so101_real pt[3], so101_real wp, so101_real wo,
                          so101_real ep[3], so101_real eo[3]) {
    const so101_real *p = F[7].p, *R = F[7].R;
    for (int i = 0; i < 3; i++) ep[i] = pt[i] - p[i], eo[i] = 0;
    if (wo > 0) {
        so101_real M[9];      /* Rt R^T */
        for (int i = 0; i < 3; i++)
            for (int k = 0; k < 3; k++)
                M[3 * i + k] = Rt[3 * i] * R[3 * k] + Rt[3 * i + 1] * R[3 * k + 1] + Rt[3 * i + 2] * R[3 * k + 2];
        log3(M, eo);
    }
    return wp * (ep[0] * ep[0] + ep[1] * ep[1] + ep[2] * ep[2]) + wo * (eo[0] * eo[0] + eo[1] * eo[1] + eo[2] * eo[2]);
}

/* Levenberg-Marquardt on wp |position error|^2 + wo |orientation error|^2 inside the URDF's limits: a step
 * is kept only if it lowers the cost, else the damping grows tenfold and it is tried again. */
static so101_real local_steps(so101_real q[SO101_NA], const so101_real T[16], so101_real wp, so101_real wo, int iters, int *taken) {
    so101_real qr[SO101_NJ] = {0}, lo[SO101_NA], hi[SO101_NA];
    for (int j = 0; j < SO101_NA; j++) {
        qr[j] = q[j] * DEG;
        lo[j] = SO101_JOINTS[j].lo;
        hi[j] = SO101_JOINTS[j].hi;
        if (qr[j] < lo[j]) qr[j] = lo[j];
        if (qr[j] > hi[j]) qr[j] = hi[j];
    }
    const so101_real Rt[9] = {T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]}, pt[3] = {T[3], T[7], T[11]};
    so101_tf F[8], Fn[8];
    so101_real ep[3], eo[3], epn[3], eon[3];
    so101_frames_rad(qr, F);
    so101_real cost = local_cost(F, Rt, pt, wp, wo, ep, eo), mu = (so101_real)1e-6;
    int it = 0;
    for (; it < iters; it++) {
        const so101_real *p = F[7].p;
        so101_real Jv[3][SO101_NA], Jw[3][SO101_NA];
        for (int j = 0; j < SO101_NA; j++) {
            const so101_real *f = F[j + 1].R, *o = F[j + 1].p;
            so101_real e[3] = {f[2], f[5], f[8]}, rel[3] = {p[0] - o[0], p[1] - o[1], p[2] - o[2]}, v[3];
            cross(e, rel, v);
            for (int i = 0; i < 3; i++) Jv[i][j] = v[i], Jw[i][j] = e[i];
        }
        so101_real H[SO101_NA * SO101_NA], g[SO101_NA], tr = 0;
        for (int a = 0; a < SO101_NA; a++) {
            for (int b = 0; b < SO101_NA; b++) {
                so101_real s = 0;
                for (int i = 0; i < 3; i++) s += wp * Jv[i][a] * Jv[i][b] + wo * Jw[i][a] * Jw[i][b];
                H[a * SO101_NA + b] = s;
            }
            tr += H[a * SO101_NA + a];
            so101_real s = 0;
            for (int i = 0; i < 3; i++) s += wp * Jv[i][a] * ep[i] + wo * Jw[i][a] * eo[i];
            g[a] = s;
        }
        so101_real dq[SO101_NA], big;
        int kept = 0;
        for (int tries = 0; tries < 8 && !kept; tries++) {
            so101_real Hc[SO101_NA * SO101_NA], lam = mu * tr + (so101_real)1e-12;
            memcpy(Hc, H, sizeof H);
            memcpy(dq, g, sizeof g);
            for (int a = 0; a < SO101_NA; a++) Hc[a * SO101_NA + a] += lam;
            if (!chol(Hc, dq, SO101_NA)) break;
            /* a joint the step would take past a limit is held at it, the rest solved again */
            int fixed[SO101_NA] = {0}, nfixed = 0;
            for (int j = 0; j < SO101_NA; j++)
                if (qr[j] + dq[j] < lo[j] || qr[j] + dq[j] > hi[j]) fixed[j] = 1, nfixed++;
            if (nfixed) {
                int idx[SO101_NA], nf = 0;
                so101_real d[SO101_NA];
                for (int j = 0; j < SO101_NA; j++) {
                    d[j] = fixed[j] ? (qr[j] + dq[j] < lo[j] ? lo[j] : hi[j]) - qr[j] : 0;
                    if (!fixed[j]) idx[nf++] = j;
                }
                so101_real Hf[SO101_NA * SO101_NA], gf[SO101_NA];
                for (int a = 0; a < nf; a++) {
                    so101_real s = g[idx[a]];
                    for (int j = 0; j < SO101_NA; j++)
                        if (fixed[j]) s -= H[idx[a] * SO101_NA + j] * d[j];
                    gf[a] = s;
                    for (int b = 0; b < nf; b++) Hf[a * nf + b] = H[idx[a] * SO101_NA + idx[b]] + (a == b ? lam : 0);
                }
                if (nf && !chol(Hf, gf, nf)) break;
                for (int a = 0; a < nf; a++) d[idx[a]] = gf[a];
                memcpy(dq, d, sizeof d);
            }
            big = 0;
            for (int j = 0; j < SO101_NA; j++) big = RFABS(dq[j]) > big ? RFABS(dq[j]) : big;
            so101_real k = big > (so101_real)0.5 ? (so101_real)0.5 / big : 1, qn[SO101_NJ] = {0};
            for (int j = 0; j < SO101_NA; j++) {
                qn[j] = qr[j] + k * dq[j];
                qn[j] = qn[j] < lo[j] ? lo[j] : qn[j] > hi[j] ? hi[j] : qn[j];
            }
            so101_frames_rad(qn, Fn);
            so101_real cn = local_cost(Fn, Rt, pt, wp, wo, epn, eon);
            if (cn <= cost) {
                kept = 1;
                memcpy(qr, qn, sizeof qr);
                memcpy(F, Fn, sizeof F);
                memcpy(ep, epn, sizeof ep);
                memcpy(eo, eon, sizeof eo);
                cost = cn;
                mu = mu / 3 > (so101_real)1e-9 ? mu / 3 : (so101_real)1e-9;
            } else {
                mu *= 10;
            }
        }
        if (!kept || big < (so101_real)1e-9) {
            it++;
            break;
        }
    }
    for (int j = 0; j < SO101_NA; j++) q[j] = qr[j] / DEG;
    *taken = it;
    return RSQRT(ep[0] * ep[0] + ep[1] * ep[1] + ep[2] * ep[2]);
}

int so101_ik_local(so101_real q[SO101_NA], const so101_real T[16], so101_real wp, so101_real wo, int iters, so101_model *fallback,
                so101_real *err) {
    so101_real q0[SO101_NA];
    memcpy(q0, q, sizeof q0);
    int taken;
    so101_real e = local_steps(q, T, wp, wo, iters, &taken);
    if (fallback && e > (so101_real)1e-3) {
        /* the pose at the pitch the arm has (or the target frame's), on its elbow branch */
        so101_real deg[SO101_NJ], pitch, xyz[3] = {T[3], T[7], T[11]};
        for (int j = 0; j < SO101_NA; j++) deg[j] = q0[j] - fallback->offsets[j];
        deg[SO101_GRIP] = 0;
        so101_pose start, p;
        so101_pose_eval(fallback, deg, &start);
        pitch = start.pitch;
        if (wo > 0) pitch = RATAN2(T[10], RSQRT(T[2] * T[2] + T[6] * T[6])) / DEG;
        if (so101_ik_pitch(fallback, xyz, pitch, deg[SO101_ROLL], 0, &start, &p) == SO101_OK
            || so101_ik(fallback, xyz, deg[SO101_ROLL], 0, &p) == SO101_OK) {
            so101_real qn[SO101_NA];
            for (int j = 0; j < SO101_NA; j++) qn[j] = p.deg[j] + fallback->offsets[j];
            int more = 0;
            so101_real en = wo > 0 ? local_steps(qn, T, wp, wo, iters, &more) : 0;
            if (wo <= 0) {
                so101_real t[3];
                so101_tip(fallback, p.deg, t, NULL);
                so101_real d0 = t[0] - xyz[0], d1 = t[1] - xyz[1], d2 = t[2] - xyz[2];
                en = RSQRT(d0 * d0 + d1 * d1 + d2 * d2);
            }
            if (en < e) {
                memcpy(q, qn, sizeof qn);
                e = en;
                taken += more;
            }
        }
    }
    if (err) *err = e;
    return taken;
}
