/* Forward kinematics, gravity torques, the rules a pose must keep, and the mesh collision check. */
#include <stdio.h>
#include <string.h>
#include "so101ik_internal.h"

void so101_model_ranges(so101_model *m, const so101_servo_cal cal[SO101_NJ]) {
    for (int j = 0; j < SO101_NA; j++) {
        so101_real mid = (so101_real)(cal[j].range_min + cal[j].range_max) / 2;
        m->lo[j] = (cal[j].range_min - mid) * 360 / (so101_real)4095;
        m->hi[j] = (cal[j].range_max - mid) * 360 / (so101_real)4095;
    }
    m->pl.valid = 0;
}

void so101_model_init(so101_model *m) {
    memset(m, 0, sizeof *m);
    so101_model_ranges(m, SO101_DEFAULT_CAL);
    m->payload = 0;
    m->table_z = (so101_real)-0.0054;
    m->margin = 5;
    m->clearance = (so101_real)0.03;
    m->tip_clearance = (so101_real)0.005;
    m->gap = (so101_real)0.008;
    m->comfort = (so101_real)0.005;
    m->smooth = (so101_real)0.0006;
    m->pitch_step = 5;
    m->near_window = 10;
    m->base_rule = 1;
    m->collisions = 1;
}

void so101_model_stock(so101_model *m) {
    so101_model_init(m);
    for (int j = 0; j < SO101_NA; j++) {
        m->offsets[j] = 0;
        m->lo[j] = SO101_JOINTS[j].lo / DEG;
        m->hi[j] = SO101_JOINTS[j].hi / DEG;
    }
    m->table_z = (so101_real)-1e9;
    m->margin = 0;
    m->base_rule = 0;
    m->collisions = 0;
}

/* a * b for 3x4 transforms */
static inline void tf_mul(const so101_tf *a, const so101_real *R, const so101_real *p, so101_tf *out) {
    so101_tf t;
    for (int i = 0; i < 3; i++) {
        const so101_real *r = a->R + 3 * i;
        for (int k = 0; k < 3; k++) t.R[3 * i + k] = r[0] * R[k] + r[1] * R[3 + k] + r[2] * R[6 + k];
        t.p[i] = r[0] * p[0] + r[1] * p[1] + r[2] * p[2] + a->p[i];
    }
    *out = t;
}

/* parent * origin * Rz(q) */
static inline void joint_step(const so101_tf *parent, const so101_joint_def *j, so101_real q, so101_tf *out) {
    so101_real c = RCOS(q), s = RSIN(q), R[9];
    for (int i = 0; i < 3; i++) {
        R[3 * i] = c * j->R[3 * i] + s * j->R[3 * i + 1];
        R[3 * i + 1] = -s * j->R[3 * i] + c * j->R[3 * i + 1];
        R[3 * i + 2] = j->R[3 * i + 2];
    }
    tf_mul(parent, R, j->p, out);
}

void so101_frames_rad(const so101_real q[SO101_NJ], so101_tf F[8]) {
    static const so101_tf I = {{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}};
    F[0] = I;
    for (int j = 0; j < 5; j++) joint_step(&F[j], &SO101_JOINTS[j], q[j], &F[j + 1]);
    joint_step(&F[5], &SO101_JOINTS[SO101_GRIP], q[SO101_GRIP], &F[6]);   /* the jaw hangs on the gripper link */
    tf_mul(&F[5], SO101_TIP.R, SO101_TIP.p, &F[7]);
}

void so101_frames(const so101_model *m, const so101_real deg[SO101_NJ], so101_tf F[8]) {
    so101_real q[SO101_NJ];
    for (int j = 0; j < SO101_NA; j++) q[j] = (deg[j] + m->offsets[j]) * DEG;
    /* LeRobot's gripper is 0 (closed) to 100 (open); the URDF's jaw turns -10 to 100 degrees. */
    q[SO101_GRIP] = (-10 + (so101_real)1.1 * deg[SO101_GRIP]) * DEG;
    so101_frames_rad(q, F);
}

static so101_real tip_pitch(const so101_tf F[8]) {
    const so101_real *T = F[7].R, *W = F[4].R;
    so101_real dx = T[2], dy = T[5], dz = T[8], nx = W[2], ny = W[5];
    return RATAN2(dz, dx * ny - dy * nx) / DEG;
}

so101_real so101_score(const so101_model *m, const so101_pose *p) {
    so101_real s = 0;
    for (int j = SO101_LIFT; j <= SO101_FLEX; j++) {
        so101_real u = 2 * p->deg[j] / (m->hi[j] - m->lo[j]);
        s += u * u;
    }
    return p->cost + m->comfort / 3 * s;
}

void so101_pose_from_frames(const so101_model *m, const so101_real deg[SO101_NJ], const so101_tf F[8], so101_pose *out) {
    memcpy(out->deg, deg, sizeof out->deg);
    for (int i = 0; i < 3; i++) {
        out->tip[i] = F[7].p[i];
        out->elbow[i] = F[3].p[i];
        out->wrist[i] = F[4].p[i];
        out->roll[i] = F[5].p[i];
    }
    out->pitch = tip_pitch(F);
    /* Gravity's torque about a joint is axis . ((sum m_i c_i - M o) x g) over the links beyond it; with
     * g = -9.81 z that is -9.81 (a_x r_y - a_y r_x) for r = sum m_i c_i - M o. The servo holds minus it. */
    so101_real mc[SO101_NL][3];
    for (int k = 1; k < SO101_NL; k++) {
        const so101_real *R = F[k].R, *c = SO101_COM[k];
        for (int i = 0; i < 3; i++)
            mc[k][i] = SO101_MASS[k] * (R[3 * i] * c[0] + R[3 * i + 1] * c[1] + R[3 * i + 2] * c[2] + F[k].p[i]);
    }
    so101_real sx = 0, sy = 0, sm = 0;     /* moment and mass of the links beyond each joint, tip first */
    if (m->payload > 0) {
        sx = m->payload * F[7].p[0];
        sy = m->payload * F[7].p[1];
        sm = m->payload;
    }
    sx += mc[6][0]; sy += mc[6][1]; sm += SO101_MASS[6];
    out->cost = 0;
    for (int j = SO101_NA - 1; j >= 0; j--) {
        const so101_tf *f = &F[j + 1];
        sx += mc[j + 1][0]; sy += mc[j + 1][1]; sm += SO101_MASS[j + 1];
        so101_real rx = sx - sm * f->p[0], ry = sy - sm * f->p[1];
        out->torque[j] = G * (f->R[2] * ry - f->R[5] * rx);
        if (j >= SO101_LIFT) out->cost += out->torque[j] * out->torque[j];
    }
    out->score = so101_score(m, out);
}

void so101_pose_eval(const so101_model *m, const so101_real deg[SO101_NJ], so101_pose *out) {
    so101_tf F[8];
    so101_frames(m, deg, F);
    so101_pose_from_frames(m, deg, F, out);
}

void so101_tip(const so101_model *m, const so101_real deg[SO101_NJ], so101_real tip[3], so101_real *pitch) {
    so101_tf F[8];
    so101_frames(m, deg, F);
    for (int i = 0; i < 3; i++) tip[i] = F[7].p[i];
    if (pitch) *pitch = tip_pitch(F);
}

void so101_fk_urdf(const so101_real q_deg[SO101_NA], so101_real T[16]) {
    so101_real q[SO101_NJ] = {0};
    for (int j = 0; j < SO101_NA; j++) q[j] = q_deg[j] * DEG;
    so101_tf F[8];
    so101_frames_rad(q, F);
    for (int i = 0; i < 3; i++) {
        T[4 * i] = F[7].R[3 * i];
        T[4 * i + 1] = F[7].R[3 * i + 1];
        T[4 * i + 2] = F[7].R[3 * i + 2];
        T[4 * i + 3] = F[7].p[i];
    }
    T[12] = T[13] = T[14] = 0;
    T[15] = 1;
}

/* ---- the rules ------------------------------------------------------------------------------- */

unsigned so101_problems(const so101_model *m, const so101_pose *p, so101_real margin) {
    unsigned out = 0;
    for (int j = 0; j < 4; j++)
        if (!(m->lo[j] + margin <= p->deg[j] && p->deg[j] <= m->hi[j] - margin)) out |= 1u << j;
    so101_real low = m->table_z + m->clearance;
    if (p->elbow[2] < low) out |= SO101_P_ELBOW_LOW;
    if (p->wrist[2] < low) out |= SO101_P_WRIST_LOW;
    if (p->roll[2] < low) out |= SO101_P_ROLL_LOW;
    if (p->tip[2] < m->table_z + m->tip_clearance) out |= SO101_P_TIP_LOW;
    if (m->base_rule) {
        const so101_real *pt[2] = {p->roll, p->tip};
        for (int k = 0; k < 2; k++) {
            so101_real dx = pt[k][0] - SO101_PAN_AXIS_X, dy = pt[k][1] - SO101_PAN_AXIS_Y;
            if (pt[k][2] < (so101_real)0.13 && dx * dx + dy * dy < (so101_real)(0.085 * 0.085))
                out |= k ? SO101_P_TIP_BASE : SO101_P_ROLL_BASE;
        }
    }
    return out;
}

const char *so101_problem_name(unsigned bit) {
    switch (bit) {
    case SO101_P_PAN: return "shoulder_pan outside its range";
    case SO101_P_LIFT: return "shoulder_lift outside its range";
    case SO101_P_ELBOW: return "elbow_flex outside its range";
    case SO101_P_FLEX: return "wrist_flex outside its range";
    case SO101_P_ELBOW_LOW: return "elbow axis near the table";
    case SO101_P_WRIST_LOW: return "wrist axis near the table";
    case SO101_P_ROLL_LOW: return "roll axis near the table";
    case SO101_P_TIP_LOW: return "tip near the table";
    case SO101_P_ROLL_BASE: return "roll inside the base";
    case SO101_P_TIP_BASE: return "tip inside the base";
    }
    return "?";
}

/* ---- the meshes ------------------------------------------------------------------------------ */

/* Pairs of links two or more apart in the chain; the jaw is the gripper's own part and turns inside the
 * wrist's reach. Codes: 1-5 a link (upper arm to jaw) and the table, 6 on the pairs. */
static const signed char PAIRS[][2] = {{0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}, {1, 3}, {1, 4}, {1, 5}, {1, 6},
                                       {2, 4}, {2, 5}, {2, 6}, {3, 5}, {3, 6}};
#define NPAIRS ((int)(sizeof PAIRS / sizeof PAIRS[0]))
static const char *LINK_NAMES[SO101_NL] = {"base_link", "shoulder_link", "upper_arm_link", "lower_arm_link",
                                        "wrist_link", "gripper_link", "moving_jaw_so101_v1_link"};

const char *so101_collision_name(int code) {
    static char buf[80];
    if (code >= 1 && code <= 5) {
        snprintf(buf, sizeof buf, "%s and the table", LINK_NAMES[code + 1]);
        return buf;
    }
    if (code >= 6 && code < 6 + NPAIRS) {
        snprintf(buf, sizeof buf, "%s and %s", LINK_NAMES[PAIRS[code - 6][0]], LINK_NAMES[PAIRS[code - 6][1]]);
        return buf;
    }
    return code ? "?" : "nothing";
}

typedef struct { float R[9], p[3]; } ftf;

static inline void xf(const ftf *t, const float *v, float *o) {
    o[0] = t->R[0] * v[0] + t->R[1] * v[1] + t->R[2] * v[2] + t->p[0];
    o[1] = t->R[3] * v[0] + t->R[4] * v[1] + t->R[5] * v[2] + t->p[1];
    o[2] = t->R[6] * v[0] + t->R[7] * v[1] + t->R[8] * v[2] + t->p[2];
}

#ifndef SO101_SMALL
static unsigned stamp;     /* this check's number: a leaf whose stamp differs has its points still to move */
#endif

#ifndef SO101_SMALL
/* A leaf's points in the world, moved there once a check (so101_world_pts: 12 bytes a point, 180 KB). */
static const float (*leaf_points(int node, const ftf *T, float (*buf)[3]))[3] {
    const so101_node *nd = &SO101_NODES[node];
    (void)buf;
    if (so101_leaf_stamp[node] != stamp) {
        for (int i = nd->first; i < nd->first + nd->n; i++) xf(T, SO101_MESH_PTS[i], so101_world_pts[i]);
        so101_leaf_stamp[node] = stamp;
    }
    return (const float (*)[3])so101_world_pts[nd->first];
}
#else
/* With SO101_SMALL (a microcontroller's RAM): moved into buf every time. */
static const float (*leaf_points(int node, const ftf *T, float (*buf)[3]))[3] {
    const so101_node *nd = &SO101_NODES[node];
    for (int i = 0; i < nd->n; i++) xf(T, SO101_MESH_PTS[nd->first + i], buf[i]);
    return (const float (*)[3])buf;
}
#endif
#define LEAF_MAX 16

#if defined(__GNUC__)
#define NOINLINE __attribute__((noinline))
#else
#define NOINLINE
#endif

/* a leaf's points below height z (the leaf's buffer lives here, not in every level of the walk) */
static NOINLINE int leaf_below(int node, const ftf *T, float z) {
    float buf[LEAF_MAX][3];
    const float (*p)[3] = leaf_points(node, T, buf);
    for (int i = 0; i < SO101_NODES[node].n; i++)
        if (p[i][2] < z) return 1;
    return 0;
}

/* any point of the subtree below height z */
static int below(int node, const ftf *T, float z) {
    const so101_node *nd = &SO101_NODES[node];
    float cz = T->R[6] * nd->c[0] + T->R[7] * nd->c[1] + T->R[8] * nd->c[2] + T->p[2];
    if (cz - nd->r >= z) return 0;
    if (cz + nd->r < z) return 1;
    if (nd->left < 0) return leaf_below(node, T, z);
    return below(nd->left, T, z) || below(nd->right, T, z);
}

/* two leaves' points within gap of each other; cb is b's ball's centre in the world */
static NOINLINE int leaves_near(int a, const ftf *TA, int b, const ftf *TB, const float cb[3], float gap) {
    const so101_node *na = &SO101_NODES[a], *nb = &SO101_NODES[b];
    float ba[LEAF_MAX][3], bb[LEAF_MAX][3];
    const float (*pa)[3] = leaf_points(a, TA, ba), (*pb)[3] = leaf_points(b, TB, bb);
    float g2 = gap * gap, rb = (nb->r + gap) * (nb->r + gap);
    for (int i = 0; i < na->n; i++) {
        float ex = pa[i][0] - cb[0], ey = pa[i][1] - cb[1], ez = pa[i][2] - cb[2];
        if (ex * ex + ey * ey + ez * ez >= rb) continue;      /* this point is far from all of b */
        for (int j = 0; j < nb->n; j++) {
            float fx = pa[i][0] - pb[j][0], fy = pa[i][1] - pb[j][1], fz = pa[i][2] - pb[j][2];
            if (fx * fx + fy * fy + fz * fz < g2) return 1;
        }
    }
    return 0;
}

/* any point of subtree a within gap of one of subtree b */
static int near_pair(int a, const ftf *TA, int b, const ftf *TB, float gap) {
    const so101_node *na = &SO101_NODES[a], *nb = &SO101_NODES[b];
    float ca[3], cb[3];
    xf(TA, na->c, ca);
    xf(TB, nb->c, cb);
    float dx = ca[0] - cb[0], dy = ca[1] - cb[1], dz = ca[2] - cb[2], d2 = dx * dx + dy * dy + dz * dz;
    float reach = na->r + nb->r + gap;
    if (d2 >= reach * reach) return 0;
    float inside = gap - na->r - nb->r;      /* every point of one within gap of every point of the other */
    if (inside > 0 && d2 < inside * inside) return 1;
    if (na->left < 0 && nb->left < 0) return leaves_near(a, TA, b, TB, cb, gap);
    if (nb->left < 0 || (na->left >= 0 && na->r >= nb->r))
        return near_pair(na->left, TA, b, TB, gap) || near_pair(na->right, TA, b, TB, gap);
    return near_pair(a, TA, nb->left, TB, gap) || near_pair(a, TA, nb->right, TB, gap);
}

int so101_collides_frames(const so101_model *m, const so101_tf F[8], so101_real gap_r) {
    ftf T[SO101_NL];
    for (int k = 0; k < SO101_NL; k++) {
        for (int i = 0; i < 9; i++) T[k].R[i] = (float)F[k].R[i];
        for (int i = 0; i < 3; i++) T[k].p[i] = (float)F[k].p[i];
    }
#ifndef SO101_SMALL
    if (++stamp == 0) {       /* wrapped: no leaf may look moved */
        for (int i = 0; i < SO101_MESH_NNODES; i++) so101_leaf_stamp[i] = 0;
        stamp = 1;
    }
#endif
    float gap = (float)gap_r, table = (float)(m->table_z + gap_r);
    for (int k = 2; k < SO101_NL; k++)          /* the table, from the upper arm on */
        if (below(SO101_LINK_ROOT[k], &T[k], table)) return k - 1;
    for (int q = 0; q < NPAIRS; q++) {
        int A = PAIRS[q][0], B = PAIRS[q][1];
        if (near_pair(SO101_LINK_ROOT[A], &T[A], SO101_LINK_ROOT[B], &T[B], gap)) return 6 + q;
    }
    return 0;
}

int so101_collides(const so101_model *m, const so101_real deg[SO101_NJ], so101_real gap) {
    so101_tf F[8];
    so101_frames(m, deg, F);
    return so101_collides_frames(m, F, gap);
}

const char *so101_strerror(int code) {
    switch (code) {
    case SO101_OK: return "ok";
    case SO101_EUNREACHABLE: return "no allowed pose reaches the point";
    case SO101_ECOLLIDES: return "every allowed pose that reaches the point collides";
    case SO101_EPATH: return "the way there runs into a range end, the table or the arm itself";
    case SO101_EIO: return "a servo did not answer";
    case SO101_ECAL: return "the servos' calibration differs from the one given";
    case SO101_ESTALL: return "a joint stalled";
    case SO101_ELOAD: return "a joint overloaded";
    case SO101_ESTOP: return "stopped";
    case SO101_ESTATE: return "not in a state for that";
    case SO101_EMOVED: return "a joint moved at torque-on";
    }
    return "unknown error";
}
