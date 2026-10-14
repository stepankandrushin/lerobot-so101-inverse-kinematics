/* What the library shares inside itself: the generated model (model.c) and the frames. */
#ifndef SO101IK_INTERNAL_H
#define SO101IK_INTERNAL_H

#include <math.h>
#include "so101ik.h"

#ifdef SO101_FLOAT
#define RSIN sinf
#define RCOS cosf
#define RATAN2 atan2f
#define RACOS acosf
#define RSQRT sqrtf
#define RFABS fabsf
#define RFMOD fmodf
#else
#define RSIN sin
#define RCOS cos
#define RATAN2 atan2
#define RACOS acos
#define RSQRT sqrt
#define RFABS fabs
#define RFMOD fmod
#endif

#define SO101_PI ((so101_real)3.14159265358979323846)
#define DEG ((so101_real)(3.14159265358979323846 / 180.0))
#define G ((so101_real)9.81)

typedef struct { so101_real R[9]; so101_real p[3]; so101_real lo, hi; } so101_joint_def;   /* lo, hi: rad */
typedef struct { so101_real R[9]; so101_real p[3]; } so101_frame_def;
/* a node of a link's tree of bounding balls: its points are first .. first + n - 1; a leaf has left -1 */
typedef struct { float c[3], r; int first, n, left, right; } so101_node;
typedef struct { so101_real R[9]; so101_real p[3]; } so101_tf;   /* row-major rotation and translation */

extern const so101_joint_def SO101_JOINTS[SO101_NJ];
extern const so101_frame_def SO101_TIP;
extern const so101_real SO101_MASS[SO101_NL];
extern const so101_real SO101_COM[SO101_NL][3];
extern const float SO101_MESH_PTS[][3];
extern const so101_node SO101_NODES[];
extern const int SO101_LINK_ROOT[SO101_NL], SO101_MESH_NPTS, SO101_MESH_NNODES;
#ifndef SO101_SMALL
extern float so101_world_pts[][3];
extern unsigned so101_leaf_stamp[];
#endif

/* F[0] the base (identity), F[1..6] shoulder to moving jaw, F[7] the tip; angles in URDF radians. */
void so101_frames_rad(const so101_real q[SO101_NJ], so101_tf F[8]);
/* The same from calibrated degrees and the model's offsets. */
void so101_frames(const so101_model *m, const so101_real deg[SO101_NJ], so101_tf F[8]);
void so101_pose_from_frames(const so101_model *m, const so101_real deg[SO101_NJ], const so101_tf F[8], so101_pose *out);
so101_real so101_score(const so101_model *m, const so101_pose *p);
int so101_collides_frames(const so101_model *m, const so101_tf F[8], so101_real gap);
/* The pose at a pitch on elbow branch sigma (+1 or -1) alone, if it is allowed (src/ik.c; src/path.c starts a
 * path on each branch). */
int so101_ik_branch(so101_model *m, const so101_real xyz[3], so101_real pitch, so101_real roll, so101_real grip, int sigma,
                    so101_pose *out);

/* atan2 as Cephes computes it (atan.c: a rational function after range reduction, 1e-16 relative),
 * a few times faster than the system's on the inner loops of the IK. */
static inline so101_real so101_atan2(so101_real y, so101_real x) {
    static const so101_real P[5] = {-8.750608600031904122785E-1, -1.615753718733365076637E1, -7.500855792314704667340E1,
                                 -1.228866684490136173410E2, -6.485021904942025371773E1};
    static const so101_real Q[5] = {2.485846490142306297962E1, 1.650270098316988542046E2, 4.328810604912902668951E2,
                                 4.853903996359136964868E2, 1.945506571482613964425E2};
    so101_real ax = RFABS(x), ay = RFABS(y), t, base = 0, z;
    if (ax == 0 && ay == 0) return 0;
    int swap = ay > ax;
    t = swap ? ax / ay : ay / ax;             /* 0..1 */
    if (t > (so101_real)0.66) {
        base = (so101_real)0.78539816339744830962;
        t = (t - 1) / (t + 1);
    }
    z = t * t;
    z = z * ((((P[0] * z + P[1]) * z + P[2]) * z + P[3]) * z + P[4])
        / (((((z + Q[0]) * z + Q[1]) * z + Q[2]) * z + Q[3]) * z + Q[4]);
    so101_real a = base + t + t * z;             /* atan(min / max) */
    if (swap) a = (so101_real)1.57079632679489661923 - a;
    if (x < 0) a = SO101_PI - a;
    return y < 0 ? -a : a;
}

static inline so101_real so101_wrap(so101_real a) {     /* degrees into [-180, 180) */
    a = RFMOD(a + 180, 360);
    if (a < 0) a += 360;
    return a - 180;
}

#endif
