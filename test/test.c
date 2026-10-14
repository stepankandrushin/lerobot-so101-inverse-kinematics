/* The library without Python: every pose so101_ik finds over a grid puts the tip on its target and keeps the
 * rules, paths of so101_ik_near stay smooth, so101_ik_local reaches the grid from a neighbour; and what each
 * call costs here. Built in double and in float (make test), as a microcontroller would run it. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "so101ik.h"

static double now(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + 1e-9 * t.tv_nsec;
}

static int cmp(const void *a, const void *b) {
    double x = *(const double *)a, y = *(const double *)b;
    return x < y ? -1 : x > y;
}

static double median(double *v, int n) {
    qsort(v, n, sizeof *v, cmp);
    return n ? v[n / 2] : NAN;
}

int main(void) {
    static so101_model m;
    so101_model_init(&m);
#ifdef SO101_SMALL
    const char *kind = "float, small";
#else
    const char *kind = sizeof(so101_real) == 4 ? "float" : "double";
#endif
    double tol = sizeof(so101_real) == 4 ? 1e-4 : 1e-9;     /* m */
    int failures = 0, reached = 0, n = 0;
    static double t_ik[1000], t_near[5000], t_local[1000];
    int nn = 0, nl = 0;
    double score_sum = 0, worst_tip = 0, worst_jump = 0, worst_local = 0;
    for (double x = 0.10; x < 0.405; x += 0.02)
        for (double z = 0.02; z < 0.365; z += 0.02) {
            so101_real xyz[3] = {(so101_real)x, (so101_real)0.05, (so101_real)z}, tip[3];
            so101_pose p, q;
            double t0 = now();
            int rc = so101_ik(&m, xyz, -90, 0, &p);
            t_ik[n++] = now() - t0;
            if (rc) continue;
            reached++;
            score_sum += p.score;
            so101_tip(&m, p.deg, tip, NULL);
            double e = sqrt(pow(tip[0] - xyz[0], 2) + pow(tip[1] - xyz[1], 2) + pow(tip[2] - xyz[2], 2));
            if (e > worst_tip) worst_tip = e;
            if (e > tol || so101_problems(&m, &p, m.margin) || so101_collides(&m, p.deg, m.gap)) {
                printf("FAIL so101_ik %.2f %.2f %.2f: tip %.2g m off, problems %x, collides %d\n", x, 0.05, z, e,
                       so101_problems(&m, &p, m.margin), so101_collides(&m, p.deg, m.gap));
                failures++;
            }
            /* 2 cm to the left in 2 mm steps */
            for (int k = 1; k <= 10; k++) {
                so101_real c[3] = {xyz[0], xyz[1] + (so101_real)0.002 * k, xyz[2]};
                t0 = now();
                rc = so101_ik_near(&m, c, &p, &q);
                t_near[nn++] = now() - t0;
                if (rc) break;
                for (int j = 0; j < 4; j++)
                    if (fabs(q.deg[j] - p.deg[j]) > worst_jump) worst_jump = fabs(q.deg[j] - p.deg[j]);
                p = q;
            }
            /* LeRobot's call: from this pose (URDF degrees) to the grid point 2 cm further */
            so101_real qd[SO101_NA], T[16], err;
            for (int j = 0; j < SO101_NA; j++) qd[j] = p.deg[j] + m.offsets[j];
            so101_fk_urdf(qd, T);
            T[3] += (so101_real)0.02;
            t0 = now();
            so101_ik_local(qd, T, 1, 0, 8, NULL, &err);
            t_local[nl++] = now() - t0;
            if (err > worst_local) worst_local = err;
        }
    so101_real deg[SO101_NJ] = {0, -60, 60, 30, -90, 0};
    double t0 = now();
    int hits = 0;   /* how many of the poses collide: not checked, kept so the loop is not optimised away */
    for (int i = 0; i < 20000; i++) {
        deg[1] = (so101_real)(-60 + (i % 100) * 0.5);
        hits += so101_collides(&m, deg, m.gap) != 0;
    }
    double t_col = (now() - t0) / 20000;
    if (hits < 0) return 2;
    so101_pose p;
    t0 = now();
    for (int i = 0; i < 200000; i++) {
        deg[1] = (so101_real)(-60 + (i % 100) * 0.5);
        so101_pose_eval(&m, deg, &p);
    }
    double t_fk = (now() - t0) / 200000;
    printf("%s: so101_ik reached %d of %d points, mean score %.6f, the tip %.2g m from its target at most; %d failures\n",
           kind, reached, n, score_sum / reached, worst_tip, failures);
    printf("%s: so101_ik_near, %d steps of 2 mm: joints %.2f degrees apart at most; so101_ik_local, 2 cm: %.2g m off at most\n",
           kind, nn, worst_jump, worst_local);
    printf("%s: a call (median): so101_ik %.1f us, so101_ik_near %.1f us, so101_ik_local %.2f us, so101_collides %.2f us, "
           "so101_pose_eval %.3f us\n", kind, 1e6 * median(t_ik, n), 1e6 * median(t_near, nn), 1e6 * median(t_local, nl),
           1e6 * t_col, 1e6 * t_fk);
    return failures != 0 || worst_jump > 3;
}
