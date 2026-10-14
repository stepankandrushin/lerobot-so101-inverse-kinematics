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

/* Fast runs (src/path.c, src/fast.c): a single joint's move is timed as the trapezoid it should be; a 20 cm
 * circle in front of the arm is planned in the pitch that makes its lap shortest, sampled within the limits and
 * on the circle, and driven on simulated servos from the folded start pose and back, the second replay
 * corrected by the first. The model has offsets that put the folded gripper above the table, as an arm's
 * measured ones may: with none it rests on the table in the model, and the fast way out is refused. */
#define LAP 316
#define TICKS 2000
static int fast_tests(const char *kind) {
    int bad = 0;
    so101_fast f;
    so101_fast_init(&f);
    static so101_real line[181][SO101_NJ], t[4 * LAP];
    for (int i = 0; i <= 180; i++) {
        for (int j = 0; j < SO101_NJ; j++) line[i][j] = 0;
        line[i][SO101_LIFT] = (so101_real)(0.5 * i);
    }
    double T = so101_path_time((const so101_real (*)[SO101_NJ])line, 181, 1, &f, t), trapezoid = 90 / 220.0 + 220 / 1500.0;
    if (fabs(T - trapezoid) > 0.01 * trapezoid) {
        printf("FAIL so101_path_time: 90 degrees in %.4f s, the trapezoid takes %.4f\n", T, trapezoid);
        bad++;
    }
    static so101_model m;
    so101_model_init(&m);
    so101_real off[SO101_NA] = {0, (so101_real)2.3, (so101_real)-5.9, (so101_real)-3.0, 0};
    for (int j = 0; j < SO101_NA; j++) m.offsets[j] = off[j];
    /* the plane 30 cm from the shoulder towards a viewer 1.5 m in front and 75 cm up, as so101ik.figures lays it */
    double S[3] = {0.0388, 0, 0.1166}, F[3] = {1.5, 0, 0.75}, w[3], r = 0.10;
    double len = sqrt(pow(F[0] - S[0], 2) + pow(F[2] - S[2], 2));
    for (int i = 0; i < 3; i++) w[i] = (F[i] - S[i]) / len;
    double right[3] = {0, -1, 0}, up[3] = {-w[2], 0, w[0]}, c[3];
    for (int i = 0; i < 3; i++) c[i] = S[i] + 0.30 * w[i] + 0.01 * up[i];
    static so101_real xyz[LAP][3], pitch[LAP], q[LAP][SO101_NJ], work[SO101_FASTEST_WORK(LAP)];
    for (int k = 0; k < LAP; k++) {
        double a = 2 * 3.14159265358979323846 * (k % (LAP - 1)) / (LAP - 1);
        for (int i = 0; i < 3; i++) xyz[k][i] = (so101_real)(c[i] + r * (sin(a) * right[i] + cos(a) * up[i]));
    }
    so101_real lap = 0;
    double t0 = now();
    int rc = so101_path_fastest(&m, (const so101_real (*)[3])xyz, LAP, -90, 2, &f, pitch, q, &lap, work);
    double t_plan = now() - t0;
    if (rc) {
        printf("FAIL so101_path_fastest: %s\n", so101_strerror(rc));
        return 1;
    }
    /* three laps at 0.85 of the limits: the simulated servos turn at 202 degrees/s at most */
    so101_path_time((const so101_real (*)[SO101_NJ])q, LAP, 3, &f, t);
    static so101_real run[TICKS][SO101_NJ], at[TICKS];
    int n = so101_path_sample((const so101_real (*)[SO101_NJ])q, LAP, 3, t, (so101_real)0.85, &f, run, at, TICKS);
    double far = 0, fastest = 0;
    for (int k = 0; k < n; k++) {
        so101_real tip[3];
        so101_tip(&m, run[k], tip, NULL);
        double d = sqrt(pow(tip[0] - c[0], 2) + pow(tip[1] - c[1], 2) + pow(tip[2] - c[2], 2));
        if (fabs(d - r) > far) far = fabs(d - r);
        for (int j = 0; k && j < SO101_NA; j++)
            if (fabs(run[k][j] - run[k - 1][j]) / f.dt > fastest) fastest = fabs(run[k][j] - run[k - 1][j]) / f.dt;
    }
    char why[200] = "";
    int refused = so101_run_check(&m, (const so101_real (*)[SO101_NJ])run, n, &f, why, sizeof why);
    double first = 0, last = 0;
    for (int j = 0; j < SO101_NJ; j++) {
        if (fabs(run[0][j] - q[0][j]) > first) first = fabs(run[0][j] - q[0][j]);
        if (fabs(run[n - 1][j] - q[0][j]) > last) last = fabs(run[n - 1][j] - q[0][j]);
    }
    if (n < 0 || refused || far > 2e-4 || fastest > 0.85 * f.vmax + 1 || first > 1e-3 || last > 1e-3) {
        printf("FAIL the circle's run: %d ticks, %s, the tip %.2g m off the circle, a joint at %.0f degrees/s, the ends "
               "%.2g and %.2g degrees off\n", n, refused ? why : "passes", far, fastest, first, last);
        bad++;
    }
    /* on simulated servos: unfold, two replays, fold */
    static so101_sim sim;
    static so101_arm arm;
    so101_hal hal;
    so101_real fold[SO101_NJ] = {0, m.lo[SO101_LIFT] + 2, m.hi[SO101_ELBOW] - 3, (so101_real)76.5, -90, 2};
    so101_sim_init(&sim, SO101_DEFAULT_CAL, fold);
    so101_hal_sim(&hal, &sim);
    so101_arm_init(&arm, &hal, NULL);
    for (int j = 0; j < SO101_NA; j++) arm.model.offsets[j] = off[j];
    static so101_real way[1000][SO101_NJ], ways[2][TICKS][SO101_NJ], pos[TICKS][SO101_NJ], fix[TICKS][SO101_NA];
    static so101_real wt[1000];
    rc = so101_arm_torque_on(&arm);
    int clear = !rc && so101_unfold_clear(&m, arm.start, q[0], f.tube), nw[2] = {0, 0};
    for (int i = 0; clear && i < 2; i++) {
        int p = so101_unfold_path(arm.start, q[0], way, 1000);
        if (i)
            for (int k = 0; k < p / 2; k++)
                for (int j = 0; j < SO101_NJ; j++) {
                    so101_real x = way[k][j];
                    way[k][j] = way[p - 1 - k][j], way[p - 1 - k][j] = x;
                }
        so101_path_time((const so101_real (*)[SO101_NJ])way, p, 1, &f, wt);
        nw[i] = so101_path_sample((const so101_real (*)[SO101_NJ])way, p, 1, wt, (so101_real)0.85, &f, ways[i], NULL, TICKS);
    }
    double rms[2] = {0, 0};
    if (clear) rc = so101_arm_fast(&arm, &f, 1);
    if (clear && !rc) rc = so101_arm_play(&arm, &f, (const so101_real (*)[SO101_NJ])ways[0], nw[0], NULL, NULL, 0, f.tube, NULL, NULL);
    for (int k = 0; k < n; k++)
        for (int j = 0; j < SO101_NA; j++) fix[k][j] = 0;
    for (int i = 0; clear && !rc && i < 2; i++) {
        rc = so101_arm_play(&arm, &f, (const so101_real (*)[SO101_NJ])run, n, NULL, (const so101_real (*)[SO101_NA])fix,
                            (so101_real)0.1, f.err_max, pos, NULL);
        for (int k = 0; k < n; k++)
            for (int j = 0; j < 4; j++) rms[i] += pow(run[k][j] - pos[k][j], 2) / (4.0 * n);
        rms[i] = sqrt(rms[i]);
        so101_path_learn((const so101_real (*)[SO101_NJ])run, (const so101_real (*)[SO101_NJ])pos, n, &f, (so101_real)0.7, fix);
    }
    if (clear && !rc) rc = so101_arm_play(&arm, &f, (const so101_real (*)[SO101_NJ])ways[1], nw[1], NULL, NULL, (so101_real)0.3, f.tube,
                                          NULL, NULL);
    if (clear && !rc) rc = so101_arm_fast(&arm, &f, 0);
    double home = 0;
    so101_real now_deg[SO101_NJ];
    if (!rc && so101_arm_read(&arm, now_deg, NULL, NULL) == SO101_OK)
        for (int j = 0; j < SO101_NA; j++) home = fabs(now_deg[j] - arm.start[j]) > home ? fabs(now_deg[j] - arm.start[j]) : home;
    so101_arm_torque_off(&arm);
    if (!clear || rc || rms[1] > rms[0] || home > 2) {
        printf("FAIL on simulated servos: %s%s, joints %.2f then %.2f degrees rms, %.1f degrees from the start pose at the end\n",
               clear ? "" : "the way out of the start pose is not clear; ", rc ? arm.err : "no stop", rms[0], rms[1], home);
        bad++;
    }
    printf("%s: fast runs: 90 degrees in %.4f s (the trapezoid %.4f); a 20 cm circle's fastest lap %.3f s, planned in "
           "%.0f ms; 3 laps at 0.85 in %d ticks, the tip %.2g m off the circle, joints up to %.0f degrees/s; simulated: "
           "joints %.2f then %.2f degrees rms, home within %.1f\n",
           kind, T, trapezoid, (double)lap, 1e3 * t_plan, n, far, fastest, rms[0], rms[1], home);
    return bad;
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
    return fast_tests(kind) + failures != 0 || worst_jump > 3;
}
