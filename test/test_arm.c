/* The arm's moves without an arm, on the simulated servos of src/sim.c: a chain followed with and without the
 * lead, the jump and start rules, lowering until contact with a floor and without one, a grasp on an object
 * and on nothing, rise, a blocked joint, a stop, home. Built in double and in float (make test). */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "so101ik.h"

#define NS SO101_NJ

static int stop_after, stops;        /* hal.stop says stop at this call (0: never) */
static int verbose;

static int h_stop(void *c) { return stop_after && ++stops >= stop_after; }
static void h_log(void *c, const char *line) {
    if (verbose) printf("    %s\n", line);
}

static int failures;
#define CHECK(cond, ...)                                     \
    do {                                                     \
        if (!(cond)) {                                       \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);      \
            printf(__VA_ARGS__);                             \
            printf("\n");                                    \
            failures++;                                      \
        }                                                    \
    } while (0)

static so101_real off_by(const so101_arm *a, const so101_real deg[NS], const so101_real *want) {
    so101_real m = 0;
    for (int j = 0; j < 4; j++)
        if (fabs(want[j] - deg[j]) > m) m = (so101_real)fabs(want[j] - deg[j]);
    return m;
}

/* An arm folded in its start pose, checked, torque on and unfolded. */
static void ready_arm(so101_sim *s, so101_arm *a) {
    so101_model m;
    so101_model_init(&m);
    so101_real fold[NS] = {0, m.lo[SO101_LIFT] + 2, m.hi[SO101_ELBOW] - 3, 76.5, -90, 2};
    so101_sim_init(s, NULL, fold);
    so101_hal h;
    so101_hal_sim(&h, s);
    h.stop = h_stop;
    h.log = h_log;
    so101_arm_init(a, &h, NULL);
    a->sag = 0;                              /* the simulation has no gravity */
    CHECK(so101_arm_check(a) == SO101_OK, "check: %s", a->err);
    CHECK(so101_arm_torque_on(a) == SO101_OK, "torque on: %s", a->err);
    CHECK(a->folded, "not taken as folded at torque-on");
    CHECK(so101_arm_unfold(a) == SO101_OK, "unfold: %s", a->err);
}

/* A chain of n poses, 2 mm apart, from the arm's pose along d (m) at a pitch. */
static int line(so101_arm *a, const so101_model *m0, const so101_real d[3], so101_real pitch, so101_real (*q)[NS], int n) {
    so101_model m = *m0;
    so101_pose p, prev;
    so101_real start[3];
    so101_tip(&m, a->goal, start, NULL);
    so101_pose_eval(&m, a->goal, &prev);
    memcpy(q[0], a->goal, sizeof q[0]);
    for (int k = 1; k < n; k++) {
        so101_real c[3] = {start[0] + d[0] * k / (n - 1), start[1] + d[1] * k / (n - 1), start[2] + d[2] * k / (n - 1)};
        if (so101_ik_pitch(&m, c, pitch, a->goal[SO101_ROLL], a->goal[SO101_GRIP], &prev, &p)) return k;
        memcpy(q[k], p.deg, sizeof q[k]);
        prev = p;
    }
    return n;
}

int main(int argc, char **argv) {
    verbose = argc > 1;
    const char *kind = sizeof(so101_real) == 4 ? "float" : "double";
    static so101_sim s;
    static so101_arm a;
    so101_real pos[NS];
    static so101_real q[200][NS];
    so101_model free;                        /* no table, no meshes: the chains below go where the test says */
    so101_model_init(&free);
    free.table_z = -1e9;
    free.collisions = 0;
    free.base_rule = 0;
    free.margin = 0;

    /* unfolding reaches the ready pose: the wrist straight, the rest as folded */
    ready_arm(&s, &a);
    so101_arm_read(&a, pos, NULL, NULL);
    CHECK(fabs(pos[SO101_FLEX]) < 1.5, "after unfold the wrist stands at %.1f", (double)pos[SO101_FLEX]);

    /* to a point over the table, then a chain 4 cm forward and down at a fixed pitch */
    so101_real xyz[3] = {0.22, 0.0, 0.12}, pitch = -60;
    CHECK(so101_arm_move_to(&a, xyz, &pitch, 40) == SO101_OK, "move_to: %s", a.err);
    so101_real d[3] = {0.04, 0.0, -0.03};
    int n = line(&a, &free, d, pitch, q, 26);
    CHECK(n == 26, "the chain stops after %d poses", n);
    so101_follow o;
    so101_follow_init(&o);
    o.speed = 0.04;

    /* without the lead the joints stop up to DEAD short of the last pose */
    strcpy(a.step, "unled");
    CHECK(so101_arm_follow(&a, (const so101_real (*)[NS])q, n, &o) == SO101_OK, "follow: %s", a.err);
    so101_arm_read(&a, pos, NULL, NULL);
    so101_real unled = off_by(&a, pos, q[n - 1]);

    /* back along it with the lead: they end within 0.25 degrees */
    so101_real back[200][NS];
    for (int i = 0; i < n; i++) memcpy(back[i], q[n - 1 - i], sizeof back[i]);
    memcpy(back[0], a.goal, sizeof back[0]);
    o.lead = 1;
    o.within = (so101_real)0.25;
    o.settle = (so101_real)0.8;
    strcpy(a.step, "led");
    CHECK(so101_arm_follow(&a, (const so101_real (*)[NS])back, n, &o) == SO101_OK, "follow led: %s", a.err);
    so101_arm_read(&a, pos, NULL, NULL);
    so101_real led = off_by(&a, pos, back[n - 1]);
    CHECK(led < 0.3 && led < unled, "led %.2f degrees off the last pose, unled %.2f", (double)led, (double)unled);
    CHECK(fabs(a.lead[SO101_LIFT]) > 0.01 || fabs(a.lead[SO101_ELBOW]) > 0.01, "the lead stayed 0");

    /* a jump in a chain, and a chain that starts away from the arm, are refused before anything moves */
    memcpy(back[0], a.goal, sizeof back[0]);
    memcpy(back[1], a.goal, sizeof back[1]);
    back[1][SO101_LIFT] += 4;
    CHECK(so101_arm_follow(&a, (const so101_real (*)[NS])back, 2, &o) == SO101_EPATH, "a 4-degree jump was not refused");
    back[0][SO101_LIFT] += 6;
    back[1][SO101_LIFT] = back[0][SO101_LIFT];
    CHECK(so101_arm_follow(&a, (const so101_real (*)[NS])back, 2, &o) == SO101_EPATH, "a chain 6 degrees off was not refused");

    /* lowering until contact: a floor 15 mm under the tip, the chain 25 mm down at a steady 12 mm/s */
    so101_real here[3];
    so101_tip(&free, a.goal, here, NULL);
    s.floor_z = here[2] - 0.015;
    so101_real down[3] = {0, 0, -0.025};
    n = line(&a, &free, down, pitch, q, 14);
    so101_follow_init(&o);
    o.speed = 0.012;
    o.steady = 1;
    o.press = 0.005;
    strcpy(a.step, "down");
    CHECK(so101_arm_follow(&a, (const so101_real (*)[NS])q, n, &o) == SO101_OK, "press: %s", a.err);
    so101_arm_read(&a, pos, NULL, NULL);
    so101_real tip[3];
    so101_tip(&free, pos, tip, NULL);
    CHECK(a.contact, "no contact found");
    CHECK(fabs(tip[2] - s.floor_z) < 0.002, "stopped %.1f mm from the floor", 1000 * (double)(tip[2] - s.floor_z));
    CHECK(off_by(&a, pos, a.goal) < 1.5, "after the press the goals are %.1f degrees from the joints", (double)off_by(&a, pos, a.goal));
    CHECK(a.have_pose, "no pose after the press");

    /* the same with no floor in reach: it ends at the chain's end, no contact */
    s.floor_z = -1e9;
    so101_real up[3] = {0, 0, 0.03};
    so101_follow_init(&o);
    strcpy(a.step, "up");
    n = line(&a, &free, up, pitch, q, 16);
    CHECK(so101_arm_follow(&a, (const so101_real (*)[NS])q, n, &o) == SO101_OK, "up: %s", a.err);
    n = line(&a, &free, down, pitch, q, 14);
    so101_follow_init(&o);
    o.speed = 0.012;
    o.steady = 1;
    o.press = 0.005;
    strcpy(a.step, "down, no floor");
    CHECK(so101_arm_follow(&a, (const so101_real (*)[NS])q, n, &o) == SO101_OK, "press, no floor: %s", a.err);
    CHECK(!a.contact, "contact found with no floor");

    /* grasp: an object holds the jaw at 30 percent */
    const so101_servo_cal *g = &s.cal[SO101_GRIP];
    so101_real target[NS];
    memcpy(target, a.goal, sizeof target);
    target[SO101_GRIP] = 45;
    strcpy(a.step, "open");
    CHECK(so101_arm_move_joints(&a, target, 90, 0) == SO101_OK, "open: %s", a.err);
    s.lo_block[SO101_GRIP] = g->range_min + (int)(0.30 * (g->range_max - g->range_min));
    so101_real at = -1;
    strcpy(a.step, "grip");
    CHECK(so101_arm_grasp(&a, 40, 3, 2.5, 4, &at) == SO101_OK, "grasp: %s", a.err);
    CHECK(fabs(at - 30) < 1.0, "the jaw stopped at %.1f, the object at 30", (double)at);
    CHECK(fabs(a.goal[SO101_GRIP] - (at - 2.5)) < 1e-3, "the goal %.1f is not squeeze past %.1f", (double)a.goal[SO101_GRIP], (double)at);
    /* ... and on nothing it closes to under empty */
    s.lo_block[SO101_GRIP] = -1;
    CHECK(so101_arm_move_joints(&a, target, 90, 0) == SO101_OK, "open again: %s", a.err);
    CHECK(so101_arm_grasp(&a, 40, 3, 2.5, 4, &at) == SO101_OK, "grasp on nothing: %s", a.err);
    CHECK(at < 4, "on nothing the jaw stopped at %.1f", (double)at);

    /* rise: straight up 4 cm from where it stands, x and y kept */
    so101_tip(&free, a.goal, here, NULL);
    CHECK(so101_arm_rise(&a, here[2] + 0.04, 0.02) == SO101_OK, "rise: %s", a.err);
    so101_tip(&free, a.goal, tip, NULL);
    CHECK(fabs(tip[2] - here[2] - 0.04) < 0.0015 && fabs(tip[0] - here[0]) < 0.001 && fabs(tip[1] - here[1]) < 0.001,
          "rise ended at %.1f %.1f %.1f cm from %.1f %.1f %.1f", 100 * (double)tip[0], 100 * (double)tip[1],
          100 * (double)tip[2], 100 * (double)here[0], 100 * (double)here[1], 100 * (double)here[2]);

    /* a blocked joint stops the move, every goal where its joint stands */
    s.hi_block[SO101_ELBOW] = (int)lround(s.pos[SO101_ELBOW]);
    s.lo_block[SO101_ELBOW] = (int)lround(s.pos[SO101_ELBOW]);
    memcpy(target, a.goal, sizeof target);
    target[SO101_ELBOW] += 20;
    strcpy(a.step, "blocked");
    int rc = so101_arm_move_joints(&a, target, 30, 0);
    CHECK(rc == SO101_ESTALL || rc == SO101_ELOAD, "a blocked elbow gave %d (%s)", rc, a.err);
    so101_arm_read(&a, pos, NULL, NULL);
    CHECK(off_by(&a, pos, a.goal) < 0.5, "after the stall the goals are %.1f degrees from the joints", (double)off_by(&a, pos, a.goal));
    s.hi_block[SO101_ELBOW] = s.lo_block[SO101_ELBOW] = -1;

    /* hal.stop freezes a move */
    target[SO101_ELBOW] = a.goal[SO101_ELBOW] + 20;
    stops = 0;
    stop_after = 10;
    rc = so101_arm_move_joints(&a, target, 30, 0);
    CHECK(rc == SO101_ESTOP, "a stop gave %d (%s)", rc, a.err);
    stop_after = 0;

    /* home from here: the start pose, torque off */
    CHECK(so101_arm_home(&a) == SO101_OK, "home: %s", a.err);
    CHECK(!a.torque, "the torque is on after home");
    so101_arm_read(&a, pos, NULL, NULL);
    CHECK(off_by(&a, pos, a.start) < 1.5, "home ended %.1f degrees from the start pose", (double)off_by(&a, pos, a.start));

    printf("%s: the arm's moves on the simulated servos: unled %.2f degrees off a chain's end, led %.2f; %d failures\n", kind,
           (double)unled, (double)led, failures);
    return failures != 0;
}
