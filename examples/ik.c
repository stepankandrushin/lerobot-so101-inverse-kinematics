/* The kinematics from C, no arm needed: the least-loaded pose for a point, then the next poses of a
 * straight line from it, 2 mm apart. `make example` builds and runs it. */
#include <stdio.h>
#include "so101ik.h"

int main(void) {
    so101_model m;
    so101_model_init(&m);                      /* zero offsets, the compiled-in calibration's ranges */
    so101_real xyz[3] = {0.20, 0.12, 0.18};    /* m: x forward, y to the arm's left, z up */
    so101_pose p, q;
    int rc = so101_ik(&m, xyz, 0, 0, &p);      /* wrist roll 0 degrees, gripper 0 (closed) */
    if (rc != SO101_OK) {
        printf("so101_ik: %s\n", so101_strerror(rc));
        return 1;
    }
    printf("pan %.1f lift %.1f elbow %.1f wrist flex %.1f: pitch %.1f, gravity cost %.4f N^2 m^2\n",
           (double)p.deg[SO101_PAN], (double)p.deg[SO101_LIFT], (double)p.deg[SO101_ELBOW],
           (double)p.deg[SO101_FLEX], (double)p.pitch, (double)p.cost);
    for (int k = 1; k <= 25; k++) {            /* 5 cm to the left, a pose every 2 mm */
        xyz[1] += (so101_real)0.002;
        rc = so101_ik_near(&m, xyz, &p, &q);
        if (rc != SO101_OK) {
            printf("so101_ik_near at step %d: %s\n", k, so101_strerror(rc));
            return 1;
        }
        p = q;
    }
    printf("5 cm on: pan %.1f lift %.1f elbow %.1f wrist flex %.1f, the tip at %.1f %.1f %.1f cm\n",
           (double)p.deg[SO101_PAN], (double)p.deg[SO101_LIFT], (double)p.deg[SO101_ELBOW],
           (double)p.deg[SO101_FLEX], 100 * (double)p.tip[0], 100 * (double)p.tip[1], 100 * (double)p.tip[2]);
    return 0;
}
