/* so101ik: the SO-101 arm (SO-ARM100/101, LeRobot's follower) in C. Kinematics from SO-ARM100's URDF in
 * LeRobot's calibrated degrees; the inverse kinematics that picks, of all the poses that put the tip at a
 * point, the one gravity loads least; a local solver for a tip frame (what LeRobot's RobotKinematics
 * does); the mesh collision check; and the arm itself: Feetech STS servos on a half-duplex bus, moves on a
 * minimum-jerk profile with a stop on stall or overload.
 *
 * C99, no allocation, no file system. Everything that touches the board goes through so101_hal, so the
 * same code runs on a Mac or Linux (src/hal_posix.c) and on a microcontroller. so101_real is double, or
 * float with -DSO101_FLOAT (an ESP32's FPU is single precision only).
 *
 * Units: metres, degrees (LeRobot's calibrated degrees: the middle of each joint's calibrated range is
 * 0), the gripper 0 (closed) to 100 (open). Coordinates are the URDF's base frame: x forward, y to the
 * arm's left, z up. */
#ifndef SO101IK_H
#define SO101IK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SO101_FLOAT
typedef float so101_real;
#else
typedef double so101_real;
#endif

enum { SO101_PAN, SO101_LIFT, SO101_ELBOW, SO101_FLEX, SO101_ROLL, SO101_GRIP, SO101_NJ };   /* joints, servo ids 1-6 */
#define SO101_NA 5      /* the arm joints, pan to roll */
#define SO101_NL 7      /* links: base, shoulder, upper arm, lower arm, wrist, gripper, moving jaw */

enum {
    SO101_OK = 0,
    SO101_EUNREACHABLE = -1,   /* no allowed pose reaches the point */
    SO101_ECOLLIDES = -2,      /* every allowed pose that reaches it collides */
    SO101_EPATH = -3,          /* the way there runs into a range end, the table or the arm itself */
    SO101_EIO = -4,            /* a servo did not answer, or answered wrong */
    SO101_ECAL = -5,           /* a servo's Homing_Offset or limits differ from the calibration given */
    SO101_ESTALL = -6,         /* a joint stalled behind its goal */
    SO101_ELOAD = -7,          /* a joint overloaded */
    SO101_ESTOP = -8,          /* hal.stop() asked to stop */
    SO101_ESTATE = -9,         /* not now: no torque, no pose to go on from */
    SO101_EMOVED = -10,        /* a joint moved when the torque came on */
};
const char *so101_strerror(int code);

/* ---- the model ------------------------------------------------------------------------------- */

typedef struct {        /* what the IK keeps for one wrist roll and gripper: the arm in its plane */
    int valid;
    so101_real roll, grip, payload, off[SO101_NA];
    so101_real s[4];                       /* the pan axis's z, and lift, elbow, wrist flex axes' y: +-1 */
    so101_real pan_x, pan_y;               /* where the pan axis stands */
    so101_real o1[2], a[2], b[2], c3[2];   /* lift axis; lift->elbow, elbow->wrist, wrist->tip at zero */
    so101_real la, lb, delta0, cd0, sd0;   /* lengths of a and b, the angle from a to b and its cos, sin */
    so101_real lc3, beta_c, d_ang;         /* length and angle of c3, the gripper axis's angle */
    so101_real tip_y, roll_y;              /* lateral offsets of the tip and the roll axis */
    so101_real roll_pt[2];                 /* wrist->roll axis at zero */
    so101_real mu1[2], mu2[2], mu3[2], m2, m3;   /* moments of the links beyond each flex axis (kg m) */
    so101_real ra[3], rr[3];               /* roll axis at zero; the moment of gripper and jaw about it */
    so101_real kc, l2, kcomf[4];           /* 1 / (2 la lb), la^2 + lb^2; 2 / range width for the comfort term */
} so101_planar;

typedef struct {
    so101_real offsets[SO101_NA];   /* degrees: URDF angle = calibrated angle + offset (0 0 0 0 0) */
    so101_real lo[SO101_NA], hi[SO101_NA];   /* calibrated ranges, degrees */
    so101_real payload;          /* kg between the jaws, a point mass at the tip */
    so101_real table_z;          /* m; -1e9 for no table */
    so101_real margin;           /* degrees kept inside each range by the IK (5) */
    so101_real clearance;        /* elbow, wrist and roll axes above the table (0.03 m) */
    so101_real tip_clearance;    /* the tip above the table (0.005 m) */
    so101_real gap;              /* the least distance between parts, and to the table (0.008 m) */
    so101_real comfort;          /* tie-breaker: N^2 m^2 at a range end (0.005) */
    so101_real smooth;           /* along a path: N^2 m^2 per degree^2 of joint change (0.0006) */
    so101_real pitch_step;       /* degrees between the pitches the search starts from (5) */
    so101_real near_window;      /* along a path, degrees of pitch either way of the pose before (10) */
    int base_rule;               /* the roll axis and tip kept out of the base (1) */
    int collisions;              /* the IK checks the meshes (1) */
    so101_planar pl;
} so101_model;

typedef struct {
    so101_real deg[SO101_NJ];       /* calibrated degrees, gripper 0-100 */
    so101_real tip[3];              /* gripper_frame_link, between the jaw tips, m */
    so101_real pitch;               /* the gripper axis against the horizontal: 0 forwards, -90 down */
    so101_real torque[SO101_NA];    /* N m each servo holds against gravity, positive in its positive direction */
    so101_real cost;                /* sum of squared torques of lift, elbow, wrist flex and roll, N^2 m^2 */
    so101_real score;               /* cost and the comfort term: what so101_ik() minimises */
    so101_real elbow[3], wrist[3], roll[3];   /* the axes' points */
} so101_pose;

typedef struct { int id, homing_offset, range_min, range_max; } so101_servo_cal;
/* A LeRobot calibration compiled in by tools/gen_model.py, for a board without a file system. */
extern const so101_servo_cal SO101_DEFAULT_CAL[SO101_NJ];

void so101_model_init(so101_model *m);   /* the defaults in brackets above, SO101_DEFAULT_CAL's ranges */
void so101_model_ranges(so101_model *m, const so101_servo_cal cal[SO101_NJ]);   /* ranges from a calibration */
void so101_model_stock(so101_model *m);   /* LeRobot's view: no offsets, the URDF's limits, no table or meshes */

void so101_pose_eval(const so101_model *m, const so101_real deg[SO101_NJ], so101_pose *out);
void so101_tip(const so101_model *m, const so101_real deg[SO101_NJ], so101_real tip[3], so101_real *pitch);
/* The tip frame (row-major 4x4) for URDF angles in degrees, pan to roll: LeRobot's forward_kinematics. */
void so101_fk_urdf(const so101_real q_deg[SO101_NA], so101_real T[16]);

/* Why a pose is not allowed, as bits (so101_problem_name for each); 0 if it is. */
enum { SO101_P_PAN = 1, SO101_P_LIFT = 2, SO101_P_ELBOW = 4, SO101_P_FLEX = 8, SO101_P_ELBOW_LOW = 16, SO101_P_WRIST_LOW = 32,
       SO101_P_ROLL_LOW = 64, SO101_P_TIP_LOW = 128, SO101_P_ROLL_BASE = 256, SO101_P_TIP_BASE = 512 };
unsigned so101_problems(const so101_model *m, const so101_pose *p, so101_real margin);
const char *so101_problem_name(unsigned bit);

/* 0 if no two parts that do not share a joint come within gap (m) of each other or of the table, by
 * the URDF's meshes (points about 4 mm apart); else which (so101_collision_name). */
int so101_collides(const so101_model *m, const so101_real deg[SO101_NJ], so101_real gap);
const char *so101_collision_name(int code);

/* ---- inverse kinematics ---------------------------------------------------------------------- */

/* The allowed pose with the least so101_pose.score of all that put the tip at xyz with this wrist roll
 * and gripper: inside the ranges less the margin, the axes above the table, no part within the gap. */
int so101_ik(so101_model *m, const so101_real xyz[3], so101_real roll, so101_real grip, so101_pose *out);
/* The same at a given pitch. near: a pose close by, whose elbow branch is tried first; may be NULL. */
int so101_ik_pitch(so101_model *m, const so101_real xyz[3], so101_real pitch, so101_real roll, so101_real grip,
                const so101_pose *near, so101_pose *out);
/* The next pose of a path: the least score plus smooth for each degree^2 of change from prev, within
 * near_window degrees of pitch of it. Wrist roll and gripper are prev's. */
int so101_ik_near(so101_model *m, const so101_real xyz[3], const so101_pose *prev, so101_pose *out);

/* A tip frame from the joints the arm has, as LeRobot's RobotKinematics.inverse_kinematics takes it:
 * URDF degrees pan to roll in q (changed in place), the target as a row-major 4x4, weights for its
 * position (m) and orientation (rad), at most iters Gauss-Newton steps inside the URDF's limits; it
 * stops when the step is under 1e-9 rad. Returns the steps taken; *err is the tip's distance from the
 * target position (m). With fallback set and a position that the steps did not reach within 1 mm,
 * the pose of fallback (so101_model_stock's, say) at the arm's present pitch is taken, if it has one. */
int so101_ik_local(so101_real q[SO101_NA], const so101_real T[16], so101_real wp, so101_real wo, int iters,
                so101_model *fallback, so101_real *err);

/* ---- the arm --------------------------------------------------------------------------------- */

typedef struct {
    double t;                 /* s since so101_arm_init */
    const char *step;
    so101_real goal[SO101_NJ], pos[SO101_NJ], load[SO101_NJ], ma[SO101_NJ];   /* degrees, % of the servo's torque, mA */
    so101_real tip[3], target[3];   /* from the present position; what the goals aim at (NAN if nothing) */
} so101_tick;

typedef struct {
    int (*write)(void *ctx, const uint8_t *buf, int n);                    /* bytes written, <0 error */
    int (*read)(void *ctx, uint8_t *buf, int n, uint32_t timeout_us);       /* bytes read before timeout */
    void (*drain)(void *ctx);                                               /* drop unread input */
    uint64_t (*now_us)(void *ctx);
    void (*sleep_us)(void *ctx, uint32_t us);
    int (*stop)(void *ctx);                       /* nonzero: stop the move now (Ctrl-C, a button); may be NULL */
    void (*log)(void *ctx, const char *line);     /* events; may be NULL */
    void (*tick)(void *ctx, const so101_tick *t); /* every tick of a move; may be NULL */
    void *ctx;
} so101_hal;

typedef struct {
    so101_hal hal;
    so101_model model;
    so101_servo_cal cal[SO101_NJ];
    so101_real goal[SO101_NJ];      /* the wanted angles last sent */
    so101_pose pose;                /* the IK pose last commanded; have_pose says if there is one */
    int have_pose, torque;
    so101_real target[3];
    int have_target;
    so101_real start[SO101_NJ];   /* the start pose: where the arm stood at torque-on if folded there, else the
                                     fold found from the ranges; unfold() leaves it, home() comes back to it */
    int folded;                   /* the arm stood folded at torque-on */
    char step[80], err[200];  /* what the arm is doing; why the last call failed */
    uint64_t t0;
    /* settings (defaults in brackets): */
    uint32_t tick_us;            /* between goals (20000) */
    so101_real lag_max;          /* degrees behind the goal with no progress for 0.3 s: stalled (7) */
    so101_real load_max;         /* % of the servo's torque for 0.3 s: overloaded (45) */
    so101_real sag;              /* degrees per N m of holding torque the goals lead by (3.7; 0 for none) */
    int acceleration, torque_limit;   /* RAM settings written at torque-on (254, 500) */
} so101_arm;


void so101_arm_init(so101_arm *a, const so101_hal *hal, const so101_servo_cal cal[SO101_NJ]);
int so101_arm_check(so101_arm *a);             /* every servo answers, its EEPROM matches cal */
int so101_arm_read(so101_arm *a, so101_real pos[SO101_NJ], so101_real load[SO101_NJ], so101_real ma[SO101_NJ]);
int so101_arm_torque_on(so101_arm *a);         /* goals = present positions, RAM settings, torque on */
int so101_arm_torque_off(so101_arm *a);
int so101_arm_move_joints(so101_arm *a, const so101_real target[SO101_NJ], so101_real speed, int check);   /* deg/s */
int so101_arm_move_to(so101_arm *a, const so101_real xyz[3], const so101_real *pitch, so101_real speed);   /* deg/s */
int so101_arm_line_to(so101_arm *a, const so101_real xyz[3], so101_real speed);                            /* m/s */
int so101_arm_move_by(so101_arm *a, const so101_real dxyz[3], so101_real speed);                           /* m/s */
int so101_arm_unfold(so101_arm *a);            /* from the start pose to it with the wrist straight up */
int so101_arm_home(so101_arm *a);              /* from anywhere to the start pose, torque off */

/* src/hal_posix.c: a serial port at 1 Mbaud on macOS or Linux. stop, log and tick are left NULL. */
int so101_hal_posix_open(so101_hal *hal, const char *port);
void so101_hal_posix_close(so101_hal *hal);

#ifdef __cplusplus
}
#endif
#endif
