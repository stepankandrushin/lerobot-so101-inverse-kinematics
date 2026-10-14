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

/* The links in chain order, then the tip frame: base_link, shoulder_link, upper_arm_link, lower_arm_link,
 * wrist_link, gripper_link, moving_jaw_so101_v1_link (the URDF's names), gripper_frame_link. */
const char *so101_link_name(int link);
/* Each link's frame and the tip frame in the base frame, row-major 4x4, for calibrated degrees. */
void so101_link_frames(const so101_model *m, const so101_real deg[SO101_NJ], so101_real T[SO101_NL + 1][16]);
/* The surface points of a link's mesh, in the link's own frame, about 4 mm apart; returns how many. */
int so101_link_points(int link, const float (**pts)[3]);
/* Where the vertical pan axis stands in the base frame, m. */
#define SO101_PAN_AXIS_X ((so101_real)0.0388353)
#define SO101_PAN_AXIS_Y ((so101_real)0.0)
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

/* ---- fast runs: a path planned whole, then replayed as fast as the servos go (src/path.c) ------ */

typedef struct {
    so101_real vmax;          /* degrees/s a joint is planned for (220) */
    so101_real amax;          /* degrees/s^2 (1500) */
    so101_real top;           /* degrees/s no run may ask of a joint, the servos' own top speed (295) */
    so101_real dt;            /* s between goals (0.01) */
    so101_real lag, dead;     /* s: a servo follows a streamed goal as a lag behind a dead time (0.060, 0.022) */
    so101_real smooth;        /* s over which the speed along the path is smoothed (0.04) */
    so101_real gap;           /* m between parts, and to the table, along a run (0.02) */
    so101_real err_max;       /* degrees from where a joint should be, for 0.15 s: the run stops (12) */
    so101_real tube;          /* the same on the way from the start pose and back (6) */
    int torque_limit, acceleration;   /* the arm servos' RAM settings during a run (1000, 0) */
} so101_fast;
void so101_fast_init(so101_fast *f);

/* The joints along tip points xyz[0..n-1] (m, a few mm apart), each pose solved from the one before:
 * at pitch[i] degrees, starting on elbow branch +1 (branch >= 0) or -1, or with pitch NULL in the
 * least-loaded pose near the one before. Every pose allowed, the parts gap apart (every 5th point), no
 * joint more than 3 degrees from the pose before. A closed path (its last point its first) must come
 * back to its first pose, which q[n-1] then is. SO101_OK, or the error with *at the point it failed at. */
int so101_path_poses(so101_model *m, const so101_real (*xyz)[3], int n, const so101_real *pitch, so101_real roll,
                     so101_real grip, int branch, so101_real gap, so101_real (*q)[SO101_NJ], int *at);
/* The fastest timing of the joint path q[0..n-1] gone round laps times (laps > 1 for a closed path),
 * starting and ending at rest, each arm joint under vmax and amax: t[i] (s) at which it passes point i,
 * (n - 1) laps + 1 of them. Returns the time of the whole. */
so101_real so101_path_time(const so101_real (*q)[SO101_NJ], int n, int laps, const so101_fast *f, so101_real *t);
/* That run every f->dt, slowed to speed (1: the limits), its speed along the path smoothed over f->smooth:
 * out[k] the joints at tick k, at[k] (may be NULL) the place along the path as a point index and a
 * fraction. Returns the number of ticks, or -1 if more than cap. */
int so101_path_sample(const so101_real (*q)[SO101_NJ], int n, int laps, const so101_real *t, so101_real speed,
                      const so101_fast *f, so101_real (*out)[SO101_NJ], so101_real *at, int cap);
/* The pitch along a closed path that makes its lap shortest: the best constant pitch, then a mean and three
 * harmonics of the place along the lap, one coefficient at a time in steps of 16 degrees down to 2. pitch[n]
 * gets it, q[n] its joints, *lap the time of a lap between two others; work: SO101_FASTEST_WORK(n) reals. */
#define SO101_FASTEST_WORK(n) (10 * (n) + 4)
int so101_path_fastest(so101_model *m, const so101_real (*xyz)[3], int n, so101_real roll, so101_real grip,
                       const so101_fast *f, so101_real *pitch, so101_real (*q)[SO101_NJ], so101_real *lap, so101_real *work);
/* A sampled run against the model as it will be driven: each arm joint 2 degrees inside its range and under
 * f->top, the parts f->gap apart and from the table (every 5th tick). 0 if it passes, else 1 and why. */
int so101_run_check(const so101_model *m, const so101_real (*q)[SO101_NJ], int n, const so101_fast *f, char *why,
                    int nwhy);
/* Iterative learning: after a replay that read pos, fix[k] += gain times what each arm joint was off at tick
 * k, smoothed over 25 ms both ways in time and put through the inverse of how a servo follows a goal. */
void so101_path_learn(const so101_real (*q)[SO101_NJ], const so101_real (*pos)[SO101_NJ], int n, const so101_fast *f,
                      so101_real gain, so101_real (*fix)[SO101_NA]);
/* The way from the folded pose a to the pose b, half a degree a point: wrist flex, elbow, roll and gripper
 * evenly, lift and pan from a quarter of the way on, so the gripper comes up off the table before the upper
 * arm comes forward. Backwards it folds. Returns the points, or -1 if more than cap. */
int so101_unfold_path(const so101_real a[SO101_NJ], const so101_real b[SO101_NJ], so101_real (*q)[SO101_NJ], int cap);
/* 1 if nothing touches along that way (the meshes, the table), also with lift, elbow and wrist flex each
 * tube degrees ahead or behind, in all eight combinations; else 0. */
int so101_unfold_clear(const so101_model *m, const so101_real a[SO101_NJ], const so101_real b[SO101_NJ], so101_real tube);

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
    /* The lead (so101_follow.lead): a servo stands still until its goal is 1-1.5 degrees away (gearbox
     * friction, P control), so each of pan, lift, elbow and wrist flex trails its goal along a chain. The
     * goal is led ahead by what the joint lags: lead_rate of the lag per second, at most lead_max degrees,
     * a lag under lead_min left alone. */
    so101_real lead[4];          /* degrees, pan to wrist flex: kept from one chain to the next; 0 at torque-on */
    so101_real lead_rate;        /* per second (5) */
    so101_real lead_max;         /* degrees (1.6) */
    so101_real lead_min;         /* degrees (0.1) */
    so101_real start_max;        /* degrees a chain's first pose may lie from the goals, pan to wrist flex (5) */
    int contact;                 /* the last so101_arm_follow with a press stopped on contact */
} so101_arm;

/* How so101_arm_follow goes along a chain of poses. so101_follow_init sets the defaults in brackets. */
typedef struct {
    so101_real speed;            /* the tip's peak speed, m/s, which sets the duration from the chain's length (0.03) */
    so101_real duration;         /* s for the whole chain instead, if over 0 (0) */
    int steady;                  /* the goals at a steady pace, not on one minimum-jerk profile (0) */
    so101_real settle;           /* s the goals hold the last pose (0.3) */
    so101_real within;           /* over 0: the hold ends as soon as pan, lift, elbow and wrist flex stand this
                                    close to the last pose, degrees (0) */
    int lead;                    /* the goals led by what the joints lag (so101_arm.lead) (0) */
    /* Lowering until contact: with press over 0 the move stops once the tip, from the servos' positions, stands
     * press m higher above the goals' tip than it did while it moved freely (the median over free_from to
     * free_to of the duration) for press_ticks ticks running. It ends there or at the chain's end with every
     * goal where its joint stands (the gripper's as it was), the lead dropped, held for settle; contact says
     * which. */
    so101_real press;            /* m (0) */
    so101_real free_from, free_to;   /* fractions of the duration (0.15, 0.5) */
    int press_ticks;             /* (3) */
} so101_follow;
void so101_follow_init(so101_follow *o);


void so101_arm_init(so101_arm *a, const so101_hal *hal, const so101_servo_cal cal[SO101_NJ]);
int so101_arm_check(so101_arm *a);             /* every servo answers, its EEPROM matches cal */
int so101_arm_read(so101_arm *a, so101_real pos[SO101_NJ], so101_real load[SO101_NJ], so101_real ma[SO101_NJ]);
int so101_arm_torque_on(so101_arm *a);         /* goals = present positions, RAM settings, torque on */
int so101_arm_torque_off(so101_arm *a);
int so101_arm_move_joints(so101_arm *a, const so101_real target[SO101_NJ], so101_real speed, int check);   /* deg/s */
int so101_arm_move_to(so101_arm *a, const so101_real xyz[3], const so101_real *pitch, so101_real speed);   /* deg/s */
int so101_arm_line_to(so101_arm *a, const so101_real xyz[3], so101_real speed);                            /* m/s */
int so101_arm_move_by(so101_arm *a, const so101_real dxyz[3], so101_real speed);                           /* m/s */
/* Through n poses (calibrated degrees, all six joints), linearly between them, as o says. The first must be
 * within start_max degrees of the goals, neighbours within 3 of each other (pan to wrist flex), every pose inside
 * the ranges by a degree; nothing else is checked: the caller planned the chain. */
int so101_arm_follow(so101_arm *a, const so101_real (*q)[SO101_NJ], int n, const so101_follow *o);
/* Close the gripper at rate percent/s until the jaw has stood still for 0.25 s with its goal past percent
 * beyond it, or the goal is at 0; *at is where it stopped. Short of empty it holds nothing and the goal stays
 * there; else the goal goes squeeze past it and holds 0.4 s. The overload rule leaves the gripper alone here
 * (its Torque_Limit caps what it presses with); the stall rule never takes the gripper. */
int so101_arm_grasp(so101_arm *a, so101_real rate, so101_real past, so101_real squeeze, so101_real empty, so101_real *at);
/* The tip straight up to height z (m) at speed (m/s), without the table checks: the way out of a pose at
 * the table. A joint at its range end stops it there with SO101_EPATH. */
int so101_arm_rise(so101_arm *a, so101_real z, so101_real speed);
int so101_arm_unfold(so101_arm *a);            /* from the start pose to it with the wrist straight up */
int so101_arm_home(so101_arm *a);              /* from anywhere to the start pose, torque off */

/* src/hal_posix.c: a serial port at 1 Mbaud on macOS or Linux. stop, log and tick are left NULL. */
int so101_hal_posix_open(so101_hal *hal, const char *port);
void so101_hal_posix_close(so101_hal *hal);

/* src/sim.c: six simulated STS servos behind so101_hal, answering the packets the real ones do, on a clock of
 * their own (a minute of moves runs in milliseconds), to try programs without an arm. A servo goes towards
 * its goal faster the further it is, and not at all within SO101_SIM_DEAD steps of it (a real one's gearbox
 * friction); its load grows slowly with that distance, fast when something holds the joint back. Nothing
 * pulls it down: no gravity, no sag. */
#define SO101_SIM_DEAD 12
typedef struct {
    uint8_t reg[SO101_NJ][256];     /* each servo's control table */
    double pos[SO101_NJ];           /* raw steps */
    int lo_block[SO101_NJ], hi_block[SO101_NJ];   /* raw steps a joint cannot pass (an object in the jaws); -1 none */
    int held[SO101_NJ];             /* something held the joint back in the last millisecond */
    double floor_z;                 /* m: the tip does not go below it (a table, a board); -1e9 none */
    uint64_t now;                   /* us */
    uint8_t rx[4096];
    int rx_n, rx_at;
    so101_model m;                  /* for the floor: the URDF, no offsets unless set here */
    so101_servo_cal cal[SO101_NJ];
} so101_sim;
/* The servos holding cal (NULL: SO101_DEFAULT_CAL) at deg, calibrated degrees, torque off. */
void so101_sim_init(so101_sim *s, const so101_servo_cal cal[SO101_NJ], const so101_real deg[SO101_NJ]);
/* A HAL on s; stop, log and tick are left NULL. */
void so101_hal_sim(so101_hal *hal, so101_sim *s);

/* ---- fast runs on the arm (src/fast.c) --------------------------------------------------------- */

/* The arm servos at f->torque_limit and f->acceleration for a run (on), or back at the arm's torque_limit
 * and acceleration. */
int so101_arm_fast(so101_arm *a, const so101_fast *f, int on);
/* Replay a sampled run q[0..n-1], a tick every f->dt, the arm standing at q[0], until rest s after the
 * last. Each arm joint's goal leads it by how a servo follows: where it should be f->dead later plus
 * f->lag times its speed there, plus the sag compensation and fix[k] (learned; may be NULL). A joint
 * err_max degrees from where it should be for 0.15 s, a load of 99 % for 0.4 s, or hal.stop() freezes every
 * goal where its joint stands. tip (may be NULL): where the tip should be, for the ticks. pos and load (may
 * be NULL) get what was read at each tick, for so101_path_learn. */
int so101_arm_play(so101_arm *a, const so101_fast *f, const so101_real (*q)[SO101_NJ], int n, const so101_real (*tip)[3],
                   const so101_real (*fix)[SO101_NA], so101_real rest, so101_real err_max, so101_real (*pos)[SO101_NJ],
                   so101_real (*load)[SO101_NJ]);

#ifdef __cplusplus
}
#endif
#endif
