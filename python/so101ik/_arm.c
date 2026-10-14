/* so101ik._core.Arm: the follower on a serial port, the C library's so101_arm. */
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <pythread.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "so101ik.h"

static PyObject *Fault, *Unreachable;
static PyTypeObject *PoseType;
static PyObject *(*new_pose)(const so101_pose *);
static int (*read_deg)(PyObject *, so101_real *);
static int (*read3)(PyObject *, so101_real *);

typedef struct {
    PyObject_HEAD
    so101_arm arm;
    so101_hal io;           /* the serial port, or the simulated servos */
    int open;
    int is_sim;             /* port "sim": sim stands in for the servos (src/sim.c) */
    so101_sim sim;
    PyObject *log;       /* a callable for each event line, or None: stdout */
    PyObject *tick;      /* a callable for each tick of a move, or None */
    FILE *csv;
    int interrupted;     /* a signal handler or the tick callable raised while a move ran */
    int stop_req;        /* stop() asked: the move running, or the next, stops at its next tick */
    unsigned long busy;  /* the thread whose call has the port, or 0 */
} ArmObject;

static const char *NAMES[SO101_NJ] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};

/* ---- the HAL: the port's calls, and stop, log and tick for Python ------------------------------ */

/* The port and the clock without the GIL, so other threads (a camera's, say) run while the arm waits. */
static int h_write(void *c, const uint8_t *b, int n) {
    ArmObject *o = c;
    int r;
    Py_BEGIN_ALLOW_THREADS r = o->io.write(o->io.ctx, b, n);
    Py_END_ALLOW_THREADS return r;
}
static int h_read(void *c, uint8_t *b, int n, uint32_t t) {
    ArmObject *o = c;
    int r;
    Py_BEGIN_ALLOW_THREADS r = o->io.read(o->io.ctx, b, n, t);
    Py_END_ALLOW_THREADS return r;
}
static void h_drain(void *c) { ArmObject *o = c; o->io.drain(o->io.ctx); }
static uint64_t h_now(void *c) { ArmObject *o = c; return o->io.now_us(o->io.ctx); }
static void h_sleep(void *c, uint32_t us) {
    ArmObject *o = c;
    Py_BEGIN_ALLOW_THREADS o->io.sleep_us(o->io.ctx, us);
    Py_END_ALLOW_THREADS
}

static int h_stop(void *c) {
    ArmObject *o = c;
    if (o->interrupted) return 1;
    if (!o->stop_req && PyErr_CheckSignals() < 0) {   /* Ctrl-C (or a handler of SIGTERM) raised: stop, keep it */
        o->interrupted = 1;
        return 1;
    }
    if (o->stop_req) {                  /* stop(), from another thread, a callback or a signal handler */
        o->stop_req = 0;
        return 1;
    }
    return 0;
}

static void h_log(void *c, const char *line) {
    ArmObject *o = c;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    char b[520];
    snprintf(b, sizeof b, "%.2f %s", tv.tv_sec + tv.tv_usec / 1e6, line);
    PyObject *et, *ev, *tb;
    PyErr_Fetch(&et, &ev, &tb);           /* a pending KeyboardInterrupt waits for the log */
    if (o->log && o->log != Py_None) {
        PyObject *r = PyObject_CallFunction(o->log, "s", b);
        if (!r) PyErr_Clear();
        Py_XDECREF(r);
    } else {
        printf("%s\n", b);
        fflush(stdout);
    }
    PyErr_Restore(et, ev, tb);
}

static PyObject *by_joint(const so101_real *v) {
    PyObject *d = PyDict_New();
    for (int j = 0; j < SO101_NJ; j++) {
        PyObject *x = PyFloat_FromDouble(v[j]);
        PyDict_SetItemString(d, NAMES[j], x);
        Py_DECREF(x);
    }
    return d;
}

static void h_tick(void *c, const so101_tick *t) {
    ArmObject *o = c;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double now = tv.tv_sec + tv.tv_usec / 1e6;
    if (o->csv) {
        fprintf(o->csv, "%.3f,%s", now, t->step);
        for (int j = 0; j < SO101_NJ; j++)
            fprintf(o->csv, ",%.2f,%.2f,%.1f,%.0f", (double)t->goal[j], (double)t->pos[j], (double)t->load[j], (double)t->ma[j]);
        for (int i = 0; i < 3; i++) fprintf(o->csv, ",%.1f", 1000 * (double)t->tip[i]);
        for (int i = 0; i < 3; i++) fprintf(o->csv, isnan(t->target[i]) ? "," : ",%.1f", 1000 * (double)t->target[i]);
        fprintf(o->csv, "\n");
    }
    if (o->tick && o->tick != Py_None && !o->interrupted) {
        /* an exception in it stops the move at the next tick and comes out of the call */
        PyObject *r = PyObject_CallFunction(o->tick, "{s:d,s:s,s:N,s:N,s:N,s:N,s:(ddd),s:(ddd)}", "time", now, "step", t->step,
                                            "goal", by_joint(t->goal), "pos", by_joint(t->pos), "load", by_joint(t->load),
                                            "ma", by_joint(t->ma), "tip", (double)t->tip[0], (double)t->tip[1],
                                            (double)t->tip[2], "target", (double)t->target[0], (double)t->target[1],
                                            (double)t->target[2]);
        if (r) Py_DECREF(r);
        else o->interrupted = 1;
    }
}

/* ---- errors ---------------------------------------------------------------------------------- */

static PyObject *done(ArmObject *o, int rc) {
    if (rc == SO101_OK) Py_RETURN_NONE;
    if (o->interrupted) {
        o->interrupted = 0;
        if (PyErr_Occurred()) return NULL;
        PyErr_SetNone(PyExc_KeyboardInterrupt);
        return NULL;
    }
    PyErr_SetString(rc == SO101_EUNREACHABLE ? Unreachable : Fault, o->arm.err[0] ? o->arm.err : so101_strerror(rc));
    return NULL;
}

#define OPEN(o) if (!(o)->open) { PyErr_SetString(Fault, "the port is closed"); return NULL; }

/* One call on the port at a time: another thread's call waits for nothing and fails. A call from inside a
 * running one (a signal handler, the tick callable: between two ticks, the port idle) may only switch the
 * torque off. */
static int take(ArmObject *o, int reentrant_ok) {
    unsigned long me = PyThread_get_thread_ident();
    if (o->busy && !(reentrant_ok && o->busy == me)) {
        PyErr_SetString(Fault, o->busy == me ? "the arm is in a move: only torque_off from inside it"
                                             : "the arm is busy with another thread's call");
        return 0;
    }
    return 1;
}
#define TAKE(o)                       \
    if (!take(o, 0)) return NULL;     \
    o->busy = PyThread_get_thread_ident();
#define GIVE(o) o->busy = 0

static PyObject *call_done(ArmObject *o, int rc) {
    GIVE(o);
    return done(o, rc);
}

/* ---- the type -------------------------------------------------------------------------------- */

static int arm_init(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"port", "calibration", "log", "csv", "on_tick", NULL};
    const char *port, *csv = NULL;
    PyObject *cal, *log = Py_None, *tick = Py_None;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "sO|OzO", kws, &port, &cal, &log, &csv, &tick)) return -1;
    /* No default: the calibration compiled in is another arm's, and the servos' EEPROM would refuse it. */
    so101_servo_cal c[SO101_NJ];
    PyObject *seq = PySequence_Fast(cal, "calibration: six (id, homing_offset, range_min, range_max)");
    if (!seq) return -1;
    if (PySequence_Fast_GET_SIZE(seq) != SO101_NJ) {
        Py_DECREF(seq);
        PyErr_SetString(PyExc_ValueError, "calibration: six (id, homing_offset, range_min, range_max), pan to gripper");
        return -1;
    }
    for (int j = 0; j < SO101_NJ; j++)
        if (!PyArg_ParseTuple(PySequence_Fast_GET_ITEM(seq, j), "iiii", &c[j].id, &c[j].homing_offset,
                              &c[j].range_min, &c[j].range_max)) {
            Py_DECREF(seq);
            return -1;
        }
    Py_DECREF(seq);
    if (!strcmp(port, "sim")) {
        /* folded in the start pose the calibration's ranges give, as so101_arm_init finds it */
        so101_model m;
        so101_model_init(&m);
        so101_model_ranges(&m, c);
        so101_real fold[SO101_NJ] = {0, m.lo[SO101_LIFT] + 2, m.hi[SO101_ELBOW] - 3, 76.5, 0, 2};
        if (fold[SO101_FLEX] > m.hi[SO101_FLEX] - 3) fold[SO101_FLEX] = m.hi[SO101_FLEX] - 3;
        so101_sim_init(&self->sim, c, fold);
        so101_hal_sim(&self->io, &self->sim);
        self->is_sim = 1;
    } else if (so101_hal_posix_open(&self->io, port) != SO101_OK) {
        PyErr_Format(Fault, "cannot open %s at 1 Mbaud", port);
        return -1;
    }
    self->open = 1;
    so101_hal h = {h_write, h_read, h_drain, h_now, h_sleep, h_stop, h_log, h_tick, self};
    so101_arm_init(&self->arm, &h, c);
    Py_INCREF(log);
    self->log = log;
    Py_INCREF(tick);
    self->tick = tick;
    if (csv) {
        self->csv = fopen(csv, "w");
        if (!self->csv) {
            PyErr_SetFromErrnoWithFilename(PyExc_OSError, csv);
            return -1;
        }
        fprintf(self->csv, "unix_time,step");
        for (int j = 0; j < SO101_NJ; j++) fprintf(self->csv, ",%s_goal,%s_pos,%s_load,%s_ma", NAMES[j], NAMES[j], NAMES[j], NAMES[j]);
        fprintf(self->csv, ",tip_x,tip_y,tip_z,target_x,target_y,target_z\n");
    }
    return 0;
}

static void arm_close_io(ArmObject *self) {
    if (self->open && !self->is_sim) so101_hal_posix_close(&self->io);
    self->open = 0;
    if (self->csv) fclose(self->csv);
    self->csv = NULL;
}

static void arm_dealloc(ArmObject *self) {
    arm_close_io(self);
    Py_XDECREF(self->log);
    Py_XDECREF(self->tick);
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *arm_close(ArmObject *self, PyObject *noargs) {
    if (!take(self, 0)) return NULL;
    arm_close_io(self);
    Py_RETURN_NONE;
}

static PyObject *arm_check(ArmObject *self, PyObject *noargs) {
    OPEN(self);
    TAKE(self);
    return call_done(self, so101_arm_check(&self->arm));
}

static PyObject *arm_read(ArmObject *self, PyObject *noargs) {
    OPEN(self);
    TAKE(self);
    so101_real pos[SO101_NJ], load[SO101_NJ], ma[SO101_NJ];
    int rc = so101_arm_read(&self->arm, pos, load, ma);
    if (rc) return call_done(self, rc);
    GIVE(self);
    PyObject *d = PyDict_New(), *l = PyDict_New(), *c = PyDict_New();
    for (int j = 0; j < SO101_NJ; j++) {
        PyObject *v;
        PyDict_SetItemString(d, NAMES[j], v = PyFloat_FromDouble(pos[j])); Py_DECREF(v);
        PyDict_SetItemString(l, NAMES[j], v = PyFloat_FromDouble(load[j])); Py_DECREF(v);
        PyDict_SetItemString(c, NAMES[j], v = PyFloat_FromDouble(ma[j])); Py_DECREF(v);
    }
    return Py_BuildValue("(NNN)", d, l, c);
}

static PyObject *arm_torque_on(ArmObject *self, PyObject *noargs) {
    OPEN(self);
    TAKE(self);
    return call_done(self, so101_arm_torque_on(&self->arm));
}

static PyObject *arm_torque_off(ArmObject *self, PyObject *noargs) {
    OPEN(self);
    if (!take(self, 1)) return NULL;
    unsigned long was = self->busy;
    self->busy = PyThread_get_thread_ident();
    int rc = so101_arm_torque_off(&self->arm);
    self->busy = was;
    return done(self, rc);
}

static PyObject *arm_stop(ArmObject *self, PyObject *noargs) {
    self->stop_req = 1;
    Py_RETURN_NONE;
}

/* Calibrated degrees (percent for the gripper) to the simulated servo's raw steps. */
static double sim_raw(const so101_sim *s, int j, double v) {
    double lo = s->cal[j].range_min, hi = s->cal[j].range_max;
    return j == SO101_GRIP ? lo + v / 100 * (hi - lo) : v * 4095 / 360 + (lo + hi) / 2;
}

static PyObject *arm_sim(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"floor", "block", "pose", NULL};
    PyObject *floor = Py_None, *block = Py_None, *pose = Py_None;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "|OOO", kws, &floor, &block, &pose)) return NULL;
    if (!self->is_sim) {
        PyErr_SetString(Fault, "sim(): the arm is not the simulated one (port 'sim')");
        return NULL;
    }
    if (!take(self, 0)) return NULL;
    so101_sim *s = &self->sim;
    if (floor != Py_None) {
        double z = PyFloat_AsDouble(floor);
        if (PyErr_Occurred()) return NULL;
        s->floor_z = z;
    }
    if (block != Py_None) {
        if (!PyDict_Check(block)) {
            PyErr_SetString(PyExc_TypeError, "block: {joint: (low, high)}, None for an open side; {} for none");
            return NULL;
        }
        for (int j = 0; j < SO101_NJ; j++) s->lo_block[j] = s->hi_block[j] = -1;
        PyObject *k, *v;
        Py_ssize_t i = 0;
        while (PyDict_Next(block, &i, &k, &v)) {
            const char *name = PyUnicode_AsUTF8(k);
            int j = -1;
            for (int n = 0; name && n < SO101_NJ; n++)
                if (!strcmp(name, NAMES[n])) j = n;
            PyObject *lo, *hi;
            if (j < 0 || !PyArg_ParseTuple(v, "OO", &lo, &hi)) {
                if (!PyErr_Occurred()) PyErr_Format(PyExc_KeyError, "block: no joint %R", k);
                return NULL;
            }
            if (lo != Py_None) s->lo_block[j] = (int)lround(sim_raw(s, j, PyFloat_AsDouble(lo)));
            if (hi != Py_None) s->hi_block[j] = (int)lround(sim_raw(s, j, PyFloat_AsDouble(hi)));
            if (PyErr_Occurred()) return NULL;
        }
    }
    if (pose != Py_None) {
        so101_real deg[SO101_NJ];
        if (!read_deg(pose, deg)) return NULL;
        for (int j = 0; j < SO101_NJ; j++) s->pos[j] = sim_raw(s, j, deg[j]);
        s->now += 1;             /* the readings follow at the next packet */
    }
    Py_RETURN_NONE;
}

static void set_step(ArmObject *self, const char *step) {
    if (step) snprintf(self->arm.step, sizeof self->arm.step, "%s", step);
}

static PyObject *arm_move_joints(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"target", "speed", "check", "step", NULL};
    PyObject *t;
    double speed = 30;
    int check = 1;
    const char *step = "joints";
    if (!PyArg_ParseTupleAndKeywords(args, kw, "O|dpz", kws, &t, &speed, &check, &step)) return NULL;
    OPEN(self);
    so101_real deg[SO101_NJ];
    memcpy(deg, self->arm.goal, sizeof deg);
    if (PyDict_Check(t)) {          /* the joints named, the rest where they are */
        for (int j = 0; j < SO101_NJ; j++) {
            PyObject *v = PyDict_GetItemString(t, NAMES[j]);
            if (v) deg[j] = PyFloat_AsDouble(v);
        }
        if (PyErr_Occurred()) return NULL;
    } else {
        deg[SO101_GRIP] = self->arm.goal[SO101_GRIP];
        if (!read_deg(t, deg)) return NULL;
    }
    TAKE(self);
    set_step(self, step ? step : "joints");
    self->arm.have_target = self->arm.have_pose = 0;
    return call_done(self, so101_arm_move_joints(&self->arm, deg, speed, check));
}

static PyObject *pose_or_raise(ArmObject *self, int rc) {
    GIVE(self);
    if (rc) return done(self, rc);
    return new_pose(&self->arm.pose);
}

/* Six degrees from a Pose, a dict or a sequence; the gripper where it is if the item does not say. */
static int read_keep_grip(ArmObject *self, PyObject *obj, so101_real deg[SO101_NJ]) {
    if (!read_deg(obj, deg)) return 0;
    int says = 1;
    if (PyDict_Check(obj)) says = PyDict_GetItemString(obj, "gripper") != NULL;
    else if (!PyObject_TypeCheck(obj, PoseType)) says = PySequence_Size(obj) >= SO101_NJ;
    if (!says) deg[SO101_GRIP] = self->arm.goal[SO101_GRIP];
    return 1;
}

static PyObject *arm_follow(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"poses", "speed", "duration", "steady", "settle", "within", "lead", "press", "free", "press_ticks",
                          "step", NULL};
    PyObject *poses;
    so101_follow o;
    so101_follow_init(&o);
    double speed = o.speed, duration = 0, settle = o.settle, within = 0, press = 0, from = o.free_from, to = o.free_to;
    int steady = 0, lead = 0, ticks = o.press_ticks;
    const char *step = NULL;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "O|ddpddpd(dd)iz", kws, &poses, &speed, &duration, &steady, &settle, &within,
                                     &lead, &press, &from, &to, &ticks, &step))
        return NULL;
    OPEN(self);
    PyObject *seq = PySequence_Fast(poses, "poses: a sequence of Poses, dicts or six angles");
    if (!seq) return NULL;
    Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
    so101_real (*q)[SO101_NJ] = PyMem_Malloc((n ? n : 1) * sizeof *q);
    if (!q) {
        Py_DECREF(seq);
        return PyErr_NoMemory();
    }
    for (Py_ssize_t i = 0; i < n; i++)
        if (!read_keep_grip(self, PySequence_Fast_GET_ITEM(seq, i), q[i])) {
            PyMem_Free(q);
            Py_DECREF(seq);
            return NULL;
        }
    Py_DECREF(seq);
    if (!take(self, 0)) {
        PyMem_Free(q);
        return NULL;
    }
    self->busy = PyThread_get_thread_ident();
    o.speed = speed, o.duration = duration, o.steady = steady, o.settle = settle, o.within = within, o.lead = lead;
    o.press = press, o.free_from = from, o.free_to = to, o.press_ticks = ticks;
    set_step(self, step ? step : "chain");
    int rc = so101_arm_follow(&self->arm, (const so101_real (*)[SO101_NJ])q, (int)n, &o);
    PyMem_Free(q);
    GIVE(self);
    if (rc) return done(self, rc);
    return PyBool_FromLong(self->arm.contact);
}

static PyObject *arm_grasp(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"rate", "past", "squeeze", "empty", "step", NULL};
    double rate = 40, past = 3, squeeze = 2.5, empty = 4;
    const char *step = "grasp";
    if (!PyArg_ParseTupleAndKeywords(args, kw, "|ddddz", kws, &rate, &past, &squeeze, &empty, &step)) return NULL;
    OPEN(self);
    TAKE(self);
    set_step(self, step ? step : "grasp");
    so101_real at = 0;
    int rc = so101_arm_grasp(&self->arm, rate, past, squeeze, empty, &at);
    GIVE(self);
    if (rc) return done(self, rc);
    return PyFloat_FromDouble(at);
}

static PyObject *arm_rise(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"z", "speed", NULL};
    double z, speed = 0.01;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "d|d", kws, &z, &speed)) return NULL;
    OPEN(self);
    TAKE(self);
    return call_done(self, so101_arm_rise(&self->arm, z, speed));
}

static PyObject *arm_move_to(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"x", "y", "z", "pitch", "speed", NULL};
    double x, y, z, speed = 30;
    PyObject *pitch = Py_None;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "ddd|Od", kws, &x, &y, &z, &pitch, &speed)) return NULL;
    OPEN(self);
    so101_real xyz[3] = {x, y, z}, ph = 0;
    if (pitch != Py_None) {
        ph = PyFloat_AsDouble(pitch);
        if (PyErr_Occurred()) return NULL;
    }
    TAKE(self);
    return pose_or_raise(self, so101_arm_move_to(&self->arm, xyz, pitch != Py_None ? &ph : NULL, speed));
}

static PyObject *arm_line_to(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"x", "y", "z", "speed", NULL};
    double x, y, z, speed = 0.03;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "ddd|d", kws, &x, &y, &z, &speed)) return NULL;
    OPEN(self);
    so101_real xyz[3] = {x, y, z};
    TAKE(self);
    return pose_or_raise(self, so101_arm_line_to(&self->arm, xyz, speed));
}

static PyObject *arm_move_by(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"dx", "dy", "dz", "speed", NULL};
    double x, y, z, speed = 0.03;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "ddd|d", kws, &x, &y, &z, &speed)) return NULL;
    OPEN(self);
    so101_real d[3] = {x, y, z};
    TAKE(self);
    return pose_or_raise(self, so101_arm_move_by(&self->arm, d, speed));
}

static PyObject *arm_unfold(ArmObject *self, PyObject *noargs) {
    OPEN(self);
    TAKE(self);
    return call_done(self, so101_arm_unfold(&self->arm));
}

static PyObject *arm_home(ArmObject *self, PyObject *noargs) {
    OPEN(self);
    TAKE(self);
    return call_done(self, so101_arm_home(&self->arm));
}

static PyObject *arm_log(ArmObject *self, PyObject *arg) {
    const char *s = PyUnicode_AsUTF8(arg);
    if (!s) return NULL;
    h_log(self, s);
    Py_RETURN_NONE;
}

static PyObject *get_goal(ArmObject *o, void *c) {
    PyObject *d = PyDict_New();
    for (int j = 0; j < SO101_NJ; j++) {
        PyObject *v = PyFloat_FromDouble(o->arm.goal[j]);
        PyDict_SetItemString(d, NAMES[j], v);
        Py_DECREF(v);
    }
    return d;
}
static PyObject *get_pose(ArmObject *o, void *c) {
    if (!o->arm.have_pose) Py_RETURN_NONE;
    return new_pose(&o->arm.pose);
}
static int set_pose(ArmObject *o, PyObject *v, void *c) {
    if (!v || v == Py_None) {
        o->arm.have_pose = 0;
        return 0;
    }
    so101_real deg[SO101_NJ];
    if (!read_deg(v, deg)) return -1;
    so101_pose_eval(&o->arm.model, deg, &o->arm.pose);    /* the tip and pitch as the arm's own model has them */
    o->arm.have_pose = 1;
    return 0;
}
static PyObject *get_step(ArmObject *o, void *c) { return PyUnicode_FromString(o->arm.step); }
static int set_step_attr(ArmObject *o, PyObject *v, void *c) {
    const char *s = v ? PyUnicode_AsUTF8(v) : NULL;
    if (!s) return -1;
    set_step(o, s);
    return 0;
}
static PyObject *get_contact(ArmObject *o, void *c) { return PyBool_FromLong(o->arm.contact); }
static PyObject *get_lead(ArmObject *o, void *c) {
    PyObject *t = PyTuple_New(4);
    for (int j = 0; j < 4; j++) PyTuple_SET_ITEM(t, j, PyFloat_FromDouble(o->arm.lead[j]));
    return t;
}
static int set_lead(ArmObject *o, PyObject *v, void *c) {
    if (!v) return -1;
    if (PyNumber_Check(v) && !PySequence_Check(v)) {
        double x = PyFloat_AsDouble(v);
        if (PyErr_Occurred()) return -1;
        for (int j = 0; j < 4; j++) o->arm.lead[j] = x;
        return 0;
    }
    PyObject *seq = PySequence_Fast(v, "lead: four degrees, pan to wrist flex, or one number for all");
    if (!seq) return -1;
    if (PySequence_Fast_GET_SIZE(seq) != 4) {
        Py_DECREF(seq);
        PyErr_SetString(PyExc_ValueError, "lead: four degrees, pan to wrist flex");
        return -1;
    }
    so101_real l[4];
    for (int j = 0; j < 4; j++) l[j] = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, j));
    Py_DECREF(seq);
    if (PyErr_Occurred()) return -1;
    memcpy(o->arm.lead, l, sizeof l);
    return 0;
}
static PyObject *get_torque(ArmObject *o, void *c) { return PyBool_FromLong(o->arm.torque); }
static PyObject *get_folded(ArmObject *o, void *c) { return PyBool_FromLong(o->arm.folded); }
static PyObject *get_start(ArmObject *o, void *c) {
    PyObject *d = PyDict_New();
    for (int j = 0; j < SO101_NJ; j++) {
        PyObject *v = PyFloat_FromDouble(o->arm.start[j]);
        PyDict_SetItemString(d, NAMES[j], v);
        Py_DECREF(v);
    }
    return d;
}

#define REAL_FIELD(name, field, doc)                                                                          \
    static PyObject *get_##name(ArmObject *o, void *c) { return PyFloat_FromDouble(o->arm.field); }            \
    static int set_##name(ArmObject *o, PyObject *v, void *c) {                                               \
        if (!v) return -1;                                                                                    \
        o->arm.field = PyFloat_AsDouble(v);                                                                   \
        return PyErr_Occurred() ? -1 : 0;                                                                     \
    }
REAL_FIELD(sag, sag, "")
REAL_FIELD(lag_max, lag_max, "")
REAL_FIELD(load_max, load_max, "")
REAL_FIELD(lead_rate, lead_rate, "")
REAL_FIELD(lead_max, lead_max, "")
REAL_FIELD(lead_min, lead_min, "")
REAL_FIELD(table_z, model.table_z, "")
REAL_FIELD(start_max, start_max, "")
static PyObject *get_offsets(ArmObject *o, void *c) {
    PyObject *t = PyTuple_New(SO101_NA);
    for (int j = 0; j < SO101_NA; j++) PyTuple_SET_ITEM(t, j, PyFloat_FromDouble(o->arm.model.offsets[j]));
    return t;
}
static int set_offsets(ArmObject *o, PyObject *v, void *c) {
    so101_real deg[SO101_NJ];
    if (!v || !read_deg(v, deg)) return -1;
    for (int j = 0; j < SO101_NA; j++) o->arm.model.offsets[j] = o->sim.m.offsets[j] = deg[j];   /* the sim's floor too */
    o->arm.model.pl.valid = 0;
    return 0;
}
static PyObject *get_tick(ArmObject *o, void *c) { return PyFloat_FromDouble(o->arm.tick_us / 1e6); }
static int set_tick(ArmObject *o, PyObject *v, void *c) {
    double t = v ? PyFloat_AsDouble(v) : -1;
    if (PyErr_Occurred()) return -1;
    if (t < 0.002 || t > 1) {
        PyErr_SetString(PyExc_ValueError, "tick: 0.002 to 1 s");
        return -1;
    }
    o->arm.tick_us = (uint32_t)(t * 1e6);
    return 0;
}

static PyGetSetDef arm_getset[] = {
    {"goal", (getter)get_goal, NULL, "the wanted angles last sent, calibrated degrees", NULL},
    {"pose", (getter)get_pose, (setter)set_pose,
     "the pose the arm was last sent to by move_to, line_to, move_by or follow, or None; line_to and move_by go on\n"
     "from it, so set it after a move_joints to a planned pose", NULL},
    {"step", (getter)get_step, (setter)set_step_attr, "what the arm is doing: the log's and the CSV's step", NULL},
    {"contact", (getter)get_contact, NULL, "the last follow with a press stopped on contact", NULL},
    {"lead", (getter)get_lead, (setter)set_lead, "degrees each of pan to wrist flex is led ahead (follow(lead=True))", NULL},
    {"lead_rate", (getter)get_lead_rate, (setter)set_lead_rate, "the lead grows by this much of the lag a second", NULL},
    {"lead_max", (getter)get_lead_max, (setter)set_lead_max, "degrees the lead goes to at most", NULL},
    {"lead_min", (getter)get_lead_min, (setter)set_lead_min, "a lag under this many degrees leaves the lead as it is", NULL},
    {"table_z", (getter)get_table_z, (setter)set_table_z, "the table's height for the arm's own checks, m", NULL},
    {"start_max", (getter)get_start_max, (setter)set_start_max, "degrees a follow's first pose may lie from the goals", NULL},
    {"torque", (getter)get_torque, NULL, "whether the torque is on", NULL},
    {"folded", (getter)get_folded, NULL, "whether the arm stood folded at torque-on", NULL},
    {"start_pose", (getter)get_start, NULL, "the start pose: folded, where home() goes", NULL},
    {"sag", (getter)get_sag, (setter)set_sag, "degrees per N m the goals lead by (0: none)", NULL},
    {"lag_max", (getter)get_lag_max, (setter)set_lag_max, "degrees behind the goal that count as stalled", NULL},
    {"load_max", (getter)get_load_max, (setter)set_load_max, "% of the servo's torque that counts as overload", NULL},
    {"tick", (getter)get_tick, (setter)set_tick, "seconds between goals", NULL},
    {"offsets", (getter)get_offsets, (setter)set_offsets, "degrees, pan to roll: URDF angle = calibrated + offset", NULL},
    {NULL}};

static PyObject *arm_play(ArmObject *self, PyObject *args, PyObject *kw);     /* fast runs, at the end of the file */
static PyObject *arm_fast(ArmObject *self, PyObject *args, PyObject *kw);

static PyMethodDef arm_methods[] = {
    {"check", (PyCFunction)arm_check, METH_NOARGS, "every servo answers and its EEPROM matches the calibration"},
    {"read", (PyCFunction)arm_read, METH_NOARGS, "read() -> (degrees, load %, mA), dicts by joint"},
    {"torque_on", (PyCFunction)arm_torque_on, METH_NOARGS, "goals to the present positions, RAM settings, torque on"},
    {"torque_off", (PyCFunction)arm_torque_off, METH_NOARGS, "torque off: the arm drops"},
    {"move_joints", (PyCFunction)(void (*)(void))arm_move_joints, METH_VARARGS | METH_KEYWORDS,
     "move_joints(target, speed=30, check=True, step='joints'): straight in joint space, peak deg/s of the joint that\n"
     "moves most"},
    {"follow", (PyCFunction)(void (*)(void))arm_follow, METH_VARARGS | METH_KEYWORDS,
     "follow(poses, speed=0.03, duration=0, steady=False, settle=0.3, within=0, lead=False, press=0, free=(0.15, 0.5),\n"
     "press_ticks=3, step=None) -> contact: through a chain of poses (so101_arm_follow)"},
    {"grasp", (PyCFunction)(void (*)(void))arm_grasp, METH_VARARGS | METH_KEYWORDS,
     "grasp(rate=40, past=3, squeeze=2.5, empty=4, step='grasp') -> where the jaw stopped, percent (so101_arm_grasp)"},
    {"rise", (PyCFunction)(void (*)(void))arm_rise, METH_VARARGS | METH_KEYWORDS,
     "rise(z, speed=0.01): the tip straight up to height z, m, without the table checks"},
    {"sim", (PyCFunction)(void (*)(void))arm_sim, METH_VARARGS | METH_KEYWORDS,
     "sim(floor=None, block=None, pose=None): the simulated arm's world: a floor the tip stops at (m), joints\n"
     "blocked at calibrated angles ({'gripper': (12.0, None)}: an object holds the jaw at 12 percent), the servos\n"
     "put at a pose"},
    {"stop", (PyCFunction)arm_stop, METH_NOARGS,
     "stop(): the move running, or the next one, stops at its next tick and raises Fault; any thread may ask"},
    {"move_to", (PyCFunction)(void (*)(void))arm_move_to, METH_VARARGS | METH_KEYWORDS,
     "move_to(x, y, z, pitch=None, speed=30) -> Pose: the tip to (x, y, z) m in the least-loaded pose, straight in\n"
     "joint space"},
    {"line_to", (PyCFunction)(void (*)(void))arm_line_to, METH_VARARGS | METH_KEYWORDS,
     "line_to(x, y, z, speed=0.03) -> Pose: the tip along a straight line, peak m/s"},
    {"move_by", (PyCFunction)(void (*)(void))arm_move_by, METH_VARARGS | METH_KEYWORDS,
     "move_by(dx, dy, dz, speed=0.03) -> Pose: a straight line from where the tip is, m"},
    {"unfold", (PyCFunction)arm_unfold, METH_NOARGS, "from the start pose to READY"},
    {"home", (PyCFunction)arm_home, METH_NOARGS, "back to the start pose from anywhere, torque off"},
    {"log", (PyCFunction)arm_log, METH_O, "log(line): an event line, with the time, where the arm's own go"},
    {"close", (PyCFunction)arm_close, METH_NOARGS, "close the port and the CSV"},
    {"play", (PyCFunction)(void (*)(void))arm_play, METH_VARARGS | METH_KEYWORDS,
     "play(q, tip=None, fix=None, pos=None, load=None, rest=0.3, err_max=None, step=None, settings=None): replay a\n"
     "sampled run (so101ik.fast), float64 arrays: q N x 6, tip N x 3, fix N x 5; pos and load N x 6 get the readings"},
    {"fast", (PyCFunction)(void (*)(void))arm_fast, METH_VARARGS | METH_KEYWORDS,
     "fast(on, settings=None): the arm servos at a fast run's Torque_Limit and Acceleration, or back"},
    {NULL}};

PyTypeObject ArmType = {
    PyVarObject_HEAD_INIT(NULL, 0).tp_name = "so101ik._core.Arm",
    .tp_basicsize = sizeof(ArmObject),
    .tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
    .tp_doc = "Arm(port, calibration, log=None, csv=None, on_tick=None): the follower on its servo bus. log(line) gets\n"
              "each event, on_tick(dict) each tick of a move (time, step, goal, pos, load, ma, tip, target).",
    .tp_new = PyType_GenericNew,
    .tp_init = (initproc)arm_init,
    .tp_dealloc = (destructor)arm_dealloc,
    .tp_methods = arm_methods,
    .tp_getset = arm_getset,
};

int arm_type_ready(PyObject *m, PyObject *fault, PyObject *unreachable, PyTypeObject *pose_type,
                   PyObject *(*np)(const so101_pose *), int (*rd)(PyObject *, so101_real *), int (*r3)(PyObject *, so101_real *)) {
    Fault = fault;
    Unreachable = unreachable;
    PoseType = pose_type;
    new_pose = np;
    read_deg = rd;
    read3 = r3;
    if (PyType_Ready(&ArmType) < 0) return -1;
    Py_INCREF(&ArmType);
    return PyModule_AddObject(m, "Arm", (PyObject *)&ArmType);
}

/* ---- fast runs (src/fast.c, so101ik/fast.py) ---------------------------------------------------- */

int fast_from(PyObject *d, so101_fast *f);     /* _path.c */

/* obj as C-contiguous float64 rows of cols (rows: -1 for any), writable if asked; 0 with an error */
static int rows_buf(PyObject *obj, Py_buffer *b, int cols, Py_ssize_t rows, int writable, const char *what) {
    if (PyObject_GetBuffer(obj, b, PyBUF_C_CONTIGUOUS | PyBUF_FORMAT | (writable ? PyBUF_WRITABLE : 0)) < 0) return 0;
    const char *f = b->format ? b->format : "B";
    if (*f == '<' || *f == '=' || *f == '@') f++;
    if (strcmp(f, "d") || b->len % (Py_ssize_t)(sizeof(double) * cols) ||
        (rows >= 0 && b->len != rows * cols * (Py_ssize_t)sizeof(double))) {
        PyErr_Format(PyExc_ValueError, "%s: a C-contiguous float64 array of %zd x %d", what, rows, cols);
        PyBuffer_Release(b);
        return 0;
    }
    return 1;
}

static PyObject *arm_play(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"q", "tip", "fix", "pos", "load", "rest", "err_max", "step", "settings", NULL};
    PyObject *o[5] = {NULL, Py_None, Py_None, Py_None, Py_None}, *settings = Py_None;
    double rest = 0.3, err_max = NAN;
    const char *step = NULL;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "O|OOOOddzO", kws, &o[0], &o[1], &o[2], &o[3], &o[4], &rest, &err_max, &step,
                                     &settings))
        return NULL;
    OPEN(self);
    so101_fast f;
    if (!fast_from(settings, &f)) return NULL;
    static const int cols[5] = {SO101_NJ, 3, SO101_NA, SO101_NJ, SO101_NJ};
    static const char *what[5] = {"q", "tip", "fix", "pos", "load"};
    Py_buffer b[5];
    int got[5] = {0}, ok = 1;
    Py_ssize_t n = -1;
    for (int i = 0; i < 5 && ok; i++) {
        if (o[i] == Py_None) continue;
        ok = got[i] = rows_buf(o[i], &b[i], cols[i], n, i >= 3, what[i]);
        if (ok && i == 0) n = b[0].len / (Py_ssize_t)(sizeof(double) * SO101_NJ);
    }
    if (ok && (ok = take(self, 0))) {
        self->busy = PyThread_get_thread_ident();
        set_step(self, step ? step : "run");
    }
    int rc = SO101_OK;
    if (ok)
        rc = so101_arm_play(&self->arm, &f, (const so101_real (*)[SO101_NJ])b[0].buf, (int)n,
                            got[1] ? (const so101_real (*)[3])b[1].buf : NULL,
                            got[2] ? (const so101_real (*)[SO101_NA])b[2].buf : NULL, rest, isnan(err_max) ? f.err_max : err_max,
                            got[3] ? (so101_real (*)[SO101_NJ])b[3].buf : NULL, got[4] ? (so101_real (*)[SO101_NJ])b[4].buf : NULL);
    for (int i = 0; i < 5; i++)
        if (got[i]) PyBuffer_Release(&b[i]);
    if (!ok) return NULL;
    return call_done(self, rc);
}

static PyObject *arm_fast(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"on", "settings", NULL};
    int on;
    PyObject *settings = Py_None;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "p|O", kws, &on, &settings)) return NULL;
    OPEN(self);
    so101_fast f;
    if (!fast_from(settings, &f)) return NULL;
    TAKE(self);
    return call_done(self, so101_arm_fast(&self->arm, &f, on));
}
