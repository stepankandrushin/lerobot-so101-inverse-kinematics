/* so101ik._core.Arm: the follower on a serial port, the C library's so101_arm. */
#define PY_SSIZE_T_CLEAN
#include <Python.h>
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
    so101_hal io;           /* the serial port */
    int open;
    PyObject *log;       /* a callable for each event line, or None: stdout */
    FILE *csv;
    int interrupted;     /* a signal handler raised while a move ran */
} ArmObject;

static const char *NAMES[SO101_NJ] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};

/* ---- the HAL: the port's calls, and stop, log and tick for Python ------------------------------ */

static int h_write(void *c, const uint8_t *b, int n) { ArmObject *o = c; return o->io.write(o->io.ctx, b, n); }
static int h_read(void *c, uint8_t *b, int n, uint32_t t) { ArmObject *o = c; return o->io.read(o->io.ctx, b, n, t); }
static void h_drain(void *c) { ArmObject *o = c; o->io.drain(o->io.ctx); }
static uint64_t h_now(void *c) { ArmObject *o = c; return o->io.now_us(o->io.ctx); }
static void h_sleep(void *c, uint32_t us) { ArmObject *o = c; o->io.sleep_us(o->io.ctx, us); }

static int h_stop(void *c) {
    ArmObject *o = c;
    if (o->interrupted) return 1;
    if (PyErr_CheckSignals() < 0) {     /* Ctrl-C (or a handler of SIGTERM) raised: stop, keep the exception */
        o->interrupted = 1;
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

static void h_tick(void *c, const so101_tick *t) {
    ArmObject *o = c;
    if (!o->csv) return;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    fprintf(o->csv, "%.3f,%s", tv.tv_sec + tv.tv_usec / 1e6, t->step);
    for (int j = 0; j < SO101_NJ; j++)
        fprintf(o->csv, ",%.2f,%.2f,%.1f,%.0f", (double)t->goal[j], (double)t->pos[j], (double)t->load[j], (double)t->ma[j]);
    for (int i = 0; i < 3; i++) fprintf(o->csv, ",%.1f", 1000 * (double)t->tip[i]);
    for (int i = 0; i < 3; i++) fprintf(o->csv, isnan(t->target[i]) ? "," : ",%.1f", 1000 * (double)t->target[i]);
    fprintf(o->csv, "\n");
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

/* ---- the type -------------------------------------------------------------------------------- */

static int arm_init(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"port", "calibration", "log", "csv", NULL};
    const char *port, *csv = NULL;
    PyObject *cal, *log = Py_None;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "sO|Oz", kws, &port, &cal, &log, &csv)) return -1;
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
    if (so101_hal_posix_open(&self->io, port) != SO101_OK) {
        PyErr_Format(Fault, "cannot open %s at 1 Mbaud", port);
        return -1;
    }
    self->open = 1;
    so101_hal h = {h_write, h_read, h_drain, h_now, h_sleep, h_stop, h_log, h_tick, self};
    so101_arm_init(&self->arm, &h, c);
    Py_INCREF(log);
    self->log = log;
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
    if (self->open) so101_hal_posix_close(&self->io);
    self->open = 0;
    if (self->csv) fclose(self->csv);
    self->csv = NULL;
}

static void arm_dealloc(ArmObject *self) {
    arm_close_io(self);
    Py_XDECREF(self->log);
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *arm_close(ArmObject *self, PyObject *noargs) {
    arm_close_io(self);
    Py_RETURN_NONE;
}

static PyObject *arm_check(ArmObject *self, PyObject *noargs) {
    OPEN(self);
    return done(self, so101_arm_check(&self->arm));
}

static PyObject *arm_read(ArmObject *self, PyObject *noargs) {
    OPEN(self);
    so101_real pos[SO101_NJ], load[SO101_NJ], ma[SO101_NJ];
    int rc = so101_arm_read(&self->arm, pos, load, ma);
    if (rc) return done(self, rc);
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
    return done(self, so101_arm_torque_on(&self->arm));
}

static PyObject *arm_torque_off(ArmObject *self, PyObject *noargs) {
    OPEN(self);
    return done(self, so101_arm_torque_off(&self->arm));
}

static PyObject *arm_move_joints(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"target", "speed", "check", NULL};
    PyObject *t;
    double speed = 30;
    int check = 1;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "O|dp", kws, &t, &speed, &check)) return NULL;
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
    snprintf(self->arm.step, sizeof self->arm.step, "joints");
    self->arm.have_target = self->arm.have_pose = 0;
    return done(self, so101_arm_move_joints(&self->arm, deg, speed, check));
}

static PyObject *pose_or_raise(ArmObject *self, int rc) {
    if (rc) return done(self, rc);
    return new_pose(&self->arm.pose);
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
    return pose_or_raise(self, so101_arm_move_to(&self->arm, xyz, pitch != Py_None ? &ph : NULL, speed));
}

static PyObject *arm_line_to(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"x", "y", "z", "speed", NULL};
    double x, y, z, speed = 0.03;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "ddd|d", kws, &x, &y, &z, &speed)) return NULL;
    OPEN(self);
    so101_real xyz[3] = {x, y, z};
    return pose_or_raise(self, so101_arm_line_to(&self->arm, xyz, speed));
}

static PyObject *arm_move_by(ArmObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"dx", "dy", "dz", "speed", NULL};
    double x, y, z, speed = 0.03;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "ddd|d", kws, &x, &y, &z, &speed)) return NULL;
    OPEN(self);
    so101_real d[3] = {x, y, z};
    return pose_or_raise(self, so101_arm_move_by(&self->arm, d, speed));
}

static PyObject *arm_unfold(ArmObject *self, PyObject *noargs) {
    OPEN(self);
    return done(self, so101_arm_unfold(&self->arm));
}

static PyObject *arm_home(ArmObject *self, PyObject *noargs) {
    OPEN(self);
    return done(self, so101_arm_home(&self->arm));
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
static PyObject *get_offsets(ArmObject *o, void *c) {
    PyObject *t = PyTuple_New(SO101_NA);
    for (int j = 0; j < SO101_NA; j++) PyTuple_SET_ITEM(t, j, PyFloat_FromDouble(o->arm.model.offsets[j]));
    return t;
}
static int set_offsets(ArmObject *o, PyObject *v, void *c) {
    so101_real deg[SO101_NJ];
    if (!v || !read_deg(v, deg)) return -1;
    for (int j = 0; j < SO101_NA; j++) o->arm.model.offsets[j] = deg[j];
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
    {"pose", (getter)get_pose, NULL, "the IK pose last reached (move_to, line_to, move_by), or None", NULL},
    {"torque", (getter)get_torque, NULL, "whether the torque is on", NULL},
    {"folded", (getter)get_folded, NULL, "whether the arm stood folded at torque-on", NULL},
    {"start_pose", (getter)get_start, NULL, "the start pose: folded, where home() goes", NULL},
    {"sag", (getter)get_sag, (setter)set_sag, "degrees per N m the goals lead by (0: none)", NULL},
    {"lag_max", (getter)get_lag_max, (setter)set_lag_max, "degrees behind the goal that count as stalled", NULL},
    {"load_max", (getter)get_load_max, (setter)set_load_max, "% of the servo's torque that counts as overload", NULL},
    {"tick", (getter)get_tick, (setter)set_tick, "seconds between goals", NULL},
    {"offsets", (getter)get_offsets, (setter)set_offsets, "degrees, pan to roll: URDF angle = calibrated + offset", NULL},
    {NULL}};

static PyMethodDef arm_methods[] = {
    {"check", (PyCFunction)arm_check, METH_NOARGS, "every servo answers and its EEPROM matches the calibration"},
    {"read", (PyCFunction)arm_read, METH_NOARGS, "read() -> (degrees, load %, mA), dicts by joint"},
    {"torque_on", (PyCFunction)arm_torque_on, METH_NOARGS, "goals to the present positions, RAM settings, torque on"},
    {"torque_off", (PyCFunction)arm_torque_off, METH_NOARGS, "torque off: the arm drops"},
    {"move_joints", (PyCFunction)(void (*)(void))arm_move_joints, METH_VARARGS | METH_KEYWORDS,
     "move_joints(target, speed=30, check=True): straight in joint space, peak deg/s of the joint that moves most"},
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
    {NULL}};

PyTypeObject ArmType = {
    PyVarObject_HEAD_INIT(NULL, 0).tp_name = "so101ik._core.Arm",
    .tp_basicsize = sizeof(ArmObject),
    .tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
    .tp_doc = "Arm(port, calibration, log=None, csv=None): the follower on its servo bus.",
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
