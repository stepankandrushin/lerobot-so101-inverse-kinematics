/* The Python face of the C library: Model, Pose, Arm, and LeRobot's two calls. Built by setup.py. */
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <structmember.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "so101ik.h"

static PyObject *Unreachable, *Fault;

/* ---- Pose ------------------------------------------------------------------------------------ */

typedef struct { PyObject_HEAD so101_pose p; } PoseObject;
static PyTypeObject PoseType;

static PyObject *tuple_of(const so101_real *v, int n) {
    PyObject *t = PyTuple_New(n);
    for (int i = 0; i < n; i++) PyTuple_SET_ITEM(t, i, PyFloat_FromDouble(v[i]));
    return t;
}

static PyObject *new_pose(const so101_pose *p) {
    PoseObject *o = PyObject_New(PoseObject, &PoseType);
    if (o) o->p = *p;
    return (PyObject *)o;
}

static const char *JOINT_NAMES[SO101_NJ] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};

static PyObject *pose_deg(PoseObject *o, void *c) {
    PyObject *d = PyDict_New();
    for (int j = 0; j < SO101_NJ; j++) {
        PyObject *v = PyFloat_FromDouble(o->p.deg[j]);
        PyDict_SetItemString(d, JOINT_NAMES[j], v);
        Py_DECREF(v);
    }
    return d;
}
static PyObject *pose_joints(PoseObject *o, void *c) { return tuple_of(o->p.deg, SO101_NJ); }
static PyObject *pose_tip(PoseObject *o, void *c) { return tuple_of(o->p.tip, 3); }
static PyObject *pose_pitch(PoseObject *o, void *c) { return PyFloat_FromDouble(o->p.pitch); }
static PyObject *pose_torque(PoseObject *o, void *c) { return tuple_of(o->p.torque, SO101_NA); }
static PyObject *pose_cost(PoseObject *o, void *c) { return PyFloat_FromDouble(o->p.cost); }
static PyObject *pose_score(PoseObject *o, void *c) { return PyFloat_FromDouble(o->p.score); }
static PyObject *pose_points(PoseObject *o, void *c) {
    return Py_BuildValue("{s:N,s:N,s:N,s:N}", "elbow", tuple_of(o->p.elbow, 3), "wrist", tuple_of(o->p.wrist, 3),
                         "roll", tuple_of(o->p.roll, 3), "tip", tuple_of(o->p.tip, 3));
}

static PyGetSetDef pose_getset[] = {
    {"deg", (getter)pose_deg, NULL, "calibrated degrees by joint name, gripper 0-100", NULL},
    {"joints", (getter)pose_joints, NULL, "the six angles in chain order", NULL},
    {"tip", (getter)pose_tip, NULL, "tip position, m", NULL},
    {"pitch", (getter)pose_pitch, NULL, "gripper pitch, degrees (0 forwards, -90 down)", NULL},
    {"torque", (getter)pose_torque, NULL, "N m each arm servo holds against gravity, pan to roll", NULL},
    {"cost", (getter)pose_cost, NULL, "sum of squared torques of lift, elbow, wrist flex, roll", NULL},
    {"score", (getter)pose_score, NULL, "cost and the comfort term", NULL},
    {"points", (getter)pose_points, NULL, "elbow, wrist and roll axes' points and the tip", NULL},
    {NULL}};

static PyObject *pose_repr(PoseObject *o) {
    char b[400];
    const so101_real *d = o->p.deg;
    snprintf(b, sizeof b, "Pose(pan %+.1f lift %+.1f elbow %+.1f wflex %+.1f roll %+.1f grip %.1f | tip %.1f %.1f %.1f cm, "
             "pitch %+.1f, cost %.4f)", d[0], d[1], d[2], d[3], d[4], d[5], 100 * o->p.tip[0], 100 * o->p.tip[1],
             100 * o->p.tip[2], o->p.pitch, o->p.cost);
    return PyUnicode_FromString(b);
}

static PyTypeObject PoseType = {
    PyVarObject_HEAD_INIT(NULL, 0).tp_name = "so101ik.Pose",
    .tp_basicsize = sizeof(PoseObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = "An arm pose: joint angles, tip, pitch and the gravity load (from Model.pose or Model.ik).",
    .tp_getset = pose_getset,
    .tp_repr = (reprfunc)pose_repr,
};

/* ---- helpers --------------------------------------------------------------------------------- */

/* Six calibrated degrees from a Pose, a dict by joint name (gripper optional) or a sequence of 5-6. */
static int read_deg(PyObject *obj, so101_real deg[SO101_NJ]) {
    if (PyObject_TypeCheck(obj, &PoseType)) {
        memcpy(deg, ((PoseObject *)obj)->p.deg, sizeof(so101_real) * SO101_NJ);
        return 1;
    }
    deg[SO101_GRIP] = 0;
    if (PyDict_Check(obj)) {
        for (int j = 0; j < SO101_NJ; j++) {
            PyObject *v = PyDict_GetItemString(obj, JOINT_NAMES[j]);
            if (!v) {
                if (j == SO101_GRIP) break;
                PyErr_Format(PyExc_KeyError, "%s", JOINT_NAMES[j]);
                return 0;
            }
            deg[j] = PyFloat_AsDouble(v);
        }
        return !PyErr_Occurred();
    }
    PyObject *seq = PySequence_Fast(obj, "joint angles: a Pose, a dict or a sequence of 5 or 6 numbers");
    if (!seq) return 0;
    Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
    if (n < SO101_NA) {
        Py_DECREF(seq);
        PyErr_SetString(PyExc_ValueError, "need 5 or 6 joint angles");
        return 0;
    }
    for (int j = 0; j < SO101_NJ && j < n; j++) deg[j] = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, j));
    Py_DECREF(seq);
    return !PyErr_Occurred();
}

static int read3(PyObject *obj, so101_real v[3]) {
    PyObject *seq = PySequence_Fast(obj, "a point: three numbers");
    if (!seq) return 0;
    if (PySequence_Fast_GET_SIZE(seq) != 3) {
        Py_DECREF(seq);
        PyErr_SetString(PyExc_ValueError, "a point is three numbers");
        return 0;
    }
    for (int i = 0; i < 3; i++) v[i] = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, i));
    Py_DECREF(seq);
    return !PyErr_Occurred();
}

static PyObject *problems_list(unsigned bad) {
    PyObject *l = PyList_New(0);
    for (unsigned b = 1; b <= SO101_P_TIP_BASE; b <<= 1)
        if (bad & b) {
            PyObject *s = PyUnicode_FromString(so101_problem_name(b));
            PyList_Append(l, s);
            Py_DECREF(s);
        }
    return l;
}

/* ---- Model ----------------------------------------------------------------------------------- */

typedef struct { PyObject_HEAD so101_model m; } ModelObject;
static PyTypeObject ModelType;

static int model_init(ModelObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"stock", NULL};
    int stock = 0;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "|p", kws, &stock)) return -1;
    if (stock) so101_model_stock(&self->m);
    else so101_model_init(&self->m);
    return 0;
}

static PyObject *model_pose(ModelObject *self, PyObject *arg) {
    so101_real deg[SO101_NJ];
    if (!read_deg(arg, deg)) return NULL;
    so101_pose p;
    so101_pose_eval(&self->m, deg, &p);
    return new_pose(&p);
}

static PyObject *model_tip(ModelObject *self, PyObject *arg) {
    so101_real deg[SO101_NJ], t[3], pitch;
    if (!read_deg(arg, deg)) return NULL;
    so101_tip(&self->m, deg, t, &pitch);
    return Py_BuildValue("(dddd)", t[0], t[1], t[2], pitch);
}

static PyObject *unreachable(int rc, const so101_real xyz[3]) {
    char b[160];
    snprintf(b, sizeof b, "%s: %.1f %.1f %.1f cm", so101_strerror(rc), 100 * xyz[0], 100 * xyz[1], 100 * xyz[2]);
    PyErr_SetString(Unreachable, b);
    return NULL;
}

static PyObject *model_ik(ModelObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"x", "y", "z", "roll", "gripper", "pitch", "near", NULL};
    double x, y, z, roll = 0, grip = 0;
    PyObject *pitch = Py_None, *near = Py_None;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "ddd|ddOO", kws, &x, &y, &z, &roll, &grip, &pitch, &near)) return NULL;
    so101_real xyz[3] = {x, y, z};
    so101_pose out;
    const so101_pose *np = NULL;
    if (near != Py_None) {
        if (!PyObject_TypeCheck(near, &PoseType)) {
            PyErr_SetString(PyExc_TypeError, "near: a Pose");
            return NULL;
        }
        np = &((PoseObject *)near)->p;
    }
    int rc;
    if (pitch != Py_None) {
        double ph = PyFloat_AsDouble(pitch);
        if (PyErr_Occurred()) return NULL;
        rc = so101_ik_pitch(&self->m, xyz, ph, roll, grip, np, &out);
    } else if (np) {
        rc = so101_ik_near(&self->m, xyz, np, &out);
    } else {
        rc = so101_ik(&self->m, xyz, roll, grip, &out);
    }
    if (rc != SO101_OK) return unreachable(rc, xyz);
    return new_pose(&out);
}

static PyObject *model_problems(ModelObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"pose", "margin", NULL};
    PyObject *obj;
    double margin = NAN;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "O|d", kws, &obj, &margin)) return NULL;
    so101_real deg[SO101_NJ];
    if (!read_deg(obj, deg)) return NULL;
    so101_pose p;
    so101_pose_eval(&self->m, deg, &p);
    return problems_list(so101_problems(&self->m, &p, isnan(margin) ? self->m.margin : margin));
}

static PyObject *model_collides(ModelObject *self, PyObject *args, PyObject *kw) {
    static char *kws[] = {"pose", "gap", NULL};
    PyObject *obj;
    double gap = NAN;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "O|d", kws, &obj, &gap)) return NULL;
    so101_real deg[SO101_NJ];
    if (!read_deg(obj, deg)) return NULL;
    int c = so101_collides(&self->m, deg, isnan(gap) ? self->m.gap : gap);
    if (!c) Py_RETURN_NONE;
    return PyUnicode_FromString(so101_collision_name(c));
}

static PyObject *model_set_ranges(ModelObject *self, PyObject *arg) {
    /* a list of five (lo, hi) in calibrated degrees */
    PyObject *seq = PySequence_Fast(arg, "ranges: five (lo, hi) pairs");
    if (!seq) return NULL;
    if (PySequence_Fast_GET_SIZE(seq) != SO101_NA) {
        Py_DECREF(seq);
        PyErr_SetString(PyExc_ValueError, "ranges: five (lo, hi) pairs, pan to roll");
        return NULL;
    }
    for (int j = 0; j < SO101_NA; j++) {
        double lo, hi;
        if (!PyArg_ParseTuple(PySequence_Fast_GET_ITEM(seq, j), "dd", &lo, &hi)) {
            Py_DECREF(seq);
            return NULL;
        }
        self->m.lo[j] = lo;
        self->m.hi[j] = hi;
    }
    Py_DECREF(seq);
    self->m.pl.valid = 0;
    Py_RETURN_NONE;
}

static PyObject *model_get_ranges(ModelObject *self, void *c) {
    PyObject *l = PyList_New(SO101_NA);
    for (int j = 0; j < SO101_NA; j++) PyList_SET_ITEM(l, j, Py_BuildValue("(dd)", self->m.lo[j], self->m.hi[j]));
    return l;
}

static PyObject *model_get_offsets(ModelObject *self, void *c) { return tuple_of(self->m.offsets, SO101_NA); }
static int model_set_offsets(ModelObject *self, PyObject *v, void *c) {
    so101_real deg[SO101_NJ];
    if (!v || !read_deg(v, deg)) return -1;
    for (int j = 0; j < SO101_NA; j++) self->m.offsets[j] = deg[j];
    self->m.pl.valid = 0;
    return 0;
}

/* the scalar settings, by their offset in so101_model */
typedef struct { const char *name; size_t off; int is_int; const char *doc; } field_t;
static const field_t FIELDS[] = {
    {"payload", offsetof(so101_model, payload), 0, "kg between the jaws"},
    {"table_z", offsetof(so101_model, table_z), 0, "the table's height, m"},
    {"margin", offsetof(so101_model, margin), 0, "degrees kept inside each range"},
    {"clearance", offsetof(so101_model, clearance), 0, "axes above the table, m"},
    {"tip_clearance", offsetof(so101_model, tip_clearance), 0, "the tip above the table, m"},
    {"gap", offsetof(so101_model, gap), 0, "least distance between parts and to the table, m"},
    {"comfort", offsetof(so101_model, comfort), 0, "the tie-breaking term at a range end, N^2 m^2"},
    {"smooth", offsetof(so101_model, smooth), 0, "along a path, N^2 m^2 per degree^2 of change"},
    {"pitch_step", offsetof(so101_model, pitch_step), 0, "degrees between the pitches the search starts from"},
    {"near_window", offsetof(so101_model, near_window), 0, "along a path, degrees of pitch either way"},
    {"base_rule", offsetof(so101_model, base_rule), 1, "the roll axis and tip kept out of the base"},
    {"collisions", offsetof(so101_model, collisions), 1, "the IK checks the meshes"},
};

static PyObject *field_get(ModelObject *self, void *c) {
    const field_t *f = c;
    char *p = (char *)&self->m + f->off;
    if (f->is_int) return PyBool_FromLong(*(int *)p);
    return PyFloat_FromDouble(*(so101_real *)p);
}
static int field_set(ModelObject *self, PyObject *v, void *c) {
    const field_t *f = c;
    char *p = (char *)&self->m + f->off;
    if (!v) return -1;
    if (f->is_int) *(int *)p = PyObject_IsTrue(v);
    else *(so101_real *)p = PyFloat_AsDouble(v);
    self->m.pl.valid = 0;
    return PyErr_Occurred() ? -1 : 0;
}

static PyMethodDef model_methods[] = {
    {"pose", (PyCFunction)model_pose, METH_O, "pose(deg) -> Pose: forward kinematics and gravity torques"},
    {"tip", (PyCFunction)model_tip, METH_O, "tip(deg) -> (x, y, z, pitch)"},
    {"ik", (PyCFunction)(void (*)(void))model_ik, METH_VARARGS | METH_KEYWORDS,
     "ik(x, y, z, roll=0, gripper=0, pitch=None, near=None) -> Pose: the least-loaded allowed pose; at pitch;\n"
     "or along a path from near. Raises Unreachable."},
    {"problems", (PyCFunction)(void (*)(void))model_problems, METH_VARARGS | METH_KEYWORDS,
     "problems(pose, margin=None) -> [str]: why a pose is not allowed"},
    {"collides", (PyCFunction)(void (*)(void))model_collides, METH_VARARGS | METH_KEYWORDS,
     "collides(pose, gap=None) -> str or None: two parts, or a part and the table, closer than gap"},
    {"set_ranges", (PyCFunction)model_set_ranges, METH_O, "set_ranges([(lo, hi)] * 5): calibrated ranges, degrees"},
    {NULL}};

static PyGetSetDef model_getset[16];

static PyTypeObject ModelType = {
    PyVarObject_HEAD_INIT(NULL, 0).tp_name = "so101ik.Model",
    .tp_basicsize = sizeof(ModelObject),
    .tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
    .tp_doc = "Model(stock=False): the SO-101 follower's kinematics. stock: LeRobot's view (no offsets, the URDF's\n"
              "limits, no table or meshes).",
    .tp_new = PyType_GenericNew,
    .tp_init = (initproc)model_init,
    .tp_methods = model_methods,
    .tp_getset = model_getset,
};

/* ---- LeRobot's two calls --------------------------------------------------------------------- */

static PyObject *py_fk_urdf(PyObject *mod, PyObject *arg) {
    PyArrayObject *a = (PyArrayObject *)PyArray_FROMANY(arg, NPY_DOUBLE, 1, 1, NPY_ARRAY_IN_ARRAY);
    if (!a) return NULL;
    if (PyArray_DIM(a, 0) < SO101_NA) {
        Py_DECREF(a);
        PyErr_SetString(PyExc_ValueError, "need 5 joint angles");
        return NULL;
    }
    npy_intp dims[2] = {4, 4};
    PyObject *T = PyArray_SimpleNew(2, dims, NPY_DOUBLE);
    if (T) so101_fk_urdf((const double *)PyArray_DATA(a), (double *)PyArray_DATA((PyArrayObject *)T));
    Py_DECREF(a);
    return T;
}

static PyObject *py_ik_local(PyObject *mod, PyObject *const *args, Py_ssize_t n) {
    if (n != 6) {
        PyErr_SetString(PyExc_TypeError, "ik_local(q, T, wp, wo, iters, fallback)");
        return NULL;
    }
    PyArrayObject *q = (PyArrayObject *)PyArray_FROMANY(args[0], NPY_DOUBLE, 1, 1, NPY_ARRAY_IN_ARRAY);
    if (!q) return NULL;
    PyArrayObject *T = (PyArrayObject *)PyArray_FROMANY(args[1], NPY_DOUBLE, 2, 2, NPY_ARRAY_IN_ARRAY);
    if (!T) {
        Py_DECREF(q);
        return NULL;
    }
    double wp = PyFloat_AsDouble(args[2]), wo = PyFloat_AsDouble(args[3]);
    long iters = PyLong_AsLong(args[4]);
    so101_model *fb = NULL;
    if (args[5] != Py_None) {
        if (!PyObject_TypeCheck(args[5], &ModelType)) {
            PyErr_SetString(PyExc_TypeError, "fallback: a Model or None");
        } else fb = &((ModelObject *)args[5])->m;
    }
    if (PyErr_Occurred() || PyArray_DIM(q, 0) < SO101_NA || PyArray_DIM(T, 0) != 4 || PyArray_DIM(T, 1) != 4) {
        if (!PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, "q: 5 or more angles; T: 4x4");
        Py_DECREF(q);
        Py_DECREF(T);
        return NULL;
    }
    npy_intp len = PyArray_DIM(q, 0);
    PyArrayObject *out = (PyArrayObject *)PyArray_SimpleNew(1, &len, NPY_DOUBLE);
    double *o = PyArray_DATA(out);
    memcpy(o, PyArray_DATA(q), sizeof(double) * len);
    double err = 0;
    int steps = so101_ik_local(o, PyArray_DATA(T), wp, wo, (int)iters, fb, &err);
    Py_DECREF(q);
    Py_DECREF(T);
    return Py_BuildValue("(Nid)", out, steps, err);
}

/* ---- module ---------------------------------------------------------------------------------- */

static PyObject *py_default_cal(PyObject *mod, PyObject *noargs) {
    PyObject *l = PyList_New(SO101_NJ);
    for (int j = 0; j < SO101_NJ; j++)
        PyList_SET_ITEM(l, j, Py_BuildValue("(iiii)", SO101_DEFAULT_CAL[j].id, SO101_DEFAULT_CAL[j].homing_offset,
                                            SO101_DEFAULT_CAL[j].range_min, SO101_DEFAULT_CAL[j].range_max));
    return l;
}

extern PyTypeObject ArmType;
int arm_type_ready(PyObject *m, PyObject *fault, PyObject *unreachable, PyTypeObject *pose_type,
                   PyObject *(*new_pose)(const so101_pose *), int (*read_deg)(PyObject *, so101_real *),
                   int (*read3)(PyObject *, so101_real *));

static PyMethodDef module_methods[] = {
    {"fk_urdf", py_fk_urdf, METH_O, "fk_urdf(q) -> 4x4 tip frame for URDF degrees (LeRobot's forward_kinematics)"},
    {"ik_local", (PyCFunction)(void (*)(void))py_ik_local, METH_FASTCALL,
     "ik_local(q, T, wp, wo, iters, fallback) -> (q, steps, err): LeRobot's inverse_kinematics"},
    {"default_calibration", py_default_cal, METH_NOARGS, "the calibration compiled in: (id, homing_offset, min, max) per servo"},
    {NULL}};

static struct PyModuleDef moduledef = {PyModuleDef_HEAD_INIT, "_core", "so101ik's C core", -1, module_methods};

PyMODINIT_FUNC PyInit__core(void) {
    import_array();
    int k = 0;
    model_getset[k++] = (PyGetSetDef){"ranges", (getter)model_get_ranges, NULL, "calibrated ranges, degrees", NULL};
    model_getset[k++] = (PyGetSetDef){"offsets", (getter)model_get_offsets, (setter)model_set_offsets,
                                      "degrees from calibrated to URDF angles, pan to roll", NULL};
    for (size_t i = 0; i < sizeof FIELDS / sizeof FIELDS[0]; i++)
        model_getset[k++] = (PyGetSetDef){(char *)FIELDS[i].name, (getter)field_get, (setter)field_set,
                                          (char *)FIELDS[i].doc, (void *)&FIELDS[i]};
    model_getset[k] = (PyGetSetDef){NULL};
    if (PyType_Ready(&PoseType) < 0 || PyType_Ready(&ModelType) < 0) return NULL;
    PyObject *m = PyModule_Create(&moduledef);
    if (!m) return NULL;
    Unreachable = PyErr_NewException("so101ik.Unreachable", PyExc_ValueError, NULL);
    Fault = PyErr_NewException("so101ik.Fault", PyExc_RuntimeError, NULL);
    PyModule_AddObject(m, "Unreachable", Unreachable);
    PyModule_AddObject(m, "Fault", Fault);
    Py_INCREF(&PoseType);
    PyModule_AddObject(m, "Pose", (PyObject *)&PoseType);
    Py_INCREF(&ModelType);
    PyModule_AddObject(m, "Model", (PyObject *)&ModelType);
    if (arm_type_ready(m, Fault, Unreachable, &PoseType, new_pose, read_deg, read3) < 0) return NULL;
    return m;
}
