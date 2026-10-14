/* so101ik._core's fast runs, planned (src/path.c): functions on numpy arrays, for so101ik/fast.py. */
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>      /* this file's own numpy API table, imported in path_functions_add */
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "so101ik.h"

typedef char real_is_double[sizeof(so101_real) == sizeof(double) ? 1 : -1];   /* numpy's float64 is so101_real here */

typedef struct { PyObject_HEAD so101_model m; } ModelObject;     /* _core.c's Model */
static PyTypeObject *ModelType;
static PyObject *Fault, *Unreachable;

/* ---- settings: so101_fast as a dict ------------------------------------------------------------ */

typedef struct { const char *name; size_t off; int is_int; } setting_t;
static const setting_t SETTINGS[] = {
    {"vmax", offsetof(so101_fast, vmax), 0}, {"amax", offsetof(so101_fast, amax), 0}, {"top", offsetof(so101_fast, top), 0},
    {"dt", offsetof(so101_fast, dt), 0}, {"lag", offsetof(so101_fast, lag), 0}, {"dead", offsetof(so101_fast, dead), 0},
    {"smooth", offsetof(so101_fast, smooth), 0}, {"gap", offsetof(so101_fast, gap), 0},
    {"err_max", offsetof(so101_fast, err_max), 0}, {"tube", offsetof(so101_fast, tube), 0},
    {"torque_limit", offsetof(so101_fast, torque_limit), 1}, {"acceleration", offsetof(so101_fast, acceleration), 1},
};
#define NSET (sizeof SETTINGS / sizeof SETTINGS[0])

/* so101_fast from a dict of settings (the defaults where one is missing; None: all of them) */
int fast_from(PyObject *d, so101_fast *f) {
    so101_fast_init(f);
    if (!d || d == Py_None) return 1;
    if (!PyDict_Check(d)) {
        PyErr_SetString(PyExc_TypeError, "settings: a dict, as fast_defaults() gives");
        return 0;
    }
    PyObject *key, *value;
    Py_ssize_t pos = 0;
    while (PyDict_Next(d, &pos, &key, &value)) {
        const char *name = PyUnicode_AsUTF8(key);
        if (!name) return 0;
        size_t i = 0;
        while (i < NSET && strcmp(SETTINGS[i].name, name)) i++;
        if (i == NSET) {
            PyErr_Format(PyExc_KeyError, "no setting %s", name);
            return 0;
        }
        char *p = (char *)f + SETTINGS[i].off;
        if (SETTINGS[i].is_int) *(int *)p = (int)PyLong_AsLong(value);
        else *(so101_real *)p = PyFloat_AsDouble(value);
        if (PyErr_Occurred()) return 0;
    }
    return 1;
}

static PyObject *py_fast_defaults(PyObject *mod, PyObject *noargs) {
    so101_fast f;
    so101_fast_init(&f);
    PyObject *d = PyDict_New();
    for (size_t i = 0; i < NSET; i++) {
        char *p = (char *)&f + SETTINGS[i].off;
        PyObject *v = SETTINGS[i].is_int ? PyLong_FromLong(*(int *)p) : PyFloat_FromDouble(*(so101_real *)p);
        PyDict_SetItemString(d, SETTINGS[i].name, v);
        Py_DECREF(v);
    }
    return d;
}

/* ---- arrays ------------------------------------------------------------------------------------ */

/* a C-contiguous float64 array of rows x cols (cols 0: one dimension); NULL with an error */
static PyArrayObject *rows_of(PyObject *obj, int cols, const char *what) {
    PyArrayObject *a = (PyArrayObject *)PyArray_FROMANY(obj, NPY_DOUBLE, cols ? 2 : 1, cols ? 2 : 1, NPY_ARRAY_IN_ARRAY);
    if (!a) return NULL;
    if (cols && PyArray_DIM(a, 1) != cols) {
        PyErr_Format(PyExc_ValueError, "%s: N x %d", what, cols);
        Py_DECREF(a);
        return NULL;
    }
    return a;
}

static PyArrayObject *new_rows(npy_intp n, int cols) {
    npy_intp dims[2] = {n, cols};
    return (PyArrayObject *)(cols ? PyArray_ZEROS(2, dims, NPY_DOUBLE, 0) : PyArray_ZEROS(1, dims, NPY_DOUBLE, 0));
}

static so101_model *model_of(PyObject *o) {
    if (!PyObject_TypeCheck(o, ModelType)) {
        PyErr_SetString(PyExc_TypeError, "model: a so101ik.Model");
        return NULL;
    }
    return &((ModelObject *)o)->m;
}

static int six(PyObject *obj, so101_real v[SO101_NJ], const char *what) {
    PyArrayObject *a = rows_of(obj, 0, what);
    if (!a) return 0;
    if (PyArray_DIM(a, 0) != SO101_NJ) {
        PyErr_Format(PyExc_ValueError, "%s: six joint angles", what);
        Py_DECREF(a);
        return 0;
    }
    memcpy(v, PyArray_DATA(a), sizeof(so101_real) * SO101_NJ);
    Py_DECREF(a);
    return 1;
}

/* ---- the functions ----------------------------------------------------------------------------- */

static PyObject *py_path_poses(PyObject *mod, PyObject *args, PyObject *kw) {
    static char *kws[] = {"model", "xyz", "pitch", "roll", "gripper", "branch", "gap", NULL};
    PyObject *mo, *xo, *po = Py_None;
    double roll = 0, grip = 0, gap = 0.02;
    int branch = 1;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "OO|Oddid", kws, &mo, &xo, &po, &roll, &grip, &branch, &gap)) return NULL;
    so101_model *m = model_of(mo);
    if (!m) return NULL;
    PyArrayObject *x = rows_of(xo, 3, "xyz"), *p = NULL;
    if (!x) return NULL;
    npy_intp n = PyArray_DIM(x, 0);
    if (po != Py_None && (!(p = rows_of(po, 0, "pitch")) || PyArray_DIM(p, 0) != n)) {
        if (p) PyErr_SetString(PyExc_ValueError, "pitch: one for each point");
        Py_XDECREF(p);
        Py_DECREF(x);
        return NULL;
    }
    PyArrayObject *q = new_rows(n, SO101_NJ);
    int at = 0, rc = so101_path_poses(m, (const so101_real (*)[3])PyArray_DATA(x), (int)n, p ? PyArray_DATA(p) : NULL, roll,
                                      grip, branch, gap, (so101_real (*)[SO101_NJ])PyArray_DATA(q), &at);
    if (rc) {
        const double *c = (const double *)PyArray_DATA(x) + 3 * at;
        const char *why = rc == SO101_ECOLLIDES ? "two parts closer than the gap"
                          : rc == SO101_EPATH  ? (at == n - 1 ? "the pose jumps, or the lap does not come back to its first pose"
                                                               : "the pose jumps more than 3 degrees")
                                               : "no allowed pose";
        char b[200];     /* PyErr_Format has no %f */
        snprintf(b, sizeof b, "point %d of %d at %.1f %.1f %.1f cm: %s", at, (int)n, 100 * c[0], 100 * c[1], 100 * c[2], why);
        PyErr_SetString(Unreachable, b);
        Py_DECREF(q);
        q = NULL;
    }
    Py_XDECREF(p);
    Py_DECREF(x);
    return (PyObject *)q;
}

static PyObject *py_path_time(PyObject *mod, PyObject *args, PyObject *kw) {
    static char *kws[] = {"q", "laps", "settings", NULL};
    PyObject *qo, *so = Py_None;
    int laps = 1;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "O|iO", kws, &qo, &laps, &so)) return NULL;
    so101_fast f;
    if (!fast_from(so, &f)) return NULL;
    PyArrayObject *q = rows_of(qo, SO101_NJ, "q");
    if (!q) return NULL;
    npy_intp n = PyArray_DIM(q, 0);
    if (n < 2 || laps < 1) {
        Py_DECREF(q);
        PyErr_SetString(PyExc_ValueError, "a path of two points or more, one lap or more");
        return NULL;
    }
    PyArrayObject *t = new_rows(laps > 1 ? (n - 1) * laps + 1 : n, 0);
    so101_path_time((const so101_real (*)[SO101_NJ])PyArray_DATA(q), (int)n, laps, &f, PyArray_DATA(t));
    Py_DECREF(q);
    return (PyObject *)t;
}

static PyObject *py_path_sample(PyObject *mod, PyObject *args, PyObject *kw) {
    static char *kws[] = {"q", "t", "laps", "speed", "settings", NULL};
    PyObject *qo, *to, *so = Py_None;
    int laps = 1;
    double speed = 1;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "OO|idO", kws, &qo, &to, &laps, &speed, &so)) return NULL;
    so101_fast f;
    if (!fast_from(so, &f)) return NULL;
    if (speed <= 0) {
        PyErr_SetString(PyExc_ValueError, "speed: over 0");
        return NULL;
    }
    PyArrayObject *q = rows_of(qo, SO101_NJ, "q"), *t = q ? rows_of(to, 0, "t") : NULL;
    if (!t) {
        Py_XDECREF(q);
        return NULL;
    }
    npy_intp n = PyArray_DIM(q, 0), N = laps > 1 ? (n - 1) * laps + 1 : n;
    if (n < 2 || PyArray_DIM(t, 0) != N) {
        PyErr_SetString(PyExc_ValueError, "t: path_time's of the same q and laps");
        Py_DECREF(q);
        Py_DECREF(t);
        return NULL;
    }
    const double *tt = PyArray_DATA(t);
    npy_intp cap = (npy_intp)((tt[N - 1] / speed + 2 * f.smooth) / f.dt) + 3;
    PyArrayObject *out = new_rows(cap, SO101_NJ), *at = new_rows(cap, 0);
    int k = so101_path_sample((const so101_real (*)[SO101_NJ])PyArray_DATA(q), (int)n, laps, tt, speed, &f,
                              (so101_real (*)[SO101_NJ])PyArray_DATA(out), PyArray_DATA(at), (int)cap);
    Py_DECREF(q);
    Py_DECREF(t);
    if (k < 0) {
        Py_DECREF(out);
        Py_DECREF(at);
        PyErr_SetString(PyExc_RuntimeError, "path_sample: more ticks than it made room for");
        return NULL;
    }
    PyObject *a = PySequence_GetSlice((PyObject *)out, 0, k), *b = PySequence_GetSlice((PyObject *)at, 0, k);
    Py_DECREF(out);
    Py_DECREF(at);
    return Py_BuildValue("(NN)", a, b);
}

static PyObject *py_path_fastest(PyObject *mod, PyObject *args, PyObject *kw) {
    static char *kws[] = {"model", "xyz", "roll", "gripper", "settings", NULL};
    PyObject *mo, *xo, *so = Py_None;
    double roll = 0, grip = 0;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "OO|ddO", kws, &mo, &xo, &roll, &grip, &so)) return NULL;
    so101_model *m = model_of(mo);
    so101_fast f;
    if (!m || !fast_from(so, &f)) return NULL;
    PyArrayObject *x = rows_of(xo, 3, "xyz");
    if (!x) return NULL;
    npy_intp n = PyArray_DIM(x, 0);
    PyArrayObject *p = new_rows(n, 0), *q = new_rows(n, SO101_NJ);
    so101_real *work = PyMem_Malloc(sizeof(so101_real) * SO101_FASTEST_WORK(n)), lap = 0;
    int rc = work ? so101_path_fastest(m, (const so101_real (*)[3])PyArray_DATA(x), (int)n, roll, grip, &f, PyArray_DATA(p),
                                       (so101_real (*)[SO101_NJ])PyArray_DATA(q), &lap, work)
                  : SO101_EPATH;
    PyMem_Free(work);
    Py_DECREF(x);
    if (rc) {
        Py_DECREF(p);
        Py_DECREF(q);
        if (!work) return PyErr_NoMemory();
        PyErr_SetString(Unreachable, "no pitch, constant or smooth along the lap, gets round the path on either elbow branch "
                                     "with every part the gap apart");
        return NULL;
    }
    return Py_BuildValue("(NNd)", p, q, lap);
}

static PyObject *py_run_check(PyObject *mod, PyObject *args, PyObject *kw) {
    static char *kws[] = {"model", "q", "settings", NULL};
    PyObject *mo, *qo, *so = Py_None;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "OO|O", kws, &mo, &qo, &so)) return NULL;
    so101_model *m = model_of(mo);
    so101_fast f;
    if (!m || !fast_from(so, &f)) return NULL;
    PyArrayObject *q = rows_of(qo, SO101_NJ, "q");
    if (!q) return NULL;
    char why[200];
    int bad = so101_run_check(m, (const so101_real (*)[SO101_NJ])PyArray_DATA(q), (int)PyArray_DIM(q, 0), &f, why, sizeof why);
    Py_DECREF(q);
    if (bad) {
        PyErr_SetString(Fault, why);
        return NULL;
    }
    Py_RETURN_NONE;
}

static PyObject *py_path_learn(PyObject *mod, PyObject *args, PyObject *kw) {
    static char *kws[] = {"q", "pos", "fix", "gain", "settings", NULL};
    PyObject *qo, *po, *fo = Py_None, *so = Py_None;
    double gain = 0.7;
    if (!PyArg_ParseTupleAndKeywords(args, kw, "OO|OdO", kws, &qo, &po, &fo, &gain, &so)) return NULL;
    so101_fast f;
    if (!fast_from(so, &f)) return NULL;
    PyArrayObject *q = rows_of(qo, SO101_NJ, "q"), *p = q ? rows_of(po, SO101_NJ, "pos") : NULL, *fx = NULL;
    if (!p) {
        Py_XDECREF(q);
        return NULL;
    }
    npy_intp n = PyArray_DIM(q, 0);
    PyArrayObject *out = new_rows(n, SO101_NA);
    if (fo != Py_None) {
        if (!(fx = rows_of(fo, SO101_NA, "fix")) || PyArray_DIM(fx, 0) != n) {
            if (fx) PyErr_SetString(PyExc_ValueError, "fix: one row of five for each tick");
            Py_XDECREF(fx);
            Py_DECREF(out);
            Py_DECREF(q);
            Py_DECREF(p);
            return NULL;
        }
        memcpy(PyArray_DATA(out), PyArray_DATA(fx), sizeof(so101_real) * SO101_NA * n);
        Py_DECREF(fx);
    }
    if (PyArray_DIM(p, 0) != n) {
        PyErr_SetString(PyExc_ValueError, "pos: one row for each tick of q");
        Py_DECREF(out);
        out = NULL;
    } else {
        so101_path_learn((const so101_real (*)[SO101_NJ])PyArray_DATA(q), (const so101_real (*)[SO101_NJ])PyArray_DATA(p), (int)n,
                         &f, gain, (so101_real (*)[SO101_NA])PyArray_DATA(out));
    }
    Py_DECREF(q);
    Py_DECREF(p);
    return (PyObject *)out;
}

static PyObject *py_unfold_path(PyObject *mod, PyObject *args) {
    PyObject *ao, *bo;
    so101_real a[SO101_NJ], b[SO101_NJ];
    if (!PyArg_ParseTuple(args, "OO", &ao, &bo) || !six(ao, a, "a") || !six(bo, b, "b")) return NULL;
    so101_real most = 0;
    for (int j = 0; j < SO101_NJ; j++) most = fabs(b[j] - a[j]) > most ? fabs(b[j] - a[j]) : most;
    int cap = (int)(most / 0.5) + 4;
    PyArrayObject *q = new_rows(cap, SO101_NJ);
    int n = so101_unfold_path(a, b, (so101_real (*)[SO101_NJ])PyArray_DATA(q), cap);
    PyObject *out = PySequence_GetSlice((PyObject *)q, 0, n);
    Py_DECREF(q);
    return out;
}

static PyObject *py_unfold_clear(PyObject *mod, PyObject *args, PyObject *kw) {
    static char *kws[] = {"model", "a", "b", "tube", NULL};
    PyObject *mo, *ao, *bo;
    double tube = 6;
    so101_real a[SO101_NJ], b[SO101_NJ];
    if (!PyArg_ParseTupleAndKeywords(args, kw, "OOO|d", kws, &mo, &ao, &bo, &tube)) return NULL;
    so101_model *m = model_of(mo);
    if (!m || !six(ao, a, "a") || !six(bo, b, "b")) return NULL;
    return PyBool_FromLong(so101_unfold_clear(m, a, b, tube));
}

static PyMethodDef path_methods[] = {
    {"fast_defaults", py_fast_defaults, METH_NOARGS, "fast_defaults() -> dict: the settings of a fast run (so101_fast)"},
    {"path_poses", (PyCFunction)(void (*)(void))py_path_poses, METH_VARARGS | METH_KEYWORDS,
     "path_poses(model, xyz, pitch=None, roll=0, gripper=0, branch=1, gap=0.02) -> N x 6: the joints along tip points,\n"
     "each from the one before, at pitch (one for each point) or least-loaded. Raises Unreachable."},
    {"path_time", (PyCFunction)(void (*)(void))py_path_time, METH_VARARGS | METH_KEYWORDS,
     "path_time(q, laps=1, settings=None) -> t: the fastest timing of a joint path gone round laps times"},
    {"path_sample", (PyCFunction)(void (*)(void))py_path_sample, METH_VARARGS | METH_KEYWORDS,
     "path_sample(q, t, laps=1, speed=1, settings=None) -> (ticks K x 6, place along the path K)"},
    {"path_fastest", (PyCFunction)(void (*)(void))py_path_fastest, METH_VARARGS | METH_KEYWORDS,
     "path_fastest(model, xyz, roll=0, gripper=0, settings=None) -> (pitch, q, lap s): the pitch along a closed path\n"
     "that makes its lap shortest"},
    {"run_check", (PyCFunction)(void (*)(void))py_run_check, METH_VARARGS | METH_KEYWORDS,
     "run_check(model, q, settings=None): a sampled run against the model; raises Fault"},
    {"path_learn", (PyCFunction)(void (*)(void))py_path_learn, METH_VARARGS | METH_KEYWORDS,
     "path_learn(q, pos, fix=None, gain=0.7, settings=None) -> fix: the corrections for the next replay"},
    {"unfold_path", py_unfold_path, METH_VARARGS, "unfold_path(a, b) -> N x 6: from the folded pose a to b, the wrist first"},
    {"unfold_clear", (PyCFunction)(void (*)(void))py_unfold_clear, METH_VARARGS | METH_KEYWORDS,
     "unfold_clear(model, a, b, tube=6) -> bool: nothing touches along unfold_path, joints tube degrees off"},
    {NULL}};

int path_functions_add(PyObject *m, PyTypeObject *model_type, PyObject *fault, PyObject *unreachable) {
    import_array1(-1);
    ModelType = model_type;
    Fault = fault;
    Unreachable = unreachable;
    return PyModule_AddFunctions(m, path_methods);
}
