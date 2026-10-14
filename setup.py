"""The C extension so101ik._core: the library in src/ and its binding in python/so101ik/."""
import numpy
from setuptools import Extension, setup

SOURCES = ["src/kin.c", "src/ik.c", "src/model.c", "src/sts.c", "src/arm.c", "src/hal_posix.c"]

setup(ext_modules=[Extension(
    "so101ik._core",
    sources=["python/so101ik/_core.c", "python/so101ik/_arm.c"] + SOURCES,
    include_dirs=["include", "src", numpy.get_include()],
    extra_compile_args=["-O3", "-Wall", "-Wno-unused-function", "-Wno-missing-field-initializers"],
)])
