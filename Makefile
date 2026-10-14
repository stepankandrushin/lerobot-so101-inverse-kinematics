# The library without Python: make (build/libso101ik.a), make test (double, float and SO101_SMALL builds, and the
# arm's moves on simulated servos in double and float),
# make example (examples/ik.c).
# The Python extension is built by uv sync (setup.py).
CC ?= cc
CFLAGS ?= -O3 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -std=gnu99
SRC = src/kin.c src/ik.c src/path.c src/model.c src/sts.c src/arm.c src/fast.c src/sim.c src/hal_posix.c
HDR = include/so101ik.h src/so101ik_internal.h src/sts.h

build/libso101ik.a: $(SRC) $(HDR)
	@mkdir -p build/obj
	for f in $(SRC); do $(CC) $(CFLAGS) -Iinclude -Isrc -c $$f -o build/obj/$$(basename $$f .c).o || exit 1; done
	ar rcs $@ build/obj/*.o

build/test: test/test.c $(SRC) $(HDR)
	@mkdir -p build
	$(CC) $(CFLAGS) -Iinclude -Isrc -o $@ test/test.c $(SRC) -lm

build/test-float: test/test.c $(SRC) $(HDR)
	@mkdir -p build
	$(CC) $(CFLAGS) -DSO101_FLOAT -Iinclude -Isrc -o $@ test/test.c $(SRC) -lm

build/test-small: test/test.c $(SRC) $(HDR)
	@mkdir -p build
	$(CC) $(CFLAGS) -DSO101_FLOAT -DSO101_SMALL -Iinclude -Isrc -o $@ test/test.c $(SRC) -lm

build/test-arm: test/test_arm.c $(SRC) $(HDR)
	@mkdir -p build
	$(CC) $(CFLAGS) -Iinclude -Isrc -o $@ test/test_arm.c $(SRC) -lm

build/test-arm-float: test/test_arm.c $(SRC) $(HDR)
	@mkdir -p build
	$(CC) $(CFLAGS) -DSO101_FLOAT -Iinclude -Isrc -o $@ test/test_arm.c $(SRC) -lm

test: build/test build/test-float build/test-small build/test-arm build/test-arm-float
	./build/test && ./build/test-float && ./build/test-small && ./build/test-arm && ./build/test-arm-float

build/ik: examples/ik.c build/libso101ik.a
	$(CC) $(CFLAGS) -Iinclude -o $@ examples/ik.c build/libso101ik.a -lm

example: build/ik
	./build/ik

model:
	uv run tools/gen_model.py

clean:
	rm -rf build

.PHONY: test example model clean
