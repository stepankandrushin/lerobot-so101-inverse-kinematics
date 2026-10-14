# The plan for an ESP32

Nothing of this is built yet. The library is written for it: C99, no allocation, no file system,
everything that touches the board behind so101_hal, single precision with -DSO101_FLOAT and the 180 KB
point cache left out with -DSO101_SMALL (`make test` runs both and checks them against double).

## Which ESP32

- An ESP32-S3 (two Xtensa LX7 cores at 240 MHz, a single-precision FPU, 512 KB of SRAM, often 8 MB of
  PSRAM, Wi-Fi and Bluetooth LE) or the plain ESP32 (LX6, the same FPU, Wi-Fi and Bluetooth). Not
  the RISC-V ones without an FPU (ESP32-C3, C6): every float operation would be a library call.
- ESP-IDF builds C as it is and C++ beside it; Arduino-ESP32 is C++ and includes the C header
  (so101.h has `extern "C"`).

## What it costs there

- Flash: src/model.c's constants, 180 KB of mesh points and 100 KB of tree nodes, sit in flash as
  read-only data.
- RAM with SO101_FLOAT and SO101_SMALL: the IK's samples 2 x 512 x 36 bytes (37 KB), a line's poses
  513 x 9 floats (18 KB), so101_arm itself under 1 KB; the stack of so101_ik under 4 KB.
- Time: on an Apple M1, so101_ik takes 17-35 microseconds, so101_ik_near 5-10, so101_ik_local 1-3 (single
  precision about the same). An ESP32-S3 is perhaps 20 to 40 times slower on such code, so about
  0.5-1.5 ms for so101_ik and 0.2-0.4 ms for so101_ik_near: well inside a 20 ms tick. To be measured with
  test/test.c, which needs only printf and a clock.
- A fast run (agents/design.md, Fast runs): its ticks are 6 floats every 10 ms (the 5.5 s of a 30 cm
  heart's three laps, 13 KB), its path 2 mm points of 3 floats with their 6 joints (480 points, 17 KB)
  and so101_path_fastest's work 10 floats a point (19 KB): PSRAM if the SRAM is short. The pitch search
  takes 20-60 ms on an M1, so perhaps 1-2 s on an S3, once before the run; the replay itself is one
  sync write and one sync read a tick, 100 Hz.

## The port

1. A component: `components/so101/CMakeLists.txt` with `idf_component_register(SRCS src/kin.c src/ik.c
   src/path.c src/model.c src/sts.c src/arm.c src/fast.c INCLUDE_DIRS include PRIV_INCLUDE_DIRS src)` and the definitions
   SO101_FLOAT, SO101_SMALL and SO101_NO_POSIX (src/hal_posix.c compiles to nothing then), plus
   `-fsingle-precision-constant` so double literals do not pull in double arithmetic.
2. so101_hal on the ESP32: write `uart_write_bytes`, read `uart_read_bytes` with the timeout in ticks,
   drain `uart_flush_input`, now_us `esp_timer_get_time`, sleep_us `vTaskDelay` (or
   `esp_rom_delay_us` under a tick), stop a flag that a button's interrupt or a network "stop" sets,
   log `ESP_LOGI`.
3. The bus: the servos take 1 Mbaud half duplex on one wire. To check on the follower's driver board
   (a USB serial chip on its USB side, a WCH CH343 on the board this was developed with): whether it
   has a UART header with the half-duplex buffer behind it, so that an ESP32's TX and RX go there
   instead of the CH343, and at what logic level. Otherwise a 74HC126-style buffer or a board made for
   it (an ESP32 servo driver board for STS servos) between the ESP32's UART and the bus. The servos
   run on the arm's 12 V supply, the ESP32 on its own 3.3 V.
4. test/test.c on the board, for the times and to see float agree with a desktop's double run.

## What it is for: coordinates over Wi-Fi, Bluetooth or buttons

- Commands, one a line or a packet: `move_to x y z [pitch]` (so101_arm_move_to), `move_by dx dy dz`
  (so101_arm_move_by), `line_to x y z`, `home`, `stop`, `state` (the tip, the joints, the last error).
  Millimetres on the wire, metres inside.
- Wi-Fi: UDP or a WebSocket on the ESP32; Bluetooth: a GATT characteristic to write commands to
  (the Nordic UART service, which phone apps speak); buttons: each press a move_by of a few
  millimetres along one axis, held down a stream of them.
- A held button or a stream from a phone wants a move that goes on as long as commands come: a
  so101_arm_stream() that takes the next target every tick through so101_ik_near and stops when the
  stream goes quiet for a tick or two. Not in the library yet; so101_arm_line_to plans a whole line
  first.
- The same stops as on a desktop (stall, overload), and a hardware stop button wired to hal.stop.
- The calibration: SO101_DEFAULT_CAL is compiled in from a LeRobot calibration by tools/gen_model.py; a
  recalibrated arm needs `make model` and a new build, or the numbers kept in the ESP32's NVS and
  sent over the link.
