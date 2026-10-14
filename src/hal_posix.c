/* so101_hal on macOS and Linux: the servo bus's USB serial port at 1 Mbaud, raw, and the monotonic clock.
 * Not compiled for a microcontroller (define SO101_NO_POSIX), which brings its own so101_hal. */
#ifndef SO101_NO_POSIX
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#endif
#include "so101ik.h"

typedef struct { int fd; } posix_t;

static uint64_t now_us(void *ctx) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000u + (uint64_t)(t.tv_nsec / 1000);
}

static void sleep_us(void *ctx, uint32_t us) {
    struct timespec t = {us / 1000000u, (long)(us % 1000000u) * 1000};
    while (nanosleep(&t, &t) && errno == EINTR) {
    }
}

static int px_write(void *ctx, const uint8_t *buf, int n) {
    posix_t *p = ctx;
    int done = 0;
    while (done < n) {
        ssize_t k = write(p->fd, buf + done, (size_t)(n - done));
        if (k < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            return -1;
        }
        done += (int)k;
    }
    return done;
}

static int px_read(void *ctx, uint8_t *buf, int n, uint32_t timeout_us) {
    posix_t *p = ctx;
    uint64_t end = now_us(ctx) + timeout_us;
    int got = 0;
    while (got < n) {
        uint64_t t = now_us(ctx);
        if (t >= end) break;
        fd_set r;
        FD_ZERO(&r);
        FD_SET(p->fd, &r);
        struct timeval tv = {(time_t)((end - t) / 1000000u), (int)((end - t) % 1000000u)};
        int s = select(p->fd + 1, &r, NULL, NULL, &tv);
        if (s < 0 && errno == EINTR) continue;
        if (s <= 0) break;
        ssize_t k = read(p->fd, buf + got, (size_t)(n - got));
        if (k < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            break;
        }
        got += (int)k;
    }
    return got;
}

static void px_drain(void *ctx) {
    posix_t *p = ctx;
    tcflush(p->fd, TCIFLUSH);
    uint8_t junk[64];
    while (read(p->fd, junk, sizeof junk) > 0) {
    }
}

int so101_hal_posix_open(so101_hal *hal, const char *port) {
    memset(hal, 0, sizeof *hal);
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return SO101_EIO;
    struct termios tio;
    if (tcgetattr(fd, &tio)) {
        close(fd);
        return SO101_EIO;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
#ifdef __APPLE__
    cfsetspeed(&tio, B115200);
    if (tcsetattr(fd, TCSANOW, &tio)) {
        close(fd);
        return SO101_EIO;
    }
    speed_t speed = 1000000;       /* not a termios constant on macOS: IOSSIOSPEED, as pyserial does */
    if (ioctl(fd, IOSSIOSPEED, &speed) < 0) {
        close(fd);
        return SO101_EIO;
    }
    unsigned long latency = 1;     /* microseconds the driver may hold received bytes */
    ioctl(fd, IOSSDATALAT, &latency);
#else
    cfsetspeed(&tio, B1000000);
    if (tcsetattr(fd, TCSANOW, &tio)) {
        close(fd);
        return SO101_EIO;
    }
#endif
    posix_t *p = malloc(sizeof *p);
    if (!p) {
        close(fd);
        return SO101_EIO;
    }
    p->fd = fd;
    hal->ctx = p;
    hal->write = px_write;
    hal->read = px_read;
    hal->drain = px_drain;
    hal->now_us = now_us;
    hal->sleep_us = sleep_us;
    px_drain(p);
    return SO101_OK;
}

void so101_hal_posix_close(so101_hal *hal) {
    posix_t *p = hal->ctx;
    if (!p) return;
    close(p->fd);
    free(p);
    hal->ctx = NULL;
}
#endif
