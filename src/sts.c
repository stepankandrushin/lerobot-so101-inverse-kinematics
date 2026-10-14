/* Feetech STS packets: 0xFF 0xFF id length instruction parameters... checksum, the length counting the
 * instruction (or error) byte, the parameters and the checksum, the checksum the inverted low byte of
 * the sum of everything after the header. A servo answers a ping, read or write with a status packet,
 * a sync read with one status packet each, in the order asked; a sync write gets no answer. */
#include <string.h>
#include "sts.h"

#define PING 0x01
#define READ 0x02
#define WRITE 0x03
#define SYNC_READ 0x82
#define SYNC_WRITE 0x83
#define BROADCAST 0xFE
#define TIMEOUT_US 20000    /* for the first byte of an answer, and between packets */

static int send(so101_hal *h, uint8_t id, uint8_t inst, const uint8_t *params, int n) {
    uint8_t buf[260];
    if (n > 250) return SO101_EIO;
    buf[0] = buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = (uint8_t)(n + 2);
    buf[4] = inst;
    unsigned sum = id + buf[3] + inst;
    for (int i = 0; i < n; i++) {
        buf[5 + i] = params[i];
        sum += params[i];
    }
    buf[5 + n] = (uint8_t)~sum;
    h->drain(h->ctx);
    return h->write(h->ctx, buf, n + 6) == n + 6 ? SO101_OK : SO101_EIO;
}

/* One status packet from id with n data bytes into data (may be NULL for none). */
static int receive(so101_hal *h, uint8_t id, uint8_t *data, int n) {
    uint8_t buf[260];
    int want = n + 6, have = 0;
    /* the header may follow stray bytes: read what the packet needs, then slide to 0xFF 0xFF */
    for (int tries = 0; tries < 4; tries++) {
        int got = h->read(h->ctx, buf + have, want - have, TIMEOUT_US);
        if (got <= 0) return SO101_EIO;
        have += got;
        int k = 0;
        while (k + 1 < have && !(buf[k] == 0xFF && buf[k + 1] == 0xFF && (k + 2 >= have || buf[k + 2] != 0xFF))) k++;
        if (k) {
            memmove(buf, buf + k, have - k);
            have -= k;
        }
        if (have < want) continue;
        if (buf[2] != id || buf[3] != n + 2) return SO101_EIO;
        unsigned sum = 0;
        for (int i = 2; i < want - 1; i++) sum += buf[i];
        if ((uint8_t)~sum != buf[want - 1]) return SO101_EIO;
        if (data) memcpy(data, buf + 5, n);
        return SO101_OK;      /* buf[4] is the servo's error byte: voltage, angle, heat, current, overload */
    }
    return SO101_EIO;
}

int so101_sts_ping(so101_hal *h, uint8_t id) {
    int rc = send(h, id, PING, NULL, 0);
    return rc ? rc : receive(h, id, NULL, 0);
}

int so101_sts_read(so101_hal *h, uint8_t id, uint8_t addr, uint8_t *data, int n) {
    uint8_t p[2] = {addr, (uint8_t)n};
    int rc = send(h, id, READ, p, 2);
    return rc ? rc : receive(h, id, data, n);
}

int so101_sts_write(so101_hal *h, uint8_t id, uint8_t addr, const uint8_t *data, int n) {
    uint8_t p[64];
    if (n > 63) return SO101_EIO;
    p[0] = addr;
    memcpy(p + 1, data, n);
    int rc = send(h, id, WRITE, p, n + 1);
    return rc ? rc : receive(h, id, NULL, 0);
}

int so101_sts_write1(so101_hal *h, uint8_t id, uint8_t addr, int value) {
    uint8_t d = (uint8_t)value;
    return so101_sts_write(h, id, addr, &d, 1);
}

int so101_sts_write2(so101_hal *h, uint8_t id, uint8_t addr, int value) {
    uint8_t d[2] = {(uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF)};
    return so101_sts_write(h, id, addr, d, 2);
}

int so101_sts_read2(so101_hal *h, uint8_t id, uint8_t addr, int *value) {
    uint8_t d[2];
    int rc = so101_sts_read(h, id, addr, d, 2);
    if (!rc) *value = so101_sts_u16(d);
    return rc;
}

int so101_sts_sync_write(so101_hal *h, uint8_t addr, int n, const uint8_t *ids, int count, const uint8_t *data) {
    uint8_t p[250];
    int k = 0;
    if (2 + count * (n + 1) > (int)sizeof p) return SO101_EIO;
    p[k++] = addr;
    p[k++] = (uint8_t)n;
    for (int i = 0; i < count; i++) {
        p[k++] = ids[i];
        memcpy(p + k, data + i * n, n);
        k += n;
    }
    return send(h, BROADCAST, SYNC_WRITE, p, k);
}

int so101_sts_sync_read(so101_hal *h, uint8_t addr, int n, const uint8_t *ids, int count, uint8_t *data) {
    uint8_t p[64];
    if (count + 2 > (int)sizeof p) return SO101_EIO;
    p[0] = addr;
    p[1] = (uint8_t)n;
    memcpy(p + 2, ids, count);
    int rc = send(h, BROADCAST, SYNC_READ, p, count + 2);
    for (int i = 0; !rc && i < count; i++) rc = receive(h, ids[i], data + i * n, n);
    return rc;
}
