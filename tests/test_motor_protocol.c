#include "motor_protocol.h"

#include <stdio.h>
#include <string.h>

/* Golden frames. The bytes are written out here; they are not produced by
 * the encoder under test.
 *
 * CONTROL, FR, SPEED +120 RPM, seq 5, session 0x3C. Id 0x011 = 0x010 | FR.
 *   byte 0  0x11  version 1, mode SPEED (1)
 *   byte 1  0x00  no ARM, no CLEAR_FAULT
 *   bytes 2-3  0x0078 = 120 RPM, little endian
 *   bytes 4-5  0 feed-forward mA
 *   byte 6  seq 5
 *   byte 7  session 0x3C
 *
 * STATUS reply, same wheel, same 120 RPM. Id 0x281 = 0x280 | FR.
 *   bytes 0-1  120 RPM
 *   bytes 2-3  544 counts = 1 A (MOTOR_COUNTS_PER_AMP)
 *   bytes 4-5  1088 counts = 2 A reference
 *   byte 6  6, MCSDK RUN
 *   byte 7  0x90 = output on | Hall shift known, stop reason none
 *
 * DIAG reply, FR, bus 12.60 V. Id 0x381 = 0x380 | FR.
 *   bytes 0-1  1260 centivolts = 12.60 V (0x04EC)
 *   bytes 2-3  fault word 0
 *   bytes 4-5  Hall shift -30 electrical degrees (0xFFE2)
 *   byte 6  alignment not run
 *   byte 7  I2t 0 percent
 */

static const uint8_t k_control_120[MOTOR_PROTOCOL_DLC] = {
    0x11, 0x00, 0x78, 0x00, 0x00, 0x00, 0x05, 0x3C
};
static const uint8_t k_status_120[MOTOR_PROTOCOL_DLC] = {
    0x78, 0x00, 0x20, 0x02, 0x40, 0x04, 0x06, 0x90
};
static const uint8_t k_diag_1260[MOTOR_PROTOCOL_DLC] = {
    0xEC, 0x04, 0x00, 0x00, 0xE2, 0xFF, 0x00, 0x00
};

/* SPEED -120 RPM and feed-forward -1500 mA. -1500 = 0xFA24, so the bytes are
 * 24 FA. Id still 0x011. */
static const uint8_t k_control_neg[MOTOR_PROTOCOL_DLC] = {
    0x11, 0x00, 0x88, 0xFF, 0x24, 0xFA, 0x06, 0x3C
};

static const uint32_t k_wheel[4] = {
    MOTOR_SELECT_FR, MOTOR_SELECT_BR, MOTOR_SELECT_BL, MOTOR_SELECT_FL
};
static const char *k_wheel_name[4] = {"FR", "BR", "BL", "FL"};

static int g_failures;

static void fail(const char *name, const char *msg)
{
    fprintf(stderr, "FAIL %s: %s\n", name, msg);
    g_failures++;
}

static void expect_reason(const char *name, enum motor_reason got, enum motor_reason want)
{
    if (got != want) {
        fprintf(stderr, "FAIL %s: reason %d want %d\n", name, (int)got, (int)want);
        g_failures++;
    }
}

static void expect_bytes(const char *name, const uint8_t *got, const uint8_t *want, unsigned n)
{
    unsigned i;

    for (i = 0; i < n; ++i) {
        if (got[i] != want[i]) {
            fprintf(stderr, "FAIL %s byte %u: got %02X want %02X\n", name, i, got[i], want[i]);
            g_failures++;
        }
    }
}

static int same_bytes(const uint8_t *a, unsigned n, uint8_t fill)
{
    unsigned i;

    for (i = 0; i < n; ++i) {
        if (a[i] != fill) {
            return 0;
        }
    }
    return 1;
}

static void test_golden_control(void)
{
    struct motor_control in = {
        .mode = MOTOR_MODE_SPEED,
        .flags = 0,
        .setpoint = 120,
        .ff_ma = 0,
        .seq = 5,
        .session = 0x3C
    };
    struct motor_control out;
    uint8_t data[MOTOR_PROTOCOL_DLC];
    uint32_t id = 0;

    memset(&out, 0, sizeof out);
    expect_reason("golden control encode",
                  motor_control_encode(&in, MOTOR_SELECT_FR, &id, data), MOTOR_R_NONE);
    if (id != (MOTOR_CAN_ID_CONTROL | MOTOR_SELECT_FR)) {
        fprintf(stderr, "FAIL golden control id: %03X\n", id);
        g_failures++;
    }
    expect_bytes("golden control", data, k_control_120, MOTOR_PROTOCOL_DLC);

    expect_reason("golden control decode",
                  motor_control_decode(0x011u, k_control_120, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_NONE);
    if (out.mode != MOTOR_MODE_SPEED || out.flags != 0 || out.setpoint != 120 ||
        out.ff_ma != 0 || out.seq != 5 || out.session != 0x3C) {
        fail("golden control decode", "fields");
    }
}

static void test_golden_status(void)
{
    struct motor_status in = {
        .rpm = 120,
        .iq_counts = MOTOR_COUNTS_PER_AMP,
        .iq_ref_counts = (int16_t)(2 * MOTOR_COUNTS_PER_AMP),
        .mcsdk_state = MOTOR_MCSDK_STATE_RUN,
        .flags = (uint8_t)(MOTOR_STATUS_OUTPUT_ON | MOTOR_STATUS_HALL_KNOWN)
    };
    struct motor_status out;
    uint8_t data[MOTOR_PROTOCOL_DLC];
    uint32_t id = 0;

    if (k_status_120[6] != MOTOR_MCSDK_STATE_RUN) {
        fail("golden status", "RUN byte drifted from the fixture");
    }
    memset(&out, 0, sizeof out);
    expect_reason("golden status encode",
                  motor_status_encode(&in, MOTOR_SELECT_FR, &id, data), MOTOR_R_NONE);
    if (id != (MOTOR_CAN_ID_STATUS | MOTOR_SELECT_FR)) {
        fprintf(stderr, "FAIL golden status id: %03X\n", id);
        g_failures++;
    }
    expect_bytes("golden status", data, k_status_120, MOTOR_PROTOCOL_DLC);

    expect_reason("golden status decode",
                  motor_status_decode(0x281u, k_status_120, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_NONE);
    if (out.rpm != 120 || out.iq_counts != MOTOR_COUNTS_PER_AMP ||
        out.iq_ref_counts != (int16_t)(2 * MOTOR_COUNTS_PER_AMP) ||
        out.mcsdk_state != MOTOR_MCSDK_STATE_RUN ||
        out.flags != (uint8_t)(MOTOR_STATUS_OUTPUT_ON | MOTOR_STATUS_HALL_KNOWN) ||
        (out.flags & 0x0Fu) != MOTOR_STOP_NONE) {
        fail("golden status decode", "fields");
    }
}

static void test_golden_diag(void)
{
    struct motor_diag in = {
        .vbus_cv = 1260,
        .faults = 0,
        .hall_shift_deg = -30,
        .align_result = MOTOR_ALIGN_NONE,
        .i2t_percent = 0
    };
    struct motor_diag out;
    uint8_t data[MOTOR_PROTOCOL_DLC];
    uint32_t id = 0;

    memset(&out, 0, sizeof out);
    expect_reason("golden diag encode",
                  motor_diag_encode(&in, MOTOR_SELECT_FR, &id, data), MOTOR_R_NONE);
    if (id != (MOTOR_CAN_ID_DIAG | MOTOR_SELECT_FR)) {
        fprintf(stderr, "FAIL golden diag id: %03X\n", id);
        g_failures++;
    }
    expect_bytes("golden diag", data, k_diag_1260, MOTOR_PROTOCOL_DLC);

    expect_reason("golden diag decode",
                  motor_diag_decode(0x381u, k_diag_1260, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_NONE);
    if (out.vbus_cv != 1260 || out.faults != 0 || out.hall_shift_deg != -30 ||
        out.align_result != MOTOR_ALIGN_NONE || out.i2t_percent != 0) {
        fail("golden diag decode", "fields");
    }
}

static void test_signed_control_bytes(void)
{
    struct motor_control in = {
        .mode = MOTOR_MODE_SPEED,
        .flags = 0,
        .setpoint = -120,
        .ff_ma = -1500,
        .seq = 6,
        .session = 0x3C
    };
    struct motor_control out;
    uint8_t data[MOTOR_PROTOCOL_DLC];
    uint32_t id = 0;

    expect_reason("signed encode",
                  motor_control_encode(&in, MOTOR_SELECT_FR, &id, data), MOTOR_R_NONE);
    expect_bytes("signed control", data, k_control_neg, MOTOR_PROTOCOL_DLC);
    expect_reason("signed decode",
                  motor_control_decode(id, data, MOTOR_PROTOCOL_DLC, &out), MOTOR_R_NONE);
    if (out.setpoint != -120 || out.ff_ma != -1500 || out.seq != 6) {
        fail("signed control", "fields");
    }
}

static void test_round_trip_each_wheel(void)
{
    int w;

    for (w = 0; w < 4; ++w) {
        struct motor_control cmd = {
            .mode = MOTOR_MODE_SPEED,
            .flags = 0,
            .setpoint = (int16_t)(100 + w),
            .ff_ma = (int16_t)(10 * w),
            .seq = (uint8_t)(20 + w),
            .session = (uint8_t)(0x40 + w)
        };
        struct motor_control cmd_out;
        struct motor_status st = {
            .rpm = (int16_t)(-50 * (w + 1)),
            .iq_counts = (int16_t)(-MOTOR_COUNTS_PER_AMP),
            .iq_ref_counts = (int16_t)((w + 1) * MOTOR_COUNTS_PER_AMP),
            .mcsdk_state = MOTOR_MCSDK_STATE_RUN,
            .flags = (uint8_t)(MOTOR_STATUS_OUTPUT_ON | MOTOR_STOP_DISABLE)
        };
        struct motor_status st_out;
        struct motor_diag diag = {
            .vbus_cv = (uint16_t)(1200 + 10 * w),
            .faults = (uint16_t)(1u << w),
            .hall_shift_deg = (int16_t)(-15 * (w + 1)),
            .align_result = MOTOR_ALIGN_OK,
            .i2t_percent = (uint8_t)(25 * w)
        };
        struct motor_diag diag_out;
        uint8_t data[MOTOR_PROTOCOL_DLC];
        uint8_t poll[MOTOR_POLL_DLC];
        uint8_t junk[MOTOR_POLL_DLC] = {0xDE, 0xAD, 0xBE, 0xEF};
        uint32_t id = 0;
        uint32_t command = 0;
        uint32_t select = 0;
        char name[32];

        snprintf(name, sizeof name, "control %s", k_wheel_name[w]);
        expect_reason(name, motor_control_encode(&cmd, k_wheel[w], &id, data), MOTOR_R_NONE);
        expect_reason(name, motor_control_decode(id, data, MOTOR_PROTOCOL_DLC, &cmd_out),
                      MOTOR_R_NONE);
        if (id != (MOTOR_CAN_ID_CONTROL | k_wheel[w]) || cmd_out.mode != cmd.mode ||
            cmd_out.setpoint != cmd.setpoint || cmd_out.ff_ma != cmd.ff_ma ||
            cmd_out.seq != cmd.seq || cmd_out.session != cmd.session) {
            fail(name, "round trip");
        }

        snprintf(name, sizeof name, "status %s", k_wheel_name[w]);
        expect_reason(name, motor_status_encode(&st, k_wheel[w], &id, data), MOTOR_R_NONE);
        expect_reason(name, motor_status_decode(id, data, MOTOR_PROTOCOL_DLC, &st_out),
                      MOTOR_R_NONE);
        if (id != (MOTOR_CAN_ID_STATUS | k_wheel[w]) || st_out.rpm != st.rpm ||
            st_out.iq_counts != st.iq_counts || st_out.iq_ref_counts != st.iq_ref_counts ||
            st_out.mcsdk_state != st.mcsdk_state || st_out.flags != st.flags) {
            fail(name, "round trip");
        }

        snprintf(name, sizeof name, "diag %s", k_wheel_name[w]);
        expect_reason(name, motor_diag_encode(&diag, k_wheel[w], &id, data), MOTOR_R_NONE);
        expect_reason(name, motor_diag_decode(id, data, MOTOR_PROTOCOL_DLC, &diag_out),
                      MOTOR_R_NONE);
        if (id != (MOTOR_CAN_ID_DIAG | k_wheel[w]) || diag_out.vbus_cv != diag.vbus_cv ||
            diag_out.faults != diag.faults || diag_out.hall_shift_deg != diag.hall_shift_deg ||
            diag_out.align_result != diag.align_result ||
            diag_out.i2t_percent != diag.i2t_percent) {
            fail(name, "round trip");
        }

        snprintf(name, sizeof name, "poll %s", k_wheel_name[w]);
        expect_reason(name,
                      motor_poll_encode(MOTOR_CAN_ID_STATUS_READ, k_wheel[w], &id, poll),
                      MOTOR_R_NONE);
        expect_reason(name, motor_poll_decode(id, junk, MOTOR_POLL_DLC, &command, &select),
                      MOTOR_R_NONE);
        if (id != (MOTOR_CAN_ID_STATUS_READ | k_wheel[w]) ||
            command != MOTOR_CAN_ID_STATUS_READ || select != k_wheel[w] ||
            poll[0] != 0 || poll[3] != 0) {
            fail(name, "status poll");
        }
        expect_reason(name,
                      motor_poll_encode(MOTOR_CAN_ID_DIAG_READ, k_wheel[w], &id, poll),
                      MOTOR_R_NONE);
        expect_reason(name, motor_poll_decode(id, poll, MOTOR_POLL_DLC, &command, &select),
                      MOTOR_R_NONE);
        if (id != (MOTOR_CAN_ID_DIAG_READ | k_wheel[w]) ||
            command != MOTOR_CAN_ID_DIAG_READ || select != k_wheel[w]) {
            fail(name, "diag poll");
        }
    }
}

static void test_broadcast_and_arm(void)
{
    struct motor_control stop = {
        .mode = MOTOR_MODE_DISABLE, .flags = 0, .setpoint = 0, .ff_ma = 0,
        .seq = 0x2A, .session = 0
    };
    struct motor_control arm = {
        .mode = MOTOR_MODE_DISABLE, .flags = MOTOR_FLAG_ARM, .setpoint = 0, .ff_ma = 0,
        .seq = 0, .session = 0x9D
    };
    struct motor_control clear = arm;
    struct motor_control current = {
        .mode = MOTOR_MODE_CURRENT, .flags = 0, .setpoint = 2500, .ff_ma = 0,
        .seq = 7, .session = 0x3C
    };
    struct motor_control out;
    uint8_t data[MOTOR_PROTOCOL_DLC];
    uint32_t id = 0;
    uint32_t command = 0;
    uint32_t select = 0;
    uint8_t poll[MOTOR_POLL_DLC];
    /* Plain DISABLE may name every wheel. 0x10 is version 1, mode DISABLE. */
    static const uint8_t k_stop[MOTOR_PROTOCOL_DLC] = {
        0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A, 0x00
    };
    static const uint8_t k_current[MOTOR_PROTOCOL_DLC] = {
        0x12, 0x00, 0xC4, 0x09, 0x00, 0x00, 0x07, 0x3C
    };

    expect_reason("broadcast encode",
                  motor_control_encode(&stop, MOTOR_SELECT_MASK, &id, data), MOTOR_R_NONE);
    if (id != 0x01Fu) {
        fail("broadcast", "id");
    }
    expect_bytes("broadcast disable", data, k_stop, MOTOR_PROTOCOL_DLC);
    expect_reason("broadcast decode",
                  motor_control_decode(0x01Fu, k_stop, MOTOR_PROTOCOL_DLC, &out), MOTOR_R_NONE);

    expect_reason("arm", motor_control_encode(&arm, MOTOR_SELECT_BR, &id, data), MOTOR_R_NONE);
    expect_reason("arm decode", motor_control_decode(id, data, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_NONE);
    if (out.flags != MOTOR_FLAG_ARM || out.session != 0x9D || out.mode != MOTOR_MODE_DISABLE) {
        fail("arm", "fields");
    }

    clear.flags = MOTOR_FLAG_CLEAR_FAULT;
    clear.session = 0x3C;
    expect_reason("clear", motor_control_encode(&clear, MOTOR_SELECT_BL, &id, data),
                  MOTOR_R_NONE);
    expect_reason("clear decode",
                  motor_control_decode(id, data, MOTOR_PROTOCOL_DLC, &out), MOTOR_R_NONE);
    if (out.flags != MOTOR_FLAG_CLEAR_FAULT ||
        id != (MOTOR_CAN_ID_CONTROL | MOTOR_SELECT_BL)) {
        fail("clear", "fields");
    }

    expect_reason("current",
                  motor_control_encode(&current, MOTOR_SELECT_FL, &id, data), MOTOR_R_NONE);
    if (id != 0x018u) {
        fail("current", "id");
    }
    expect_bytes("current 2500 mA", data, k_current, MOTOR_PROTOCOL_DLC);

    expect_reason("status broadcast poll",
                  motor_poll_encode(MOTOR_CAN_ID_STATUS_READ, MOTOR_SELECT_MASK, &id, poll),
                  MOTOR_R_NONE);
    expect_reason("status broadcast poll decode",
                  motor_poll_decode(0x10Fu, poll, MOTOR_POLL_DLC, &command, &select),
                  MOTOR_R_NONE);
    if (id != 0x10Fu || command != MOTOR_CAN_ID_STATUS_READ || select != MOTOR_SELECT_MASK) {
        fail("status broadcast poll", "fields");
    }
}

static void test_rejects(void)
{
    struct motor_control speed = {
        .mode = MOTOR_MODE_SPEED, .flags = 0, .setpoint = 120, .ff_ma = 0,
        .seq = 5, .session = 0x3C
    };
    struct motor_control both = {
        .mode = MOTOR_MODE_DISABLE,
        .flags = (uint8_t)(MOTOR_FLAG_ARM | MOTOR_FLAG_CLEAR_FAULT),
        .setpoint = 0, .ff_ma = 0, .seq = 1, .session = 1
    };
    struct motor_control bad_mode = speed;
    struct motor_control with_ff = {
        .mode = MOTOR_MODE_CURRENT, .flags = 0, .setpoint = 2500, .ff_ma = 1,
        .seq = 5, .session = 0x3C
    };
    struct motor_control dirty_stop = {
        .mode = MOTOR_MODE_DISABLE, .flags = 0, .setpoint = 1, .ff_ma = 0,
        .seq = 5, .session = 0x3C
    };
    struct motor_control arm_speed = speed;
    arm_speed.flags = MOTOR_FLAG_ARM;
    struct motor_control out;
    struct motor_status status_out;
    uint8_t data[MOTOR_PROTOCOL_DLC];
    uint8_t version2[MOTOR_PROTOCOL_DLC] = {0x23, 0x00, 0x78, 0x00, 0x00, 0x00, 0x05, 0x3C};
    uint8_t reserved[MOTOR_PROTOCOL_DLC] = {0x11, 0x07, 0x78, 0x00, 0x00, 0x00, 0x05, 0x3C};
    uint32_t id = 0xA5A5A5A5u;
    uint32_t command = 7;
    uint32_t select = 7;
    int i;

    memset(data, 0x5A, sizeof data);
    memset(&out, 0x5A, sizeof out);
    bad_mode.mode = 3;
    expect_reason("mode 3", motor_control_encode(&bad_mode, MOTOR_SELECT_FR, &id, data),
                  MOTOR_R_BAD_MODE);
    if (id != 0xA5A5A5A5u || !same_bytes(data, MOTOR_PROTOCOL_DLC, 0x5A)) {
        fail("mode 3", "encode wrote outputs");
    }

    expect_reason("version before mode",
                  motor_control_decode(0x011u, version2, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_BAD_VERSION);
    if (out.seq != 5 || out.mode != 3) {
        fail("version before mode", "fields not filled");
    }

    expect_reason("reserved flag before ARM+CLEAR",
                  motor_control_decode(0x011u, reserved, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_RESERVED_NONZERO);
    expect_reason("ARM and CLEAR",
                  motor_control_encode(&both, MOTOR_SELECT_FR, &id, data), MOTOR_R_BAD_FLAGS);
    expect_reason("ARM with SPEED",
                  motor_control_encode(&arm_speed, MOTOR_SELECT_FR, &id, data),
                  MOTOR_R_BAD_FLAGS);
    expect_reason("current with ff",
                  motor_control_encode(&with_ff, MOTOR_SELECT_FR, &id, data),
                  MOTOR_R_RESERVED_NONZERO);
    expect_reason("disable with setpoint",
                  motor_control_encode(&dirty_stop, MOTOR_SELECT_FR, &id, data),
                  MOTOR_R_RESERVED_NONZERO);
    expect_reason("speed to two wheels",
                  motor_control_encode(&speed, MOTOR_SELECT_FR | MOTOR_SELECT_BR, &id, data),
                  MOTOR_R_BAD_SELECT);
    expect_reason("status to two wheels",
                  motor_status_encode(&(struct motor_status){0},
                                      MOTOR_SELECT_FR | MOTOR_SELECT_BR, &id, data),
                  MOTOR_R_BAD_SELECT);
    if (id != 0xA5A5A5A5u || !same_bytes(data, MOTOR_PROTOCOL_DLC, 0x5A)) {
        fail("rejected encode", "outputs changed");
    }

    memset(&status_out, 0x5A, sizeof status_out);
    expect_reason("short status",
                  motor_status_decode(0x281u, k_status_120, 4u, &status_out),
                  MOTOR_R_BAD_LENGTH);
    for (i = 0; i < (int)sizeof status_out; ++i) {
        if (((const uint8_t *)&status_out)[i] != 0x5A) {
            fail("short status", "out was written");
            break;
        }
    }
    expect_reason("status two wheels",
                  motor_status_decode(0x283u, k_status_120, MOTOR_PROTOCOL_DLC, &status_out),
                  MOTOR_R_BAD_SELECT);
    if (status_out.rpm != 120 || status_out.iq_counts != MOTOR_COUNTS_PER_AMP) {
        fail("status two wheels", "payload not filled");
    }
    expect_reason("status as control",
                  motor_control_decode(0x281u, k_status_120, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_BAD_LENGTH);
    expect_reason("null control",
                  motor_control_decode(0x011u, NULL, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_BAD_LENGTH);
    expect_reason("poll no wheel",
                  motor_poll_decode(MOTOR_CAN_ID_DIAG_READ, data, MOTOR_POLL_DLC, &command, &select),
                  MOTOR_R_BAD_SELECT);
    if (command != 7 || select != 7) {
        fail("poll no wheel", "outputs written");
    }
    expect_reason("not a poll",
                  motor_poll_encode(MOTOR_CAN_ID_CONTROL, MOTOR_SELECT_FR, &id, data),
                  MOTOR_R_BAD_LENGTH);
}

int main(void)
{
    test_golden_control();
    test_golden_status();
    test_golden_diag();
    test_signed_control_bytes();
    test_round_trip_each_wheel();
    test_broadcast_and_arm();
    test_rejects();
    if (g_failures != 0) {
        fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    printf("motor protocol: CONTROL, STATUS, DIAG ok\n");
    return 0;
}
