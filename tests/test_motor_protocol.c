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

/* CONTROL_REPLY, FR. Seq 5 applied, RUNNING, first Hall edge, session 0x3C,
 * 118 RPM, 1.25 A. Id 0x411 = 0x410 | FR.
 *   byte 0  request seq 5
 *   byte 1  applied seq 5
 *   byte 2  0x43 = RUNNING (3) | APPLIED (0) | first_edge
 *   byte 3  reason NONE
 *   byte 4  session 0x3C
 *   bytes 5-6  0x0076 = 118 RPM
 *   byte 7  0x19 = 25 steps of 50 mA = 1250 mA
 *
 * Negative twin: -118 RPM (0xFF8A), -1250 mA (step -25 = 0xE7).
 */
static const uint8_t k_reply_118[MOTOR_PROTOCOL_DLC] = {
    0x05, 0x05, 0x43, 0x00, 0x3C, 0x76, 0x00, 0x19
};
static const uint8_t k_reply_neg[MOTOR_PROTOCOL_DLC] = {
    0x05, 0x05, 0x43, 0x00, 0x3C, 0x8A, 0xFF, 0xE7
};

static int reply_match(const struct motor_reply *got, const struct motor_reply *want)
{
    return got->request_seq == want->request_seq && got->applied_seq == want->applied_seq &&
           got->lifecycle == want->lifecycle && got->outcome == want->outcome &&
           got->first_edge == want->first_edge && got->reason == want->reason &&
           got->session == want->session && got->rpm == want->rpm && got->iq_ma == want->iq_ma;
}

static int struct_untouched(const void *p, unsigned n)
{
    unsigned i;
    const uint8_t *b = p;

    for (i = 0; i < n; ++i) {
        if (b[i] != 0x5A) {
            return 0;
        }
    }
    return 1;
}

static void test_reason_numbers(void)
{
    /* The lifecycle is not implemented here. The numbers still have to match
     * the wire table, and BAD_LENGTH stays 20. */
    if (MOTOR_R_OUT_OF_RANGE != 6 || MOTOR_R_NOT_ARMED != 7 || MOTOR_R_BAD_SESSION != 8 ||
        MOTOR_R_STALE != 9 || MOTOR_R_SEQ_AMBIGUOUS != 10 || MOTOR_R_SEQ_JUMP != 11 ||
        MOTOR_R_NOT_ALIGNED != 12 || MOTOR_R_BUSY != 13 || MOTOR_R_MODE_CHANGE != 14 ||
        MOTOR_R_NOT_IDLE != 15 || MOTOR_R_LIMITED != 16 || MOTOR_R_STARTING != 17 ||
        MOTOR_R_FF_UNUSED != 18 || MOTOR_R_CONFLICT != 19 || MOTOR_R_BAD_LENGTH != 20 ||
        MOTOR_R_UNSUPPORTED != 21 || MOTOR_R_STALLED != 22 || MOTOR_R_FAULT_MCSDK != 32 ||
        MOTOR_R_FAULT_STALL != 33 || MOTOR_R_FAULT_HALL_LOST != 34 || MOTOR_R_TIMEOUT != 35 ||
        MOTOR_R_LIFECYCLE_OVERFLOW != 36 || MOTOR_SEQ_WINDOW != 16u ||
        MOTOR_REPLY_IQ_STEP_MA != 50 || (int)MOTOR_LC_RUNNING != 3 ||
        (int)MOTOR_OUT_DUPLICATE != 4) {
        fail("reason numbers", "wire codes drifted");
    }
}

static void test_golden_reply(void)
{
    struct motor_reply in = {
        .request_seq = 5,
        .applied_seq = 5,
        .lifecycle = MOTOR_LC_RUNNING,
        .outcome = MOTOR_OUT_APPLIED,
        .first_edge = 1,
        .reason = MOTOR_R_NONE,
        .session = 0x3C,
        .rpm = 118,
        .iq_ma = 1250
    };
    struct motor_reply neg = in;
    struct motor_reply out;
    uint8_t data[MOTOR_PROTOCOL_DLC];
    uint32_t id = 0;

    if (25 * MOTOR_REPLY_IQ_STEP_MA != 1250) {
        fail("golden reply", "1.25 A is not 25 steps");
    }
    memset(&out, 0, sizeof out);
    expect_reason("golden reply encode",
                  motor_reply_encode(&in, MOTOR_SELECT_FR, &id, data), MOTOR_R_NONE);
    if (id != 0x411u || id != (MOTOR_CAN_ID_CONTROL_REPLY | MOTOR_SELECT_FR)) {
        fprintf(stderr, "FAIL golden reply id: %03X\n", id);
        g_failures++;
    }
    expect_bytes("golden reply", data, k_reply_118, MOTOR_PROTOCOL_DLC);
    expect_reason("golden reply decode",
                  motor_reply_decode(0x411u, k_reply_118, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_NONE);
    if (!reply_match(&out, &in)) {
        fail("golden reply decode", "fields");
    }

    neg.rpm = -118;
    neg.iq_ma = -1250;
    expect_reason("negative reply encode",
                  motor_reply_encode(&neg, MOTOR_SELECT_FR, &id, data), MOTOR_R_NONE);
    expect_bytes("negative reply", data, k_reply_neg, MOTOR_PROTOCOL_DLC);
    expect_reason("negative reply decode",
                  motor_reply_decode(id, k_reply_neg, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_NONE);
    if (!reply_match(&out, &neg)) {
        fail("negative reply decode", "fields");
    }
}

static void test_reply_each_wheel(void)
{
    int w;
    int life;
    int outcome;
    int edge;

    for (w = 0; w < 4; ++w) {
        struct motor_reply in = {
            .request_seq = (uint8_t)(9 + w),
            .applied_seq = (uint8_t)(3 + w),
            .lifecycle = MOTOR_LC_HOLD,
            .outcome = MOTOR_OUT_INHIBITED,
            .first_edge = 0,
            .reason = MOTOR_R_LIMITED,
            .session = (uint8_t)(1u + k_wheel[w]),
            .rpm = (int16_t)(-40 * (w + 1)),
            .iq_ma = (int16_t)((w % 2 == 0 ? 1 : -1) * 500)
        };
        struct motor_reply out;
        uint8_t data[MOTOR_PROTOCOL_DLC];
        uint32_t id = 0;
        char name[32];

        snprintf(name, sizeof name, "reply %s", k_wheel_name[w]);
        expect_reason(name, motor_reply_encode(&in, k_wheel[w], &id, data), MOTOR_R_NONE);
        expect_reason(name, motor_reply_decode(id, data, MOTOR_PROTOCOL_DLC, &out),
                      MOTOR_R_NONE);
        if (id != (MOTOR_CAN_ID_CONTROL_REPLY | k_wheel[w]) || !reply_match(&out, &in)) {
            fail(name, "round trip");
        }
    }

    for (life = 0; life <= (int)MOTOR_LC_FAULTED; ++life) {
        for (outcome = 0; outcome <= (int)MOTOR_OUT_DUPLICATE; ++outcome) {
            for (edge = 0; edge <= 1; ++edge) {
                struct motor_reply in = {
                    .request_seq = 0xA5,
                    .applied_seq = 0x5A,
                    .lifecycle = (uint8_t)life,
                    .outcome = (uint8_t)outcome,
                    .first_edge = (uint8_t)edge,
                    .reason = 0,
                    .session = 255,
                    .rpm = -120,
                    .iq_ma = -1250
                };
                struct motor_reply out;
                uint8_t data[MOTOR_PROTOCOL_DLC];
                uint32_t id = 0;
                uint8_t byte2 = (uint8_t)(life | (outcome << 3) | (edge ? 0x40 : 0));

                expect_reason("reply status",
                              motor_reply_encode(&in, MOTOR_SELECT_BR, &id, data),
                              MOTOR_R_NONE);
                if (id != 0x412u || data[2] != byte2 || data[5] != 0x88u || data[6] != 0xFFu ||
                    data[7] != 0xE7u) {
                    fail("reply status", "packing");
                    return;
                }
                expect_reason("reply status decode",
                              motor_reply_decode(id, data, MOTOR_PROTOCOL_DLC, &out),
                              MOTOR_R_NONE);
                if (!reply_match(&out, &in)) {
                    fail("reply status", "fields");
                    return;
                }
            }
        }
    }
}

static void test_reply_iq_steps(void)
{
    static const struct {
        int16_t ma;
        int8_t steps;
    } cases[] = {
        {0, 0}, {1, 0}, {-1, 0}, {24, 0}, {-24, 0}, {25, 1}, {-25, -1},
        {26, 1}, {-26, -1}, {74, 1}, {-74, -1}, {75, 2}, {-75, -2},
        {1249, 25}, {1250, 25}, {-1250, -25}, {1274, 25}, {-1274, -25},
        {1275, 26}, {-1275, -26}, {6324, 126}, {-6324, -126},
        {6325, 127}, {-6325, -127}, {6350, 127}, {-6350, -127},
        {6351, 127}, {-6351, -127}, {8000, 127}, {-8000, -127},
        {INT16_MAX, 127}, {INT16_MIN, -127}
    };
    unsigned i;

    for (i = 0; i < sizeof cases / sizeof cases[0]; ++i) {
        struct motor_reply in = {
            .request_seq = 5, .applied_seq = 5, .lifecycle = MOTOR_LC_RUNNING,
            .outcome = MOTOR_OUT_APPLIED, .first_edge = 1, .reason = 0,
            .session = 0x3C, .rpm = 118, .iq_ma = cases[i].ma
        };
        struct motor_reply out;
        uint8_t data[MOTOR_PROTOCOL_DLC];
        uint32_t id = 0;
        int16_t want_ma = (int16_t)(cases[i].steps * MOTOR_REPLY_IQ_STEP_MA);

        expect_reason("iq step",
                      motor_reply_encode(&in, MOTOR_SELECT_FR, &id, data), MOTOR_R_NONE);
        if (data[7] != (uint8_t)cases[i].steps ||
            data[0] != 0x05 || data[5] != 0x76) {
            fprintf(stderr, "FAIL iq %d: byte7 %02X want %02X\n",
                    (int)cases[i].ma, data[7], (uint8_t)cases[i].steps);
            g_failures++;
            return;
        }
        expect_reason("iq step decode",
                      motor_reply_decode(0x411u, data, MOTOR_PROTOCOL_DLC, &out),
                      MOTOR_R_NONE);
        if (out.iq_ma != want_ma || out.rpm != 118) {
            fprintf(stderr, "FAIL iq %d: decoded %d want %d\n",
                    (int)cases[i].ma, (int)out.iq_ma, (int)want_ma);
            g_failures++;
            return;
        }
    }
}

static void test_reply_rejects(void)
{
    struct motor_reply in = {
        .request_seq = 5, .applied_seq = 5, .lifecycle = MOTOR_LC_RUNNING,
        .outcome = MOTOR_OUT_APPLIED, .first_edge = 1, .reason = MOTOR_R_CONFLICT,
        .session = 0x3C, .rpm = 118, .iq_ma = 1250
    };
    struct motor_reply bad = in;
    struct motor_reply out;
    uint8_t data[MOTOR_PROTOCOL_DLC];
    uint8_t packed[MOTOR_PROTOCOL_DLC];
    uint32_t id = 0xA5A5A5A5u;
    unsigned reason;

    memset(data, 0x5A, sizeof data);
    bad.lifecycle = 6;
    expect_reason("reply lifecycle 6",
                  motor_reply_encode(&bad, MOTOR_SELECT_FR, &id, data),
                  MOTOR_R_RESERVED_NONZERO);
    bad = in;
    bad.outcome = 5;
    expect_reason("reply outcome 5",
                  motor_reply_encode(&bad, MOTOR_SELECT_FR, &id, data),
                  MOTOR_R_RESERVED_NONZERO);
    expect_reason("reply two wheels",
                  motor_reply_encode(&in, MOTOR_SELECT_FR | MOTOR_SELECT_BR, &id, data),
                  MOTOR_R_BAD_SELECT);
    expect_reason("reply no wheel",
                  motor_reply_encode(&in, 0, &id, data), MOTOR_R_BAD_SELECT);
    if (id != 0xA5A5A5A5u || !same_bytes(data, MOTOR_PROTOCOL_DLC, 0x5A)) {
        fail("rejected reply encode", "outputs changed");
    }

    expect_reason("reply reason passthrough",
                  motor_reply_encode(&in, MOTOR_SELECT_FL, &id, packed), MOTOR_R_NONE);
    if (packed[3] != MOTOR_R_CONFLICT || id != 0x418u) {
        fail("reply reason", "CONFLICT not carried");
    }
    for (reason = 0; reason <= 255u; ++reason) {
        in.reason = (uint8_t)reason;
        expect_reason("reply any reason",
                      motor_reply_encode(&in, MOTOR_SELECT_FL, &id, packed), MOTOR_R_NONE);
        expect_reason("reply any reason decode",
                      motor_reply_decode(id, packed, MOTOR_PROTOCOL_DLC, &out),
                      MOTOR_R_NONE);
        if (out.reason != (uint8_t)reason) {
            fail("reply any reason", "byte changed");
            return;
        }
    }

    memset(&out, 0x5A, sizeof out);
    expect_reason("reply short",
                  motor_reply_decode(0x411u, k_reply_118, 4u, &out), MOTOR_R_BAD_LENGTH);
    expect_reason("reply status id",
                  motor_reply_decode(0x281u, k_reply_118, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_BAD_LENGTH);
    packed[2] = 0xC3;
    expect_reason("reply bit 7",
                  motor_reply_decode(0x411u, packed, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_RESERVED_NONZERO);
    packed[2] = 6;
    expect_reason("reply decode lifecycle 6",
                  motor_reply_decode(0x411u, packed, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_RESERVED_NONZERO);
    packed[2] = (uint8_t)(5u << 3);
    expect_reason("reply decode outcome 5",
                  motor_reply_decode(0x411u, packed, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_RESERVED_NONZERO);
    if (!struct_untouched(&out, sizeof out)) {
        fail("rejected reply decode", "out was written");
    }

    expect_reason("reply decode two wheels",
                  motor_reply_decode(0x413u, k_reply_118, MOTOR_PROTOCOL_DLC, &out),
                  MOTOR_R_BAD_SELECT);
    if (out.rpm != 118 || out.iq_ma != 1250 || out.lifecycle != MOTOR_LC_RUNNING ||
        out.first_edge != 1 || out.session != 0x3C) {
        fail("reply decode two wheels", "payload not filled");
    }
}

/* Signed 8-bit distance, written out so it is not the implementation. */
static enum motor_seq_class seq_window(uint8_t last_seq, uint8_t seq)
{
    int d = (int)seq - (int)last_seq;

    if (d > 127) {
        d -= 256;
    } else if (d < -128) {
        d += 256;
    }
    if (d == 0) {
        return MOTOR_SEQ_DUPLICATE;
    }
    if (d == -128) {
        return MOTOR_SEQ_AMBIGUOUS;
    }
    if (d < 0) {
        return MOTOR_SEQ_STALE;
    }
    if (d <= (int)MOTOR_SEQ_WINDOW) {
        return MOTOR_SEQ_NEXT;
    }
    return MOTOR_SEQ_JUMP;
}

static void test_seq_classify(void)
{
    static const struct {
        uint8_t seq;
        enum motor_seq_class want;
    } from_250[] = {
        {251, MOTOR_SEQ_NEXT},       /* d = 1 */
        {10, MOTOR_SEQ_NEXT},        /* d = 16 */
        {11, MOTOR_SEQ_JUMP},        /* d = 17 */
        {250, MOTOR_SEQ_DUPLICATE},  /* d = 0 */
        {249, MOTOR_SEQ_STALE},      /* d = -1 */
        {122, MOTOR_SEQ_AMBIGUOUS}   /* d = -128 */
    };
    int last;
    int seq;
    unsigned i;

    for (i = 0; i < sizeof from_250 / sizeof from_250[0]; ++i) {
        enum motor_seq_class got = motor_seq_classify(250, from_250[i].seq);

        if (got != from_250[i].want) {
            fprintf(stderr, "FAIL seq 250 -> %u: class %d want %d\n",
                    from_250[i].seq, (int)got, (int)from_250[i].want);
            g_failures++;
        }
    }
    for (last = 0; last < 256; ++last) {
        for (seq = 0; seq < 256; ++seq) {
            if (motor_seq_classify((uint8_t)last, (uint8_t)seq) !=
                seq_window((uint8_t)last, (uint8_t)seq)) {
                fprintf(stderr, "FAIL seq window last %d seq %d\n", last, seq);
                g_failures++;
                return;
            }
        }
    }
}

/* Documented formula, including the boot modulus of 255. */
static uint8_t session_ref(uint8_t prev, uint32_t entropy)
{
    if (prev == 0u) {
        return (uint8_t)(1u + (entropy % 255u));
    }
    return (uint8_t)(1u + ((uint32_t)(prev - 1u) + 1u + (entropy % 254u)) % 255u);
}

static void test_session_next(void)
{
    static const uint32_t extra[] = {
        0, 253, 254, 255, 256, 508, 509, 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFFu
    };
    int prev;
    unsigned i;

    if (motor_session_next(0, MOTOR_SELECT_FR) != 2u ||
        motor_session_next(0, MOTOR_SELECT_BR) != 3u ||
        motor_session_next(0, MOTOR_SELECT_BL) != 5u ||
        motor_session_next(0, MOTOR_SELECT_FL) != 9u) {
        fail("boot session", "1 + select");
    }
    for (prev = 0; prev < 256; ++prev) {
        uint8_t seen[256];
        unsigned span = (prev == 0) ? 255u : 254u;
        unsigned entropy;

        memset(seen, 0, sizeof seen);
        for (entropy = 0; entropy < span; ++entropy) {
            uint8_t next = motor_session_next((uint8_t)prev, entropy);

            if (next == 0u || next == (uint8_t)prev || seen[next] != 0u ||
                next != session_ref((uint8_t)prev, entropy)) {
                fprintf(stderr, "FAIL session prev %d entropy %u -> %u\n",
                        prev, entropy, next);
                g_failures++;
                return;
            }
            seen[next] = 1u;
        }
        if (seen[0] != 0u) {
            fail("session", "produced 0");
            return;
        }
        for (i = 1; i <= 255u; ++i) {
            if (seen[i] != (i != (unsigned)prev)) {
                fprintf(stderr, "FAIL session reach prev %d value %u\n", prev, i);
                g_failures++;
                return;
            }
        }
        for (i = 0; i < sizeof extra / sizeof extra[0]; ++i) {
            uint8_t next = motor_session_next((uint8_t)prev, extra[i]);

            if (next == 0u || next == (uint8_t)prev ||
                next != session_ref((uint8_t)prev, extra[i])) {
                fprintf(stderr, "FAIL session prev %d entropy %u -> %u\n",
                        prev, extra[i], next);
                g_failures++;
                return;
            }
        }
    }
}

int main(void)
{
    test_reason_numbers();
    test_golden_control();
    test_golden_status();
    test_golden_diag();
    test_golden_reply();
    test_signed_control_bytes();
    test_round_trip_each_wheel();
    test_broadcast_and_arm();
    test_rejects();
    test_reply_each_wheel();
    test_reply_iq_steps();
    test_reply_rejects();
    test_seq_classify();
    test_session_next();
    if (g_failures != 0) {
        fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    printf("motor protocol: CONTROL, CONTROL_REPLY, STATUS, DIAG ok\n");
    return 0;
}
