#include "motor_cmd.h"

#include <stdio.h>
#include <string.h>

/* Host slice: arm, session, sequence, watchdog, STARTING/RUNNING. No MCSDK. */

static const uint32_t k_wheel[4] = {
    MOTOR_SELECT_FR, MOTOR_SELECT_BR, MOTOR_SELECT_BL, MOTOR_SELECT_FL
};

static int g_failures;

static void fail(const char *name, const char *msg)
{
    fprintf(stderr, "FAIL %s: %s\n", name, msg);
    g_failures++;
}

static void expect_eq(const char *name, int got, int want)
{
    if (got != want) {
        fprintf(stderr, "FAIL %s: got %d want %d\n", name, got, want);
        g_failures++;
    }
}

static struct motor_control ctrl(uint8_t mode, uint8_t flags, int16_t sp, int16_t ff,
                                 uint8_t seq, uint8_t session)
{
    struct motor_control c;

    memset(&c, 0, sizeof c);
    c.mode = mode;
    c.flags = flags;
    c.setpoint = sp;
    c.ff_ma = ff;
    c.seq = seq;
    c.session = session;
    return c;
}

static void boot(struct motor_cmd *m, uint32_t select, uint8_t current_supported)
{
    struct motor_cmd_cfg cfg;

    memset(&cfg, 0, sizeof cfg);
    cfg.select = select;
    cfg.max_rpm = 900;
    cfg.max_ma = 8000;
    cfg.current_supported = current_supported;
    motor_cmd_init(m, &cfg, select);
}

static void expect_reply(const char *name, const struct motor_reply *r, uint8_t req,
                         uint8_t applied, uint8_t lifecycle, uint8_t outcome,
                         uint8_t reason, uint8_t session)
{
    if (r->request_seq != req || r->applied_seq != applied || r->lifecycle != lifecycle ||
        r->outcome != outcome || r->reason != reason || r->session != session ||
        r->first_edge != 0u || r->rpm != 0 || r->iq_ma != 0) {
        fprintf(stderr,
                "FAIL %s reply: req %u applied %u lc %u out %u why %u sess %u edge %u rpm %d iq %d\n",
                name, r->request_seq, r->applied_seq, r->lifecycle, r->outcome, r->reason,
                r->session, r->first_edge, r->rpm, r->iq_ma);
        g_failures++;
    }
}

static int wire_matches(const struct motor_reply *out, uint32_t reply_id, const uint8_t *wire)
{
    struct motor_reply decoded;

    memset(&decoded, 0, sizeof decoded);
    if (motor_reply_decode(reply_id, wire, MOTOR_PROTOCOL_DLC, &decoded) != MOTOR_R_NONE) {
        return 0;
    }
    return decoded.request_seq == out->request_seq && decoded.applied_seq == out->applied_seq &&
           decoded.lifecycle == out->lifecycle && decoded.outcome == out->outcome &&
           decoded.first_edge == out->first_edge && decoded.reason == out->reason &&
           decoded.session == out->session && decoded.rpm == out->rpm && decoded.iq_ma == out->iq_ma;
}

static enum motor_reason submit(struct motor_cmd *m, const struct motor_control *frame,
                                uint32_t select, struct motor_reply *out)
{
    uint8_t data[MOTOR_PROTOCOL_DLC];
    uint8_t wire[MOTOR_PROTOCOL_DLC];
    uint32_t id = 0;
    uint32_t reply_id = 0;
    enum motor_reason encoded;
    enum motor_reason reason;

    encoded = motor_control_encode(frame, select, &id, data);
    if (encoded != MOTOR_R_NONE) {
        fprintf(stderr, "FAIL submit encode %d\n", (int)encoded);
        g_failures++;
        return encoded;
    }
    reason = motor_cmd_control(m, id, data, MOTOR_PROTOCOL_DLC, 0u, NULL, NULL, &reply_id, wire, out);
    if (reply_id != (MOTOR_CAN_ID_CONTROL_REPLY | m->cfg.select) ||
        !wire_matches(out, reply_id, wire)) {
        fail("submit", "reply wire");
    }
    return reason;
}

static void test_init_session(void)
{
    unsigned i;

    for (i = 0; i < 4u; ++i) {
        struct motor_cmd m;

        boot(&m, k_wheel[i], 0);
        expect_eq("boot lifecycle", m.lifecycle, MOTOR_LC_DISARMED);
        expect_eq("boot session", m.session, (int)(1u + k_wheel[i]));
        expect_eq("boot setpoint", m.setpoint, 0);
        expect_eq("boot ff", m.ff_ma, 0);
        expect_eq("boot applied", m.applied_seq, 0);
        if (m.session != motor_session_next(0, k_wheel[i])) {
            fail("boot session", "helper");
        }
    }
}

static void test_wheel(uint32_t select)
{
    struct motor_cmd m;
    struct motor_cmd saved;
    struct motor_reply r;
    struct motor_control c;
    uint8_t session;

    boot(&m, select, 0);
    session = m.session;

    /* Disarmed motion is NOT_ARMED even when the session and sequence are nonsense. */
    c = ctrl(MOTOR_MODE_SPEED, 0, 10, 0, 200, (uint8_t)(session + 1u));
    saved = m;
    expect_eq("disarmed motion", submit(&m, &c, select, &r), MOTOR_R_NOT_ARMED);
    expect_reply("disarmed motion", &r, 200, 0, MOTOR_LC_DISARMED, MOTOR_OUT_REJECTED,
                 MOTOR_R_NOT_ARMED, session);
    if (memcmp(&saved, &m, sizeof saved) != 0) {
        fail("disarmed motion", "state changed");
    }

    c = ctrl(MOTOR_MODE_DISABLE, 0, 0, 0, 1, 0);
    expect_eq("disable disarmed", submit(&m, &c, select, &r), MOTOR_R_NONE);
    expect_reply("disable disarmed", &r, 1, 0, MOTOR_LC_DISARMED, MOTOR_OUT_APPLIED,
                 MOTOR_R_NONE, session);
    expect_eq("disable disarmed lc", m.lifecycle, MOTOR_LC_DISARMED);

    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_CLEAR_FAULT, 0, 0, 2, session);
    saved = m;
    expect_eq("clear unsupported", submit(&m, &c, select, &r), MOTOR_R_UNSUPPORTED);
    expect_reply("clear unsupported", &r, 2, 0, MOTOR_LC_DISARMED, MOTOR_OUT_REJECTED,
                 MOTOR_R_UNSUPPORTED, session);
    if (memcmp(&saved, &m, sizeof saved) != 0) {
        fail("clear unsupported", "state changed");
    }

    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 7, (uint8_t)(session + 1u));
    saved = m;
    expect_eq("bad session", submit(&m, &c, select, &r), MOTOR_R_BAD_SESSION);
    expect_reply("bad session", &r, 7, 0, MOTOR_LC_DISARMED, MOTOR_OUT_REJECTED,
                 MOTOR_R_BAD_SESSION, session);
    if (memcmp(&saved, &m, sizeof saved) != 0) {
        fail("bad session", "state changed");
    }

    /* ARM sequence 250. Distances below are the uint8 step from 250. */
    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 250, session);
    expect_eq("arm", submit(&m, &c, select, &r), MOTOR_R_NONE);
    expect_reply("arm", &r, 250, 0, MOTOR_LC_READY, MOTOR_OUT_APPLIED, MOTOR_R_NONE, session);
    expect_eq("arm lc", m.lifecycle, MOTOR_LC_READY);
    expect_eq("arm seq", m.last_seq, 250);
    expect_eq("arm target", m.setpoint, 0);

    saved = m;
    c = ctrl(MOTOR_MODE_SPEED, 0, 1, 0, 122, session); /* d = -128 */
    expect_eq("ambiguous", submit(&m, &c, select, &r), MOTOR_R_SEQ_AMBIGUOUS);
    expect_reply("ambiguous", &r, 122, 0, MOTOR_LC_READY, MOTOR_OUT_REJECTED,
                 MOTOR_R_SEQ_AMBIGUOUS, session);
    c = ctrl(MOTOR_MODE_SPEED, 0, 1, 0, 249, session); /* d = -1 */
    expect_eq("stale", submit(&m, &c, select, &r), MOTOR_R_STALE);
    c = ctrl(MOTOR_MODE_SPEED, 0, 1, 0, 11, session); /* d = 17 */
    expect_eq("jump", submit(&m, &c, select, &r), MOTOR_R_SEQ_JUMP);
    c = ctrl(MOTOR_MODE_SPEED, 0, 1, 0, 250, session); /* same seq, nothing stored yet */
    expect_eq("conflict baseline", submit(&m, &c, select, &r), MOTOR_R_CONFLICT);
    c = ctrl(MOTOR_MODE_SPEED, 0, 1, 0, 9, (uint8_t)(session ^ 0xFFu)); /* would be NEXT */
    expect_eq("bad session motion", submit(&m, &c, select, &r), MOTOR_R_BAD_SESSION);
    if (memcmp(&saved, &m, sizeof saved) != 0) {
        fail("seq rejects", "state changed");
    }

    c = ctrl(MOTOR_MODE_SPEED, 0, 100, -8000, 10, session); /* d = 16, at the ff limit */
    expect_eq("window edge", submit(&m, &c, select, &r), MOTOR_R_NONE);
    expect_reply("window edge", &r, 10, 10, MOTOR_LC_READY, MOTOR_OUT_APPLIED, MOTOR_R_NONE,
                 session);
    expect_eq("applied rpm", m.setpoint, 100);
    expect_eq("applied ff", m.ff_ma, -8000);
    expect_eq("stay ready", m.lifecycle, MOTOR_LC_READY);
    expect_eq("applied seq", m.applied_seq, 10);

    saved = m;
    expect_eq("duplicate", submit(&m, &c, select, &r), MOTOR_R_NONE);
    expect_reply("duplicate", &r, 10, 10, MOTOR_LC_READY, MOTOR_OUT_DUPLICATE, MOTOR_R_NONE,
                 session);
    c = ctrl(MOTOR_MODE_SPEED, 0, 101, -8000, 10, session);
    expect_eq("conflict", submit(&m, &c, select, &r), MOTOR_R_CONFLICT);
    expect_reply("conflict", &r, 10, 10, MOTOR_LC_READY, MOTOR_OUT_REJECTED, MOTOR_R_CONFLICT,
                 session);
    if (memcmp(&saved, &m, sizeof saved) != 0) {
        fail("duplicate", "state changed");
    }

    c = ctrl(MOTOR_MODE_SPEED, 0, 901, 0, 11, session);
    saved = m;
    expect_eq("over rpm", submit(&m, &c, select, &r), MOTOR_R_OUT_OF_RANGE);
    c = ctrl(MOTOR_MODE_SPEED, 0, -901, 0, 11, session);
    expect_eq("over rpm neg", submit(&m, &c, select, &r), MOTOR_R_OUT_OF_RANGE);
    c = ctrl(MOTOR_MODE_SPEED, 0, 100, 8001, 11, session);
    expect_eq("over ff", submit(&m, &c, select, &r), MOTOR_R_OUT_OF_RANGE);
    c = ctrl(MOTOR_MODE_SPEED, 0, 100, -8001, 11, session);
    expect_eq("over ff neg", submit(&m, &c, select, &r), MOTOR_R_OUT_OF_RANGE);
    c = ctrl(MOTOR_MODE_CURRENT, 0, 100, 0, 11, session);
    expect_eq("current off", submit(&m, &c, select, &r), MOTOR_R_UNSUPPORTED);
    if (memcmp(&saved, &m, sizeof saved) != 0) {
        fail("range", "state changed");
    }

    c = ctrl(MOTOR_MODE_SPEED, 0, -900, 8000, 11, session);
    expect_eq("at limit", submit(&m, &c, select, &r), MOTOR_R_NONE);
    expect_eq("at limit rpm", m.setpoint, -900);
    expect_eq("at limit ff", m.ff_ma, 8000);
    expect_eq("at limit lc", m.lifecycle, MOTOR_LC_READY);

    c = ctrl(MOTOR_MODE_DISABLE, 0, 0, 0, 40, 0);
    expect_eq("disable", submit(&m, &c, select, &r), MOTOR_R_NONE);
    expect_reply("disable", &r, 40, 0, MOTOR_LC_READY, MOTOR_OUT_APPLIED, MOTOR_R_NONE, session);
    expect_eq("disable lc", m.lifecycle, MOTOR_LC_READY);
    expect_eq("disable target", m.setpoint, 0);
    expect_eq("disable ff", m.ff_ma, 0);
    expect_eq("disable seq kept", m.last_seq, 11);

    /* The payload is still remembered, so the retransmission does not restore it. */
    c = ctrl(MOTOR_MODE_SPEED, 0, -900, 8000, 11, session);
    expect_eq("dup after disable", submit(&m, &c, select, &r), MOTOR_R_NONE);
    expect_eq("dup outcome", r.outcome, MOTOR_OUT_DUPLICATE);
    expect_eq("dup target stays clear", m.setpoint, 0);
}

static void test_current_and_rearm(uint32_t select)
{
    struct motor_cmd m;
    struct motor_reply r;
    struct motor_control c;
    uint8_t session;

    boot(&m, select, 1);
    session = m.session;
    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 1, session);
    expect_eq("rearm arm", submit(&m, &c, select, &r), MOTOR_R_NONE);
    c = ctrl(MOTOR_MODE_CURRENT, 0, 1500, 0, 2, session);
    expect_eq("current", submit(&m, &c, select, &r), MOTOR_R_NONE);
    expect_eq("current ma", m.setpoint, 1500);
    expect_eq("current mode", m.mode, MOTOR_MODE_CURRENT);
    expect_eq("current lc", m.lifecycle, MOTOR_LC_READY);

    c = ctrl(MOTOR_MODE_CURRENT, 0, 8001, 0, 3, session);
    expect_eq("current range", submit(&m, &c, select, &r), MOTOR_R_OUT_OF_RANGE);
    expect_eq("current range kept", m.setpoint, 1500);

    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 9, session);
    expect_eq("rearm", submit(&m, &c, select, &r), MOTOR_R_NONE);
    expect_eq("rearm lc", m.lifecycle, MOTOR_LC_READY);
    expect_eq("rearm seq", m.last_seq, 9);
    expect_eq("rearm keeps target", m.setpoint, 1500);
    expect_eq("rearm forgets payload", m.last_valid, 0);

    c = ctrl(MOTOR_MODE_CURRENT, 0, 1500, 0, 9, session);
    expect_eq("rearm conflict", submit(&m, &c, select, &r), MOTOR_R_CONFLICT);
    expect_eq("rearm conflict target", m.setpoint, 1500);
}

static void test_faulted_arm(uint32_t select)
{
    struct motor_cmd m;
    struct motor_cmd saved;
    struct motor_reply r;
    struct motor_control c;

    boot(&m, select, 0);
    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 1, m.session);
    expect_eq("fault setup arm", submit(&m, &c, select, &r), MOTOR_R_NONE);
    c = ctrl(MOTOR_MODE_SPEED, 0, 20, 0, 2, m.session);
    expect_eq("fault setup motion", submit(&m, &c, select, &r), MOTOR_R_NONE);
    m.lifecycle = MOTOR_LC_FAULTED;
    m.latched = MOTOR_R_FAULT_MCSDK;

    saved = m;
    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 3, 0);
    expect_eq("fault arm bad session", submit(&m, &c, select, &r), MOTOR_R_FAULT_MCSDK);
    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 3, m.session);
    expect_eq("fault arm", submit(&m, &c, select, &r), MOTOR_R_FAULT_MCSDK);
    expect_reply("fault arm", &r, 3, 2, MOTOR_LC_FAULTED, MOTOR_OUT_REJECTED,
                 MOTOR_R_FAULT_MCSDK, m.session);
    c = ctrl(MOTOR_MODE_SPEED, 0, 20, 0, 4, m.session);
    expect_eq("fault motion", submit(&m, &c, select, &r), MOTOR_R_FAULT_MCSDK);
    if (memcmp(&saved, &m, sizeof saved) != 0) {
        fail("fault arm", "state changed");
    }

    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_CLEAR_FAULT, 0, 0, 5, m.session);
    expect_eq("clear fault", submit(&m, &c, select, &r), MOTOR_R_UNSUPPORTED);
    expect_reply("clear fault", &r, 5, 2, MOTOR_LC_FAULTED, MOTOR_OUT_REJECTED,
                 MOTOR_R_UNSUPPORTED, m.session);
    if (memcmp(&saved, &m, sizeof saved) != 0) {
        fail("clear fault", "state changed");
    }

    c = ctrl(MOTOR_MODE_DISABLE, 0, 0, 0, 6, 0);
    expect_eq("disable faulted", submit(&m, &c, select, &r), MOTOR_R_NONE);
    expect_eq("disable faulted lc", m.lifecycle, MOTOR_LC_FAULTED);
    expect_eq("disable faulted target", m.setpoint, 0);
    expect_eq("disable faulted latch", m.latched, MOTOR_R_FAULT_MCSDK);
}

static void test_structural_and_broadcast(void)
{
    unsigned i;

    for (i = 0; i < 4u; ++i) {
        struct motor_cmd m;
        struct motor_cmd saved;
        struct motor_reply r;
        struct motor_control c;
        uint8_t raw[MOTOR_PROTOCOL_DLC];
        uint8_t wire[MOTOR_PROTOCOL_DLC];
        uint32_t id;
        uint32_t reply_id;
        uint32_t other;

        boot(&m, k_wheel[i], 0);
        saved = m;
        memset(raw, 0, sizeof raw);
        raw[0] = 0x13; /* version 1, mode 3 */
        raw[6] = 9;
        id = MOTOR_CAN_ID_CONTROL | k_wheel[i];
        reply_id = 0;
        expect_eq("bad mode",
                  motor_cmd_control(&m, id, raw, MOTOR_PROTOCOL_DLC, 0u, NULL, NULL, &reply_id, wire,
                                    &r),
                  MOTOR_R_BAD_MODE);
        expect_reply("bad mode", &r, 9, 0, MOTOR_LC_DISARMED, MOTOR_OUT_REJECTED,
                     MOTOR_R_BAD_MODE, m.session);
        if (memcmp(&saved, &m, sizeof saved) != 0) {
            fail("bad mode", "state changed");
        }

        other = k_wheel[(i + 1u) % 4u];
        c = ctrl(MOTOR_MODE_SPEED, 0, 10, 0, 1, m.session);
        expect_eq("foreign wheel", submit(&m, &c, other, &r), MOTOR_R_BAD_SELECT);
        if (memcmp(&saved, &m, sizeof saved) != 0) {
            fail("foreign wheel", "state changed");
        }

        c = ctrl(MOTOR_MODE_DISABLE, 0, 0, 0, 4, 0);
        expect_eq("broadcast disable", submit(&m, &c, MOTOR_SELECT_MASK, &r), MOTOR_R_NONE);
        expect_eq("broadcast lc", m.lifecycle, MOTOR_LC_DISARMED);
        expect_reply("broadcast disable", &r, 4, 0, MOTOR_LC_DISARMED, MOTOR_OUT_APPLIED,
                     MOTOR_R_NONE, saved.session);

        memset(wire, 0xA5, sizeof wire);
        reply_id = 0xA5A5A5A5u;
        memset(&r, 0x5A, sizeof r);
        if (motor_cmd_control(&m, id, raw, 4u, 0u, NULL, NULL, &reply_id, wire, &r) !=
                MOTOR_R_BAD_LENGTH ||
            reply_id != 0xA5A5A5A5u || wire[0] != 0xA5 || r.request_seq != 0x5A) {
            fail("bad length", "reply was written");
        }
    }
}

static void test_arm_reply_bytes(void)
{
    struct motor_cmd m;
    struct motor_control c;
    struct motor_reply r;
    uint8_t data[MOTOR_PROTOCOL_DLC];
    uint8_t wire[MOTOR_PROTOCOL_DLC];
    uint8_t want[MOTOR_PROTOCOL_DLC] = {5, 0, 0x01, 0, 2, 0, 0, 0};
    uint32_t id = 0;
    uint32_t reply_id = 0;
    unsigned i;

    boot(&m, MOTOR_SELECT_FR, 0);
    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 5, m.session);
    expect_eq("golden encode", motor_control_encode(&c, MOTOR_SELECT_FR, &id, data), MOTOR_R_NONE);
    expect_eq("golden arm",
              motor_cmd_control(&m, id, data, MOTOR_PROTOCOL_DLC, 0u, NULL, NULL, &reply_id, wire, &r),
              MOTOR_R_NONE);
    expect_eq("golden id", (int)reply_id, (int)(MOTOR_CAN_ID_CONTROL_REPLY | MOTOR_SELECT_FR));
    for (i = 0; i < MOTOR_PROTOCOL_DLC; ++i) {
        if (wire[i] != want[i]) {
            fprintf(stderr, "FAIL golden reply byte %u: got %02X want %02X\n", i, wire[i], want[i]);
            g_failures++;
        }
    }
}

static struct motor_cmd_inputs kin(uint32_t now_ms, uint8_t mc, uint8_t aligned, uint8_t busy)
{
    struct motor_cmd_inputs in;

    memset(&in, 0, sizeof in);
    in.now_ms = now_ms;
    in.mc = mc;
    in.aligned = aligned;
    in.busy = busy;
    return in;
}

static enum motor_reason submit_at(struct motor_cmd *m, const struct motor_control *frame,
                                   uint32_t select, uint32_t rx_ms,
                                   const struct motor_cmd_inputs *in,
                                   struct motor_cmd_actions *act, struct motor_reply *out)
{
    uint8_t data[MOTOR_PROTOCOL_DLC];
    uint8_t wire[MOTOR_PROTOCOL_DLC];
    uint32_t id = 0;
    uint32_t reply_id = 0;
    enum motor_reason encoded;
    enum motor_reason reason;

    encoded = motor_control_encode(frame, select, &id, data);
    if (encoded != MOTOR_R_NONE) {
        fprintf(stderr, "FAIL submit_at encode %d\n", (int)encoded);
        g_failures++;
        return encoded;
    }
    reason = motor_cmd_control(m, id, data, MOTOR_PROTOCOL_DLC, rx_ms, in, act, &reply_id, wire,
                               out);
    if (reply_id != (MOTOR_CAN_ID_CONTROL_REPLY | m->cfg.select) ||
        !wire_matches(out, reply_id, wire)) {
        fail("submit_at", "reply wire");
    }
    return reason;
}

static void arm_at(struct motor_cmd *m, uint32_t select, uint32_t t, struct motor_reply *r)
{
    struct motor_cmd_inputs in = kin(t, MOTOR_CMD_MC_IDLE, 1, 0);
    struct motor_cmd_actions act;
    struct motor_control c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 1, m->session);

    expect_eq("arm_at", submit_at(m, &c, select, t, &in, &act, r), MOTOR_R_NONE);
}

/* Tick at 99 stays armed; tick at 100 disarms onto a new session. */
static void test_watchdog(uint32_t select)
{
    struct motor_cmd m;
    struct motor_reply r;
    struct motor_control c;
    struct motor_cmd_inputs in;
    struct motor_cmd_actions act;
    uint8_t session;

    boot(&m, select, 0);
    session = m.session;
    arm_at(&m, select, 0, &r);
    expect_eq("wd arm lc", m.lifecycle, MOTOR_LC_READY);
    expect_eq("wd arm why", m.disarm_reason, 0);

    in = kin(MOTOR_WATCHDOG_MS - 1u, MOTOR_CMD_MC_IDLE, 1, 0);
    motor_cmd_tick(&m, &in, &act);
    expect_eq("wd 99 lc", m.lifecycle, MOTOR_LC_READY);
    expect_eq("wd 99 sess", m.session, session);
    expect_eq("wd 99 stop", act.stop, 0);

    in = kin(MOTOR_WATCHDOG_MS, MOTOR_CMD_MC_IDLE, 1, 0);
    motor_cmd_tick(&m, &in, &act);
    expect_eq("wd 100 lc", m.lifecycle, MOTOR_LC_DISARMED);
    expect_eq("wd 100 sess", m.session, (int)motor_session_next(session, MOTOR_WATCHDOG_MS));
    expect_eq("wd 100 applied", m.applied_seq, 0);
    expect_eq("wd 100 sp", m.setpoint, 0);
    expect_eq("wd 100 why", m.disarm_reason, MOTOR_R_TIMEOUT);
    expect_eq("wd 100 stop", act.stop, 0);

    c = ctrl(MOTOR_MODE_SPEED, 0, 10, 0, 2, session);
    in = kin(MOTOR_WATCHDOG_MS, MOTOR_CMD_MC_IDLE, 1, 0);
    expect_eq("wd old motion",
              submit_at(&m, &c, select, MOTOR_WATCHDOG_MS, &in, &act, &r), MOTOR_R_NOT_ARMED);
    expect_reply("wd old motion", &r, 2, 0, MOTOR_LC_DISARMED, MOTOR_OUT_REJECTED,
                 MOTOR_R_NOT_ARMED, m.session);
    expect_eq("wd old sp", m.setpoint, 0);

    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 3, m.session);
    expect_eq("wd rearm", submit_at(&m, &c, select, MOTOR_WATCHDOG_MS, &in, &act, &r),
              MOTOR_R_NONE);
    expect_eq("wd rearm why", m.disarm_reason, 0);
    expect_eq("wd rearm lc", m.lifecycle, MOTOR_LC_READY);
}

/* STARTING is output-on, so the same trip asks the caller to stop. */
static void test_watchdog_stops(uint32_t select)
{
    struct motor_cmd m;
    struct motor_reply r;
    struct motor_control c;
    struct motor_cmd_inputs in;
    struct motor_cmd_actions act;
    uint8_t session;

    boot(&m, select, 0);
    arm_at(&m, select, 0, &r);
    session = m.session;
    in = kin(0, MOTOR_CMD_MC_IDLE, 1, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, 25, 3, 2, session);
    expect_eq("wd start", submit_at(&m, &c, select, 0, &in, &act, &r), MOTOR_R_STARTING);
    expect_eq("wd start lc", m.lifecycle, MOTOR_LC_STARTING);
    expect_eq("wd start go", act.start, 1);
    expect_eq("wd start tgt", act.set_target, 1);
    expect_eq("wd start stop", act.stop, 0);

    in = kin(MOTOR_WATCHDOG_MS, MOTOR_CMD_MC_STARTING, 1, 0);
    motor_cmd_tick(&m, &in, &act);
    expect_eq("wd stop lc", m.lifecycle, MOTOR_LC_DISARMED);
    expect_eq("wd stop sess", m.session, (int)motor_session_next(session, MOTOR_WATCHDOG_MS));
    expect_eq("wd stop flag", act.stop, 1);
    expect_eq("wd stop start", act.start, 0);
    expect_eq("wd stop sp", m.setpoint, 0);
    expect_eq("wd stop seq", m.applied_seq, 0);
}

/* DISABLE, a second ARM, a duplicate, and a reject leave the deadline where it was. */
static void test_watchdog_ignores(uint32_t select)
{
    struct motor_cmd m;
    struct motor_reply r;
    struct motor_control c;
    struct motor_cmd_inputs in;
    struct motor_cmd_actions act;
    uint8_t session;

    boot(&m, select, 0);
    session = m.session;
    arm_at(&m, select, 0, &r);
    in = kin(50, MOTOR_CMD_MC_IDLE, 0, 0);
    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 4, session);
    expect_eq("ign arm", submit_at(&m, &c, select, 50, &in, &act, &r), MOTOR_R_NONE);
    expect_eq("ign arm watch", (int)m.watch_ms, 0);
    in = kin(MOTOR_WATCHDOG_MS - 1u, MOTOR_CMD_MC_IDLE, 0, 0);
    motor_cmd_tick(&m, &in, &act);
    expect_eq("ign arm 99", m.lifecycle, MOTOR_LC_READY);
    in = kin(MOTOR_WATCHDOG_MS, MOTOR_CMD_MC_IDLE, 0, 0);
    motor_cmd_tick(&m, &in, &act);
    expect_eq("ign arm 100", m.lifecycle, MOTOR_LC_DISARMED);

    boot(&m, select, 0);
    session = m.session;
    arm_at(&m, select, 0, &r);
    in = kin(10, MOTOR_CMD_MC_IDLE, 0, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, 20, 0, 2, session);
    expect_eq("ign motion", submit_at(&m, &c, select, 10, &in, &act, &r), MOTOR_R_STARTING);
    expect_eq("ign motion watch", (int)m.watch_ms, 10);
    expect_eq("ign motion lc", m.lifecycle, MOTOR_LC_READY); /* not aligned: held, not started */

    c = ctrl(MOTOR_MODE_SPEED, 0, 20, 0, 2, session);
    in = kin(100, MOTOR_CMD_MC_IDLE, 0, 0);
    expect_eq("ign dup", submit_at(&m, &c, select, 100, &in, &act, &r), MOTOR_R_NONE);
    expect_eq("ign dup out", r.outcome, MOTOR_OUT_DUPLICATE);
    expect_eq("ign dup watch", (int)m.watch_ms, 10);

    boot(&m, select, 0);
    session = m.session;
    arm_at(&m, select, 0, &r);
    in = kin(10, MOTOR_CMD_MC_IDLE, 0, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, 20, 0, 2, session);
    expect_eq("ign base", submit_at(&m, &c, select, 10, &in, &act, &r), MOTOR_R_STARTING);
    c = ctrl(MOTOR_MODE_SPEED, 0, 901, 0, 3, session);
    in = kin(100, MOTOR_CMD_MC_IDLE, 0, 0);
    expect_eq("ign range", submit_at(&m, &c, select, 100, &in, &act, &r), MOTOR_R_OUT_OF_RANGE);
    expect_eq("ign range watch", (int)m.watch_ms, 10);
    expect_eq("ign range sp", m.setpoint, 20);

    boot(&m, select, 0);
    session = m.session;
    arm_at(&m, select, 0, &r);
    in = kin(10, MOTOR_CMD_MC_IDLE, 0, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, 20, 0, 2, session);
    expect_eq("ign dis base", submit_at(&m, &c, select, 10, &in, &act, &r), MOTOR_R_STARTING);
    c = ctrl(MOTOR_MODE_DISABLE, 0, 0, 0, 8, 0);
    in = kin(50, MOTOR_CMD_MC_IDLE, 0, 0);
    expect_eq("ign dis", submit_at(&m, &c, select, 50, &in, &act, &r), MOTOR_R_NONE);
    expect_eq("ign dis watch", (int)m.watch_ms, 10);
    expect_eq("ign dis sess", m.session, session);
    expect_eq("ign dis sp", m.setpoint, 0);

    in = kin(10u + MOTOR_WATCHDOG_MS - 1u, MOTOR_CMD_MC_IDLE, 0, 0);
    motor_cmd_tick(&m, &in, &act);
    expect_eq("ign dis 99", m.lifecycle, MOTOR_LC_READY);
    in = kin(10u + MOTOR_WATCHDOG_MS, MOTOR_CMD_MC_IDLE, 0, 0);
    motor_cmd_tick(&m, &in, &act);
    expect_eq("ign dis 100", m.lifecycle, MOTOR_LC_DISARMED);
    expect_eq("ign dis new", m.session, (int)motor_session_next(session, 10u + MOTOR_WATCHDOG_MS));
}

static void test_late_motion(uint32_t select)
{
    struct motor_cmd m;
    struct motor_reply r;
    struct motor_control c;
    struct motor_cmd_inputs in;
    struct motor_cmd_actions act;
    uint8_t session;

    boot(&m, select, 0);
    session = m.session;
    in = kin(1000, MOTOR_CMD_MC_IDLE, 1, 0);
    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 1, session);
    expect_eq("late arm", submit_at(&m, &c, select, 1000, &in, &act, &r), MOTOR_R_NONE);

    /* First motion is already 100 ms old: rejected, and it must not start the bridge. */
    c = ctrl(MOTOR_MODE_SPEED, 0, 80, 0, 2, session);
    expect_eq("late first", submit_at(&m, &c, select, 1000u - MOTOR_WATCHDOG_MS, &in, &act, &r),
              MOTOR_R_TIMEOUT);
    expect_reply("late first", &r, 2, 0, MOTOR_LC_READY, MOTOR_OUT_REJECTED, MOTOR_R_TIMEOUT,
                 session);
    expect_eq("late first lc", m.lifecycle, MOTOR_LC_READY);
    expect_eq("late first sp", m.setpoint, 0);
    expect_eq("late first start", act.start, 0);
    expect_eq("late first seq", m.last_seq, 1);
    expect_eq("late first watch", (int)m.watch_ms, 1000);

    /* 99 ms old is still fresh. */
    in = kin(1000u + MOTOR_WATCHDOG_MS - 1u, MOTOR_CMD_MC_IDLE, 1, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, 11, 4, 2, session);
    expect_eq("age 99",
              submit_at(&m, &c, select, 1000, &in, &act, &r), MOTOR_R_STARTING);
    expect_eq("age 99 sp", m.setpoint, 11);
    expect_eq("age 99 ff", m.ff_ma, 4);
    expect_eq("age 99 lc", m.lifecycle, MOTOR_LC_STARTING);
    expect_reply("age 99", &r, 2, 2, MOTOR_LC_STARTING, MOTOR_OUT_INHIBITED, MOTOR_R_STARTING,
                 session);

    /* Older than the accepted frame: rejected, target kept, watchdog not tripped. */
    c = ctrl(MOTOR_MODE_SPEED, 0, 77, 0, 3, session);
    expect_eq("late held", submit_at(&m, &c, select, 999, &in, &act, &r), MOTOR_R_TIMEOUT);
    expect_reply("late held", &r, 3, 2, MOTOR_LC_STARTING, MOTOR_OUT_REJECTED, MOTOR_R_TIMEOUT,
                 session);
    expect_eq("late held lc", m.lifecycle, MOTOR_LC_STARTING);
    expect_eq("late held sp", m.setpoint, 11);
    expect_eq("late held seq", m.last_seq, 2);
    expect_eq("late held start", act.start, 0);

    /* Exactly 100 ms after the accepted frame: TIMEOUT, then the watchdog disarms. */
    in = kin(1000u + MOTOR_WATCHDOG_MS, MOTOR_CMD_MC_STARTING, 1, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, 99, 0, 4, session);
    expect_eq("late exact",
              submit_at(&m, &c, select, 1000, &in, &act, &r), MOTOR_R_TIMEOUT);
    expect_reply("late exact", &r, 4, 2, MOTOR_LC_STARTING, MOTOR_OUT_REJECTED, MOTOR_R_TIMEOUT,
                 session);
    expect_eq("late exact lc", m.lifecycle, MOTOR_LC_DISARMED);
    expect_eq("late exact seq", m.last_seq, 2);
    expect_eq("late exact sp", m.setpoint, 0);
    expect_eq("late exact sess", m.session,
              (int)motor_session_next(session, 1000u + MOTOR_WATCHDOG_MS));

    /* Receive time exactly 100 ms after the last accept finds the wheel already disarmed. */
    boot(&m, select, 0);
    session = m.session;
    arm_at(&m, select, 1000, &r);
    in = kin(1000, MOTOR_CMD_MC_IDLE, 1, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, 33, 0, 2, session);
    expect_eq("dl motion", submit_at(&m, &c, select, 1000, &in, &act, &r), MOTOR_R_STARTING);
    in = kin(1099, MOTOR_CMD_MC_STARTING, 1, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, 34, 0, 3, session);
    expect_eq("dl 99", submit_at(&m, &c, select, 1099, &in, &act, &r), MOTOR_R_STARTING);
    expect_eq("dl 99 sp", m.setpoint, 34);
    expect_eq("dl 99 lc", m.lifecycle, MOTOR_LC_STARTING);
    in = kin(1199, MOTOR_CMD_MC_STARTING, 1, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, 35, 0, 4, session);
    expect_eq("dl 100", submit_at(&m, &c, select, 1199, &in, &act, &r), MOTOR_R_NOT_ARMED);
    expect_reply("dl 100", &r, 4, 0, MOTOR_LC_DISARMED, MOTOR_OUT_REJECTED, MOTOR_R_NOT_ARMED,
                 m.session);
    expect_eq("dl 100 stop", act.stop, 1);
    expect_eq("dl 100 sp", m.setpoint, 0);
    expect_eq("dl 100 sess", m.session, (int)motor_session_next(session, 1199));

    /* A late ARM still arms, then the same call disarms because its receive time is the clock. */
    boot(&m, select, 0);
    session = m.session;
    in = kin(1000, MOTOR_CMD_MC_IDLE, 1, 0);
    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 1, session);
    expect_eq("late arm frame",
              submit_at(&m, &c, select, 1000u - MOTOR_WATCHDOG_MS, &in, &act, &r), MOTOR_R_NONE);
    expect_reply("late arm frame", &r, 1, 0, MOTOR_LC_READY, MOTOR_OUT_APPLIED, MOTOR_R_NONE,
                 session);
    expect_eq("late arm state", m.lifecycle, MOTOR_LC_DISARMED);
    expect_eq("late arm sess", m.session, (int)motor_session_next(session, 1000));
    expect_eq("late arm why", m.disarm_reason, MOTOR_R_TIMEOUT);
    expect_eq("late arm stop", act.stop, 0);
}

static void test_start_run(uint32_t select)
{
    struct motor_cmd m;
    struct motor_reply r;
    struct motor_control c;
    struct motor_cmd_inputs in;
    struct motor_cmd_actions act;
    uint8_t session;

    boot(&m, select, 1);
    session = m.session;
    arm_at(&m, select, 0, &r);

    in = kin(0, MOTOR_CMD_MC_IDLE, 1, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, 0, 0, 2, session);
    expect_eq("zero", submit_at(&m, &c, select, 0, &in, &act, &r), MOTOR_R_NONE);
    expect_eq("zero lc", m.lifecycle, MOTOR_LC_READY);
    expect_eq("zero start", act.start, 0);

    in = kin(10, MOTOR_CMD_MC_IDLE, 0, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, 40, 0, 3, session);
    expect_eq("unaligned", submit_at(&m, &c, select, 10, &in, &act, &r), MOTOR_R_STARTING);
    expect_eq("unaligned lc", m.lifecycle, MOTOR_LC_READY);
    expect_eq("unaligned start", act.start, 0);
    expect_eq("unaligned sp", m.setpoint, 40);

    in = kin(20, MOTOR_CMD_MC_IDLE, 1, 1);
    c = ctrl(MOTOR_MODE_SPEED, 0, 41, 0, 4, session);
    expect_eq("busy", submit_at(&m, &c, select, 20, &in, &act, &r), MOTOR_R_STARTING);
    expect_eq("busy lc", m.lifecycle, MOTOR_LC_READY);
    expect_eq("busy start", act.start, 0);

    in = kin(30, MOTOR_CMD_MC_STARTING, 1, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, 42, 0, 5, session);
    expect_eq("not idle", submit_at(&m, &c, select, 30, &in, &act, &r), MOTOR_R_STARTING);
    expect_eq("not idle lc", m.lifecycle, MOTOR_LC_READY);
    expect_eq("not idle start", act.start, 0);

    in = kin(40, MOTOR_CMD_MC_IDLE, 1, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, 50, 12, 6, session);
    expect_eq("starting", submit_at(&m, &c, select, 40, &in, &act, &r), MOTOR_R_STARTING);
    expect_reply("starting", &r, 6, 6, MOTOR_LC_STARTING, MOTOR_OUT_INHIBITED, MOTOR_R_STARTING,
                 session);
    expect_eq("starting lc", m.lifecycle, MOTOR_LC_STARTING);
    expect_eq("starting go", act.start, 1);
    expect_eq("starting tgt", act.set_target, 1);
    expect_eq("starting stop", act.stop, 0);
    expect_eq("starting sp", m.setpoint, 50);
    expect_eq("starting ff", m.ff_ma, 12);
    expect_eq("starting mode", m.mode, MOTOR_MODE_SPEED);

    in = kin(50, MOTOR_CMD_MC_STARTING, 1, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, 55, 12, 7, session);
    expect_eq("still", submit_at(&m, &c, select, 50, &in, &act, &r), MOTOR_R_STARTING);
    expect_eq("still lc", m.lifecycle, MOTOR_LC_STARTING);
    expect_eq("still start", act.start, 0);
    expect_eq("still tgt", act.set_target, 1);
    expect_eq("still sp", m.setpoint, 55);

    in = kin(60, MOTOR_CMD_MC_RUN, 1, 0);
    motor_cmd_tick(&m, &in, &act);
    expect_eq("run lc", m.lifecycle, MOTOR_LC_RUNNING);
    expect_eq("run start", act.start, 0);
    expect_eq("run stop", act.stop, 0);

    in = kin(70, MOTOR_CMD_MC_RUN, 1, 0);
    c = ctrl(MOTOR_MODE_SPEED, 0, -60, 0, 8, session);
    expect_eq("running", submit_at(&m, &c, select, 70, &in, &act, &r), MOTOR_R_NONE);
    expect_reply("running", &r, 8, 8, MOTOR_LC_RUNNING, MOTOR_OUT_APPLIED, MOTOR_R_NONE, session);
    expect_eq("running tgt", act.set_target, 1);
    expect_eq("running start", act.start, 0);
    expect_eq("running sp", m.setpoint, -60);

    c = ctrl(MOTOR_MODE_DISABLE, 0, 0, 0, 9, 0);
    in = kin(80, MOTOR_CMD_MC_RUN, 1, 0);
    expect_eq("stop", submit_at(&m, &c, select, 80, &in, &act, &r), MOTOR_R_NONE);
    expect_reply("stop", &r, 9, 0, MOTOR_LC_READY, MOTOR_OUT_APPLIED, MOTOR_R_NONE, session);
    expect_eq("stop flag", act.stop, 1);
    expect_eq("stop lc", m.lifecycle, MOTOR_LC_READY);
    expect_eq("stop sess", m.session, session);
    expect_eq("stop sp", m.setpoint, 0);

    in = kin(90, MOTOR_CMD_MC_IDLE, 1, 0);
    c = ctrl(MOTOR_MODE_CURRENT, 0, 100, 0, 10, session);
    expect_eq("current start", submit_at(&m, &c, select, 90, &in, &act, &r), MOTOR_R_STARTING);
    expect_eq("current lc", m.lifecycle, MOTOR_LC_STARTING);
    expect_eq("current mode", m.mode, MOTOR_MODE_CURRENT);
    expect_eq("current sp", m.setpoint, 100);
    expect_eq("current go", act.start, 1);
}

static void keep_starting(struct motor_cmd *m, uint32_t select, uint8_t session)
{
    struct motor_reply r;
    struct motor_cmd_actions act;
    struct motor_control c;
    uint32_t t;
    uint8_t seq = 2;

    for (t = 0; t <= 450u; t += 90u) {
        struct motor_cmd_inputs in = kin(t, t == 0u ? MOTOR_CMD_MC_IDLE : MOTOR_CMD_MC_STARTING, 1, 0);

        c = ctrl(MOTOR_MODE_SPEED, 0, 80, 0, seq, session);
        expect_eq("keep", submit_at(m, &c, select, t, &in, &act, &r), MOTOR_R_STARTING);
        expect_eq("keep lc", m->lifecycle, MOTOR_LC_STARTING);
        expect_eq("keep latch", m->latched, 0);
        expect_eq("keep start", act.start, t == 0u ? 1 : 0);
        seq++;
    }
}

static void test_start_timeout(uint32_t select)
{
    struct motor_cmd m;
    struct motor_cmd saved;
    struct motor_reply r;
    struct motor_control c;
    struct motor_cmd_inputs in;
    struct motor_cmd_actions act;
    uint8_t session;

    boot(&m, select, 0);
    session = m.session;
    arm_at(&m, select, 0, &r);
    keep_starting(&m, select, session);

    in = kin(MOTOR_CMD_START_TIMEOUT_MS - 1u, MOTOR_CMD_MC_STARTING, 1, 0);
    motor_cmd_tick(&m, &in, &act);
    expect_eq("t499 lc", m.lifecycle, MOTOR_LC_STARTING);
    expect_eq("t499 stop", act.stop, 0);
    expect_eq("t499 latch", m.latched, 0);

    in = kin(MOTOR_CMD_START_TIMEOUT_MS, MOTOR_CMD_MC_STARTING, 1, 0);
    motor_cmd_tick(&m, &in, &act);
    expect_eq("t500 lc", m.lifecycle, MOTOR_LC_FAULTED);
    expect_eq("t500 latch", m.latched, MOTOR_R_FAULT_MCSDK);
    expect_eq("t500 stop", act.stop, 1);
    expect_eq("t500 start", act.start, 0);
    expect_eq("t500 tgt", act.set_target, 0);
    expect_eq("t500 sp", m.setpoint, 0);
    expect_eq("t500 sess", m.session, session);

    saved = m;
    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 20, 0);
    expect_eq("t500 arm0", submit(&m, &c, select, &r), MOTOR_R_FAULT_MCSDK);
    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_ARM, 0, 0, 20, session);
    expect_eq("t500 arm", submit(&m, &c, select, &r), MOTOR_R_FAULT_MCSDK);
    c = ctrl(MOTOR_MODE_SPEED, 0, 80, 0, 21, session);
    expect_eq("t500 motion", submit(&m, &c, select, &r), MOTOR_R_FAULT_MCSDK);
    c = ctrl(MOTOR_MODE_DISABLE, MOTOR_FLAG_CLEAR_FAULT, 0, 0, 22, session);
    expect_eq("t500 clear", submit(&m, &c, select, &r), MOTOR_R_UNSUPPORTED);
    if (memcmp(&saved, &m, sizeof saved) != 0) {
        fail("t500 held", "state changed");
    }

    /* RUN on the last millisecond before the deadline enters RUNNING, not FAULTED. */
    boot(&m, select, 0);
    session = m.session;
    arm_at(&m, select, 0, &r);
    keep_starting(&m, select, session);
    in = kin(MOTOR_CMD_START_TIMEOUT_MS - 1u, MOTOR_CMD_MC_RUN, 1, 0);
    motor_cmd_tick(&m, &in, &act);
    expect_eq("run in time", m.lifecycle, MOTOR_LC_RUNNING);
    expect_eq("run in time latch", m.latched, 0);
    expect_eq("run in time stop", act.stop, 0);
}

int main(void)
{
    unsigned i;
    const uint32_t sampled[2] = {MOTOR_SELECT_FR, MOTOR_SELECT_BL};

    test_init_session();
    for (i = 0; i < 4u; ++i) {
        test_wheel(k_wheel[i]);
        test_current_and_rearm(k_wheel[i]);
        test_faulted_arm(k_wheel[i]);
    }
    test_structural_and_broadcast();
    test_arm_reply_bytes();
    for (i = 0; i < 2u; ++i) {
        test_watchdog(sampled[i]);
        test_watchdog_stops(sampled[i]);
        test_watchdog_ignores(sampled[i]);
        test_late_motion(sampled[i]);
        test_start_run(sampled[i]);
        test_start_timeout(sampled[i]);
    }
    if (g_failures != 0) {
        fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    printf("motor_cmd: arm, watchdog, starting, running ok\n");
    return 0;
}
