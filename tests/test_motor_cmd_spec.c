/* Independent Stage 2 oracle derived from A-protocol, B-lifecycle and motor_cmd.h.
 * No motor plant is simulated: start/stop/ack change sampled MCSDK state on the
 * next tick. HOLD, CURRENT operation and stall/Hall-loss monitors are Stage 4/5.
 */
#include "check.h"
#include "motor_cmd.h"
#include <string.h>

static unsigned failures, checks, wheel;
static const char *case_name;
static unsigned variant;
#define EXPECT(x) do { ++checks; if (!(x)) { \
  ++failures; fprintf(stderr, "FAIL %s wheel=0x%x variant=%u line=%d: %s\n", \
                       case_name, wheel, variant, __LINE__, #x); } } while (0)
static void equal(long long actual, long long expected, const char *expr, int line)
{
  ++checks;
  if (actual != expected) {
    ++failures;
    fprintf(stderr, "FAIL %s wheel=0x%x variant=%u line=%d: %s (got %lld, expected %lld)\n",
            case_name, wheel, variant, line, expr, actual, expected);
  }
}
#define EQ(a,b) equal((long long)(a), (long long)(b), #a " == " #b, __LINE__)

struct fixture {
  struct motor_cmd m;
  struct motor_cmd_inputs in;
  struct motor_cmd_actions a;
  int pending_mc;
};

static void init(struct fixture *f, uint32_t entropy)
{
  const struct motor_cmd_config cfg = {wheel, 2976, 6000, 0};
  memset(f, 0, sizeof(*f));
  motor_cmd_init(&f->m, &cfg, entropy);
  f->in.aligned = 1;
  f->in.limit_ma = 6000; /* IQMAX: the effective limit a cool, configured ESC reports */
  f->pending_mc = -1;
  EQ(f->m.lifecycle, MOTOR_LC_DISARMED);
  EXPECT(f->m.session != 0);
}

static void tick(struct fixture *f, uint32_t now)
{
  if (f->pending_mc >= 0) {
    f->in.mc = (uint8_t)f->pending_mc;
    f->pending_mc = -1;
  }
  f->in.now_ms = now;
  memset(&f->a, 0xa5, sizeof(f->a));
  motor_cmd_tick(&f->m, &f->in, &f->a);
  if (f->a.stop || f->a.ack_fault) f->pending_mc = MOTOR_CMD_MC_IDLE;
  if (f->a.start) f->pending_mc = MOTOR_CMD_MC_STARTING;
  EQ(!!motor_cmd_output_on(&f->m),
     f->m.lifecycle == MOTOR_LC_STARTING || f->m.lifecycle == MOTOR_LC_RUNNING ||
     f->m.lifecycle == MOTOR_LC_HOLD);
  EQ(!!motor_cmd_output_off(&f->m), !motor_cmd_output_on(&f->m));
  EXPECT(f->m.session != 0);
}

static struct motor_control command(unsigned kind, uint8_t seq, uint8_t session)
{
  struct motor_control c = {0};
  c.seq = seq;
  c.session = session;
  if (kind == 0) { c.mode = MOTOR_MODE_SPEED; c.setpoint = 120; }
  if (kind == 2) c.flags = MOTOR_FLAG_ARM;
  if (kind == 3) c.flags = MOTOR_FLAG_CLEAR_FAULT;
  return c;
}

static void rx_select(struct fixture *f, struct motor_control c, unsigned sel, uint32_t at)
{
  uint8_t data[8];
  uint32_t id;
  EQ(motor_control_encode(&c, sel, &id, data), MOTOR_R_NONE);
  motor_cmd_rx(&f->m, id, data, sizeof(data), at);
}

static void rx(struct fixture *f, struct motor_control c, uint32_t at)
{
  rx_select(f, c, wheel, at);
}

static unsigned replies(struct fixture *f, struct motor_reply *r)
{
  uint8_t data[8], again[8];
  uint32_t id, id_again;
  unsigned n = 0;
  memset(r, 0, MOTOR_CMD_REPLIES * sizeof(*r));
  while (motor_cmd_reply_peek(&f->m, &id, data)) {
    EXPECT(n < MOTOR_CMD_REPLIES);
    if (n >= MOTOR_CMD_REPLIES) break;
    EQ(id, 0x410U | wheel);
    EQ(motor_cmd_reply_peek(&f->m, &id_again, again), 1);
    EQ(id_again, id);
    EXPECT(memcmp(data, again, 8) == 0);
    EQ(data[2] & 0x80, 0);
    EQ(motor_reply_decode(data, 8, &r[n]), MOTOR_R_NONE);
    EXPECT(r[n].session != 0);
    if (r[n].lifecycle == MOTOR_LC_DISARMED || r[n].lifecycle == MOTOR_LC_READY ||
        r[n].lifecycle == MOTOR_LC_FAULTED) {
      EQ(r[n].rpm, 0);
      EQ(r[n].iq_ma, 0);
    }
    ++n;
    motor_cmd_reply_pop(&f->m);
  }
  EQ(motor_cmd_reply_peek(&f->m, &id, data), 0);
  return n;
}

static struct motor_reply one(struct fixture *f, uint8_t seq, unsigned outcome, unsigned reason)
{
  struct motor_reply r[4] = {{0}};
  EQ(replies(f, r), 1);
  EQ(r[0].request_seq, seq);
  EQ(r[0].outcome, outcome);
  EQ(r[0].reason, reason);
  EQ(r[0].session, f->m.session);
  return r[0];
}

static void empty(struct fixture *f)
{
  struct motor_reply r[4];
  EQ(replies(f, r), 0);
}

static void establish(struct fixture *f, unsigned state)
{
  init(f, 41);
  if (state == MOTOR_LC_DISARMED) return;
  rx(f, command(2, 10, f->m.session), 10);
  tick(f, 10);
  one(f, 10, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  EQ(f->m.lifecycle, MOTOR_LC_READY);
  if (state == MOTOR_LC_READY) return;
  rx(f, command(0, 11, f->m.session), 11);
  tick(f, 11);
  one(f, 11, MOTOR_OUT_INHIBITED, MOTOR_R_STARTING);
  EQ(f->a.start, 1);
  EQ(f->a.set_target, 1);
  EQ(f->m.lifecycle, MOTOR_LC_STARTING);
  if (state == MOTOR_LC_STARTING) return;
  f->pending_mc = -1;
  f->in.mc = MOTOR_CMD_MC_RUN;
  tick(f, 12);
  EQ(f->m.lifecycle, MOTOR_LC_RUNNING);
  empty(f);
  if (state == MOTOR_LC_RUNNING) return;
  f->in.mc = MOTOR_CMD_MC_FAULT;
  f->in.mc_fault_present = 1;
  tick(f, 13);
  EQ(f->m.lifecycle, MOTOR_LC_FAULTED);
  EQ(f->m.latched, MOTOR_R_FAULT_MCSDK);
  f->in.mc = MOTOR_CMD_MC_IDLE;
  f->in.mc_fault_present = 0;
  f->pending_mc = -1;
}

static const unsigned states[] = {
  MOTOR_LC_DISARMED, MOTOR_LC_READY, MOTOR_LC_STARTING,
  MOTOR_LC_RUNNING, MOTOR_LC_FAULTED
};

static void transitions(void)
{
  for (unsigned i = 0; i < sizeof(states)/sizeof(states[0]); ++i) {
    unsigned s = states[i];
    for (unsigned k = 0; k < 4; ++k) {
      struct fixture f;
      variant = s * 10 + k;
      establish(&f, s);
      uint8_t session = f.m.session, seq = f.m.last_seq;
      rx(&f, command(k, 12, session), 20);
      tick(&f, 20);
      unsigned expected = s, outcome = MOTOR_OUT_APPLIED, reason = MOTOR_R_NONE;
      if (k == 0) {
        if (s == MOTOR_LC_DISARMED) { outcome = MOTOR_OUT_REJECTED; reason = MOTOR_R_NOT_ARMED; }
        else if (s == MOTOR_LC_FAULTED) {
          // Decided (B): A lists INHIBITED(fault); B has no FAULTED motion row.
          // Assume REJECTED(latched cause) for a frame received while faulted;
          // INHIBITED applies if a monitor faults after accepting a target.
          outcome = MOTOR_OUT_REJECTED; reason = MOTOR_R_FAULT_MCSDK;
        } else if (s == MOTOR_LC_READY || s == MOTOR_LC_STARTING) {
          expected = MOTOR_LC_STARTING; outcome = MOTOR_OUT_INHIBITED; reason = MOTOR_R_STARTING;
        }
      } else if (k == 1) {
        if (s == MOTOR_LC_STARTING || s == MOTOR_LC_RUNNING) expected = MOTOR_LC_READY;
        EQ(f.a.stop, s == MOTOR_LC_STARTING || s == MOTOR_LC_RUNNING);
        EQ(f.a.abort_busy, 0);
        EQ(f.m.last_seq, seq);
        // A: a plain APPLIED carries reason NONE, also for DISABLE while FAULTED.
      } else if (k == 2) {
        if (s == MOTOR_LC_DISARMED) expected = MOTOR_LC_READY;
        if (s == MOTOR_LC_FAULTED) {
          // Decided (B): B excludes FAULTED from ARM but supplies no verdict.
          // Assume REJECTED with the latched fault.
          outcome = MOTOR_OUT_REJECTED; reason = MOTOR_R_FAULT_MCSDK;
        } else EQ(f.m.last_seq, 12);
      } else if (s == MOTOR_LC_FAULTED) {
        // Decided (B): notes require DISABLE+CLEAR_FAULT, but the table has no
        // DISABLE prerequisite. Follow the table: idle, cause gone suffices.
        expected = MOTOR_LC_DISARMED;
        // B: a successful CLEAR_FAULT acknowledges the MCSDK fault.
        EQ(f.a.ack_fault, 1);
        EXPECT(f.m.session != session);
        EQ(f.m.applied_seq, 0);
      }
      EQ(f.m.lifecycle, expected);
      if (!(k == 3 && s == MOTOR_LC_FAULTED)) EQ(f.m.session, session);
      if (k != 0) EQ(f.a.start, 0);
      struct motor_reply r = one(&f, 12, outcome, reason);
      EQ(r.lifecycle, expected);
      EQ(r.applied_seq, f.m.applied_seq);
    }
    struct fixture f;
    establish(&f, s);
    uint8_t baseline = f.m.last_seq, session = f.m.session;
    rx(&f, command(2, 200, (uint8_t)(session + 1)), 20);
    tick(&f, 20);
    // B: an ARM with a wrong session is BAD_SESSION in any state but FAULTED, which names its latched cause
    // (amended 2026-10-22: the Nucleo's session-0 probe is how a new session learns why a wheel is off).
    one(&f, 200, MOTOR_OUT_REJECTED, s == MOTOR_LC_FAULTED ? MOTOR_R_FAULT_MCSDK : MOTOR_R_BAD_SESSION);
    EQ(f.m.lifecycle, s);
    EQ(f.m.last_seq, baseline);
    EQ(f.m.session, session);
    EQ(f.a.start, 0);
  }
}

/* Oracle for B's ordered drain: motion is deferred; lifecycle takes effect
 * immediately; accepted motion advances the baseline even when superseded. */
struct expected_reply { unsigned seq, outcome, reason, state, session, applied; };
static void ordering(void)
{
  const unsigned initial[] = {MOTOR_LC_READY, MOTOR_LC_RUNNING, MOTOR_LC_FAULTED};
  for (unsigned si = 0; si < 3; ++si) {
    for (unsigned len = 2; len <= 3; ++len) {
      unsigned combinations = len == 2 ? 16 : 64;
      for (unsigned code = 0; code < combinations; ++code) {
        struct fixture f;
        struct expected_reply e[3] = {{0}};
        struct motor_reply r[4];
        unsigned kinds[3], v = code, state = initial[si], pending = 3;
        unsigned stop = 0, ack = 0, start = 0, target = 0, rejected = 0;
        unsigned applied = state == MOTOR_LC_READY ? 0 : 11;
        variant = si * 1000 + len * 100 + code;
        establish(&f, state);
        uint8_t session = f.m.session;
        uint8_t current_session = session;
        for (unsigned j = 0; j < len; ++j) {
          kinds[j] = v % 4; v /= 4;
          unsigned k = kinds[j];
          e[j].seq = 20 + j;
          e[j].outcome = MOTOR_OUT_APPLIED;
          e[j].session = current_session;
          e[j].applied = applied;
          if (k == 0) {
            if (state == MOTOR_LC_FAULTED) {
              e[j].outcome = MOTOR_OUT_REJECTED; e[j].reason = MOTOR_R_FAULT_MCSDK;
            } else if (state == MOTOR_LC_DISARMED) {
              e[j].outcome = MOTOR_OUT_REJECTED; e[j].reason = MOTOR_R_NOT_ARMED;
            } else if (current_session != session) {
              e[j].outcome = MOTOR_OUT_REJECTED; e[j].reason = MOTOR_R_BAD_SESSION;
            } else {
              if (pending < len) { e[pending].outcome = MOTOR_OUT_SUPERSEDED; e[pending].state = state;
                e[pending].session = current_session; e[pending].applied = applied; }
              pending = j;
            }
          } else if (k == 1 || k == 2) {
            if (pending < len) { e[pending].outcome = MOTOR_OUT_SUPERSEDED; e[pending].state = state;
              e[pending].session = current_session; e[pending].applied = applied; pending = 3; }
            if (k == 1) {
              if (state == MOTOR_LC_RUNNING || state == MOTOR_LC_STARTING) { stop = 1; state = MOTOR_LC_READY; }
            } else if (current_session != session) {
              e[j].outcome = MOTOR_OUT_REJECTED; e[j].reason = MOTOR_R_BAD_SESSION;
            } else if (state == MOTOR_LC_FAULTED) {
              e[j].outcome = MOTOR_OUT_REJECTED; e[j].reason = MOTOR_R_FAULT_MCSDK;
            } else if (state == MOTOR_LC_DISARMED) state = MOTOR_LC_READY;
          } else if (state == MOTOR_LC_FAULTED) {
            state = MOTOR_LC_DISARMED; ack = 1; applied = 0;
            current_session = (uint8_t)(1U + ((unsigned)session + 20U % 254U) % 255U);
            e[j].session = current_session; e[j].applied = applied;
          }
          e[j].state = state;
          struct motor_control c = command(k, (uint8_t)e[j].seq, session);
          if (k == 0) c.setpoint = (int16_t)(120 + j * 10);
          rx(&f, c, 20);
        }
        if (pending < len) {
          /* B: the start waits for MCSDK IDLE (MC_StartMotor1 refuses otherwise), e.g. after a
           * DISABLE in the same drain; until then the target is held and answered STARTING. */
          if (state == MOTOR_LC_READY) {
            e[pending].outcome = MOTOR_OUT_INHIBITED; e[pending].reason = MOTOR_R_STARTING;
            if (f.in.mc == MOTOR_CMD_MC_IDLE) { start = 1; target = 1; state = MOTOR_LC_STARTING; }
          } else {
            target = 1;
          }
          e[pending].state = state;
          e[pending].session = current_session; e[pending].applied = e[pending].seq;
          applied = e[pending].seq;
        }
        tick(&f, 20);
        EQ(f.m.lifecycle, state);
        EQ(f.a.stop, stop);
        EQ(f.a.stop_cause, stop ? MOTOR_CMD_STOP_DISABLE : MOTOR_CMD_STOP_NONE);
        EQ(f.a.abort_busy, 0);
        // B: a successful CLEAR_FAULT acknowledges, also when MCSDK is already IDLE.
        EQ(f.a.ack_fault, ack);
        EQ(f.a.start, start); EQ(f.a.set_target, target);
        if (target) {
          EQ(f.a.mode, MOTOR_MODE_SPEED); EQ(f.a.setpoint, 120 + pending * 10); EQ(f.a.ff_ma, 0);
        }
        EQ(replies(&f, r), len);
        unsigned seen = 0;
        for (unsigned j = 0; j < len; ++j) {
          unsigned index = r[j].request_seq - 20U;
          EXPECT(index < len);
          if (index >= len) continue;
          EXPECT(!(seen & (1U << index))); seen |= 1U << index;
          EQ(r[j].outcome, e[index].outcome);
          // A: DISABLE while FAULTED is answered APPLIED with reason NONE.
          EQ(r[j].reason, e[index].reason);
          // Decided (B): A does not say whether replies snapshot each FIFO step
          // or the final tick. Assume per-step snapshots, candidate at consume.
          EQ(r[j].lifecycle, e[index].state);
          EQ(r[j].session, e[index].session);
          EQ(r[j].applied_seq, e[index].applied);
          EQ(r[j].first_edge, 0);
          if (e[index].outcome == MOTOR_OUT_REJECTED) ++rejected;
        }
        EQ(seen, (1U << len) - 1);
        EQ(f.m.count.rejected, rejected);
        EQ(f.m.count.replies_dropped, 0);
        EQ(f.m.applied_seq, applied);
        if (ack) EXPECT(f.m.session != session); else EQ(f.m.session, session);
      }
    }
  }
}

static void aged_lifecycle(void)
{
  const unsigned initial[] = {MOTOR_LC_READY, MOTOR_LC_RUNNING, MOTOR_LC_FAULTED};
  for (unsigned si = 0; si < 3; ++si) {
    for (unsigned k = 1; k < 4; ++k) {
      struct fixture f;
      variant = initial[si] * 10 + k;
      establish(&f, initial[si]);
      uint8_t session = f.m.session;
      rx(&f, command(k, 22, session), 0);
      tick(&f, 100);
      // B: only motion frames expire by receive age; a late DISABLE still stops
      // (the safe direction) and a late ARM only starts the watchdog clock.
      unsigned state = initial[si], outcome = MOTOR_OUT_APPLIED, reason = MOTOR_R_NONE;
      if (k == 1 && state == MOTOR_LC_RUNNING) state = MOTOR_LC_READY;
      if (k == 2 && state == MOTOR_LC_FAULTED) {
        outcome = MOTOR_OUT_REJECTED; reason = MOTOR_R_FAULT_MCSDK;
      }
      if (k == 3 && state == MOTOR_LC_FAULTED) state = MOTOR_LC_DISARMED;
      struct motor_reply r = one(&f, 22, outcome, reason);
      EQ(r.lifecycle, state); EQ(f.m.lifecycle, state);
      EQ(f.a.stop, k == 1 && initial[si] == MOTOR_LC_RUNNING);
      EQ(f.a.start, 0);
    }
  }
  struct fixture f;
  variant = 999;
  establish(&f, MOTOR_LC_DISARMED);
  uint8_t old = f.m.session;
  rx(&f, command(2, 1, old), 0); tick(&f, 100);
  // B: an old ARM starts the watchdog from its receive time, so it times out
  // at once and rotates the session; it is not rejected like old motion.
  struct motor_reply r[4]; EQ(replies(&f, r), 1);
  EQ(r[0].request_seq, 1); EQ(r[0].outcome, MOTOR_OUT_APPLIED);
  EQ(r[0].reason, MOTOR_R_NONE); EQ(r[0].lifecycle, MOTOR_LC_READY);
  EQ(r[0].session, old); EQ(f.m.lifecycle, MOTOR_LC_DISARMED);
  EXPECT(f.m.session != old);
}

static void ordering_rejects(void)
{
  for (unsigned which = 0; which < 4; ++which) {
    struct fixture f;
    variant = which;
    establish(&f, MOTOR_LC_RUNNING);
    struct motor_control motion = command(0, 12, f.m.session);
    rx(&f, motion, 20);
    struct motor_control next = motion;
    if (which == 0) ++next.session;
    if (which == 1) next.setpoint = 121;
    if (which == 2) next.seq = 40;
    if (which == 3) { next = command(2, 90, (uint8_t)(f.m.session + 1)); }
    rx(&f, next, 20);
    tick(&f, 20);
    struct motor_reply r[4]; EQ(replies(&f, r), 2);
    // Decided (B): a rejected ARM is not said to cancel a pending motion. Assume
    // only accepted lifecycle operations cancel it, as rejected means unchanged.
    EQ(f.m.applied_seq, 12); EQ(f.a.set_target, 1); EQ(f.a.setpoint, 120);
    unsigned seen = 0;
    for (unsigned i = 0; i < 2; ++i) {
      if (r[i].outcome == MOTOR_OUT_APPLIED) { EQ(r[i].request_seq, 12); seen |= 1; }
      if (r[i].outcome == MOTOR_OUT_REJECTED) {
        const unsigned reasons[] = {MOTOR_R_BAD_SESSION, MOTOR_R_CONFLICT, MOTOR_R_SEQ_JUMP, MOTOR_R_BAD_SESSION};
        EQ(r[i].request_seq, next.seq); EQ(r[i].reason, reasons[which]); seen |= 2;
      }
    }
    EQ(seen, 3);
  }
  struct fixture f;
  establish(&f, MOTOR_LC_READY);
  struct motor_control c = command(0, 11, f.m.session);
  rx(&f, c, 20); rx(&f, c, 20);
  tick(&f, 20);
  struct motor_reply r[4]; EQ(replies(&f, r), 2);
  EQ(r[0].request_seq, 11); EQ(r[0].outcome, MOTOR_OUT_DUPLICATE);
  EQ(r[1].request_seq, 11); EQ(r[1].outcome, MOTOR_OUT_INHIBITED);
  EQ(f.a.start, 1); EQ(f.m.count.rejected, 0);

  /* An expired later target must not supersede the fresh one before it. */
  establish(&f, MOTOR_LC_RUNNING);
  rx(&f, command(0, 12, f.m.session), 100);
  rx(&f, command(0, 13, f.m.session), 0);
  tick(&f, 100); EQ(f.m.applied_seq, 12); EQ(f.a.set_target, 1);
  EQ(replies(&f, r), 2);
  EQ(r[0].request_seq, 13); EQ(r[0].outcome, MOTOR_OUT_REJECTED); EQ(r[0].reason, MOTOR_R_TIMEOUT);
  EQ(r[1].request_seq, 12); EQ(r[1].outcome, MOTOR_OUT_APPLIED);
}

static void watchdog(void)
{
  struct fixture f;
  /* B refines A's "ARM does not refresh": first ARM starts the clock;
   * subsequent ARM while armed must not refresh it. */
  establish(&f, MOTOR_LC_DISARMED);
  uint8_t old = f.m.session;
  rx(&f, command(2, 200, old), 40);
  tick(&f, 80);
  one(&f, 200, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  tick(&f, 139); EQ(f.m.lifecycle, MOTOR_LC_READY);
  tick(&f, 140); EQ(f.m.lifecycle, MOTOR_LC_DISARMED);
  EXPECT(f.m.session != old); EQ(f.a.stop, 0);
  /* motor_cmd.h: a stop cause comes only with a real switch-off; the output was off here. */
  EQ(f.a.stop_cause, MOTOR_CMD_STOP_NONE); EQ(f.m.applied_seq, 0);

  for (unsigned kind = 0; kind < 10; ++kind) {
    variant = kind;
    establish(&f, MOTOR_LC_RUNNING);
    old = f.m.session;
    struct motor_control c = command(0, 12, old);
    if (kind == 0) { c.seq = 11; } /* exact duplicate of accepted startup frame */
    if (kind == 1) { c.session = (uint8_t)(old + 1); }
    if (kind == 2) { c.seq = 40; }
    if (kind == 3) c = command(2, 100, old);
    if (kind == 4) c = command(1, 100, 0);
    if (kind == 5) c = command(3, 100, 0);
    if (kind == 6) c.setpoint = 2977;
    if (kind == 7) c.setpoint = 121, c.seq = 11;
    if (kind == 8) c.mode = MOTOR_MODE_CURRENT;
    if (kind == 9) f.in.busy = 1;
    rx(&f, c, 90); tick(&f, 90);
    struct motor_reply r[4]; EQ(replies(&f, r), 1);
    tick(&f, 110); EXPECT(f.m.lifecycle != MOTOR_LC_DISARMED);
    tick(&f, 111); EQ(f.m.lifecycle, MOTOR_LC_DISARMED);
    EXPECT(f.m.session != old); EQ(f.m.applied_seq, 0);
    rx(&f, command(2, 201, f.m.session), 112); tick(&f, 112);
    one(&f, 201, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
    rx(&f, command(0, 202, old), 113); tick(&f, 113);
    one(&f, 202, MOTOR_OUT_REJECTED, MOTOR_R_BAD_SESSION);
    EQ(f.a.start, 0);
  }
  establish(&f, MOTOR_LC_RUNNING);
  rx(&f, command(0, 12, f.m.session), 90); tick(&f, 100);
  one(&f, 12, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  tick(&f, 189); EQ(f.m.lifecycle, MOTOR_LC_RUNNING);
  tick(&f, 190); EQ(f.m.lifecycle, MOTOR_LC_DISARMED);
  EQ(f.a.stop, 1); EQ(f.a.stop_cause, MOTOR_CMD_STOP_TIMEOUT);

  for (unsigned age = 99; age <= 101; ++age) {
    variant = age;
    establish(&f, MOTOR_LC_READY);
    rx(&f, command(0, 11, f.m.session), 10);
    tick(&f, 10 + age);
    if (age == 99) {
      one(&f, 11, MOTOR_OUT_INHIBITED, MOTOR_R_STARTING);
      EQ(f.a.start, 1); EQ(f.m.lifecycle, MOTOR_LC_STARTING);
    } else {
      struct motor_reply r[4]; EQ(replies(&f, r), 1);
      EQ(r[0].request_seq, 11); EQ(r[0].outcome, MOTOR_OUT_REJECTED);
      EQ(r[0].reason, MOTOR_R_TIMEOUT);
      // Decided (B): the expiry reply precedes watchdog disarm; its session is
      // the snapshot before that transition, not the new challenge.
      EXPECT(r[0].session != f.m.session);
      EQ(f.a.start, 0); EQ(f.m.lifecycle, MOTOR_LC_DISARMED);
    }
  }
  /* Unsigned clock rollover must preserve receive age and exact trip. */
  init(&f, 0);
  rx(&f, command(2, 1, f.m.session), UINT32_MAX - 49U);
  tick(&f, UINT32_MAX - 49U); one(&f, 1, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  tick(&f, 49); EQ(f.m.lifecycle, MOTOR_LC_READY);
  tick(&f, 50); EQ(f.m.lifecycle, MOTOR_LC_DISARMED);
}

static void sequences(void)
{
  struct fixture f;
  const int delta[] = {0, 1, 16, 17, 127, 128, 129, 255};
  const unsigned reason[] = {MOTOR_R_CONFLICT, MOTOR_R_NONE, MOTOR_R_NONE,
    MOTOR_R_SEQ_JUMP, MOTOR_R_SEQ_JUMP, MOTOR_R_SEQ_AMBIGUOUS, MOTOR_R_STALE, MOTOR_R_STALE};
  const unsigned bases[] = {0, 1, 127, 250, 255};
  for (unsigned b = 0; b < sizeof(bases)/sizeof(bases[0]); ++b) {
    for (unsigned d = 0; d < sizeof(delta)/sizeof(delta[0]); ++d) {
      variant = bases[b] * 256 + (unsigned)delta[d];
      establish(&f, MOTOR_LC_READY);
      rx(&f, command(2, (uint8_t)bases[b], f.m.session), 20); tick(&f, 20);
      one(&f, (uint8_t)bases[b], MOTOR_OUT_APPLIED, MOTOR_R_NONE);
      uint8_t seq = (uint8_t)(bases[b] + delta[d]);
      rx(&f, command(0, seq, f.m.session), 21); tick(&f, 21);
      if (reason[d] == MOTOR_R_NONE) {
        one(&f, seq, MOTOR_OUT_INHIBITED, MOTOR_R_STARTING);
        EQ(f.m.last_seq, seq);
      } else {
        // Decided (B): immediately after ARM there is no last motion payload.
        // Treat d=0 as CONFLICT rather than a duplicate of an ARM.
        one(&f, seq, MOTOR_OUT_REJECTED, reason[d]);
        EQ(f.m.last_seq, bases[b]); EQ(f.a.start, 0);
      }
    }
  }
  establish(&f, MOTOR_LC_RUNNING);
  rx(&f, command(0, 11, f.m.session), 20); tick(&f, 20);
  one(&f, 11, MOTOR_OUT_DUPLICATE, MOTOR_R_NONE); EQ(f.a.set_target, 0);
  for (unsigned field = 0; field < 3; ++field) {
    variant = field;
    struct motor_control c = command(0, 11, f.m.session);
    if (field == 0) c.setpoint = -120;
    if (field == 1) c.ff_ma = 1;
    if (field == 2) { c.mode = MOTOR_MODE_CURRENT; c.setpoint = 120; }
    rx(&f, c, 21 + field); tick(&f, 21 + field);
    // Decided (B): sequence versus capability precedence is unspecified. Follow
    // A's session/sequence paragraph first: different mode is a conflict too.
    one(&f, 11, MOTOR_OUT_REJECTED, MOTOR_R_CONFLICT);
    EQ(f.m.last_seq, 11); EQ(f.a.set_target, 0);
  }
  rx(&f, command(2, 250, f.m.session), 30); tick(&f, 30);
  one(&f, 250, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  EQ(f.m.applied_seq, 11);
  rx(&f, command(0, 255, f.m.session), 31); tick(&f, 31);
  one(&f, 255, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  rx(&f, command(0, 0, f.m.session), 32); tick(&f, 32);
  one(&f, 0, MOTOR_OUT_APPLIED, MOTOR_R_NONE); EQ(f.m.applied_seq, 0);
  rx(&f, command(1, 99, 0), 33); tick(&f, 33);
  one(&f, 99, MOTOR_OUT_APPLIED, MOTOR_R_NONE); EQ(f.m.last_seq, 0);
  rx(&f, command(0, 1, f.m.session), 34); tick(&f, 34);
  one(&f, 1, MOTOR_OUT_INHIBITED, MOTOR_R_STARTING); EQ(f.a.start, 1);

  /* ARM invalidates duplicate identity even when re-baselined to the last
   * motion's seq and the next payload would otherwise be byte-identical. */
  establish(&f, MOTOR_LC_RUNNING);
  rx(&f, command(2, 11, f.m.session), 20); tick(&f, 20);
  one(&f, 11, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  rx(&f, command(0, 11, f.m.session), 21); tick(&f, 21);
  one(&f, 11, MOTOR_OUT_REJECTED, MOTOR_R_CONFLICT); EQ(f.a.set_target, 0);

  establish(&f, MOTOR_LC_RUNNING);
  rx(&f, command(1, 199, 0), 20); tick(&f, 20);
  one(&f, 199, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  rx(&f, command(0, 11, f.m.session), 21); tick(&f, 21);
  one(&f, 11, MOTOR_OUT_DUPLICATE, MOTOR_R_NONE);
  EQ(f.m.lifecycle, MOTOR_LC_READY); EQ(f.a.start, 0); EQ(f.a.set_target, 0);
}

static void faults(void)
{
  for (unsigned i = 0; i < sizeof(states)/sizeof(states[0]); ++i) {
    struct fixture f;
    unsigned s = states[i]; variant = s;
    establish(&f, s);
    f.pending_mc = -1;
    f.in.mc = MOTOR_CMD_MC_FAULT; f.in.mc_fault_present = 1;
    tick(&f, 20);
    // Decided (B): B only supplies the MCSDK fault row with output on. Assume
    // faults latch even with output off, preventing ARM into MC_FAULT.
    EQ(f.m.lifecycle, MOTOR_LC_FAULTED);
    EQ(f.a.start, 0);
    rx(&f, command(0, 12, f.m.session), 21); tick(&f, 21);
    EQ(f.a.start, 0);
    struct motor_reply r[4]; EQ(replies(&f, r), 1);
  }
  for (unsigned mc = MOTOR_CMD_MC_IDLE; mc <= MOTOR_CMD_MC_FAULT; ++mc) {
    for (unsigned present = 0; present <= 1; ++present) {
      struct fixture f;
      variant = 100 + mc * 2 + present;
      establish(&f, MOTOR_LC_FAULTED);
      uint8_t old = f.m.session;
      f.in.mc = (uint8_t)mc; f.in.mc_fault_present = (uint8_t)present;
      f.pending_mc = -1;
      rx(&f, command(3, 90, 0), 20); tick(&f, 20);
      /* B: the cause is gone when no fault is present and MCSDK is IDLE or in FAULT_OVER
       * (MCSDK leaves FAULT_OVER only through the acknowledgement this clear sends). */
      if (!present && (mc == MOTOR_CMD_MC_IDLE || mc == MOTOR_CMD_MC_FAULT)) {
        one(&f, 90, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
        EQ(f.m.lifecycle, MOTOR_LC_DISARMED);
        EQ(f.a.ack_fault, 1);
        f.in.mc = MOTOR_CMD_MC_IDLE; /* MCSDK leaves FAULT_OVER on the acknowledgement */
        EXPECT(f.m.session != old); EQ(f.m.applied_seq, 0);
        tick(&f, 21); EQ(f.a.start, 0);
        rx(&f, command(0, 12, f.m.session), 22); tick(&f, 22);
        one(&f, 12, MOTOR_OUT_REJECTED, MOTOR_R_NOT_ARMED);
        rx(&f, command(2, 100, f.m.session), 23); tick(&f, 23);
        one(&f, 100, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
        rx(&f, command(0, 101, old), 24); tick(&f, 24);
        one(&f, 101, MOTOR_OUT_REJECTED, MOTOR_R_BAD_SESSION); EQ(f.a.start, 0);
      } else {
        /* B permits NOT_IDLE or the latched reason while the cause persists. */
        struct motor_reply r[4]; EQ(replies(&f, r), 1);
        EQ(r[0].request_seq, 90);
        EQ(r[0].outcome, MOTOR_OUT_REJECTED);
        EXPECT(r[0].reason == MOTOR_R_NOT_IDLE || r[0].reason == MOTOR_R_FAULT_MCSDK);
        EQ(f.m.lifecycle, MOTOR_LC_FAULTED); EQ(f.m.session, old); EQ(f.a.ack_fault, 0);
      }
    }
  }
  /* Fault evaluation precedes lifecycle and candidate application. */
  struct fixture f;
  establish(&f, MOTOR_LC_RUNNING);
  f.in.mc = MOTOR_CMD_MC_FAULT; f.in.mc_fault_present = 1;
  rx(&f, command(0, 12, f.m.session), 20);
  rx(&f, command(1, 13, 0), 20);
  rx(&f, command(2, 14, f.m.session), 20);
  tick(&f, 20);
  EQ(f.m.lifecycle, MOTOR_LC_FAULTED); EQ(f.a.start, 0); EQ(f.a.set_target, 0);
  struct motor_reply r[4]; EQ(replies(&f, r), 3);
  EQ(r[0].request_seq, 12); EQ(r[0].outcome, MOTOR_OUT_REJECTED); EQ(r[0].reason, MOTOR_R_FAULT_MCSDK);
  EQ(r[1].request_seq, 13); EQ(r[1].outcome, MOTOR_OUT_APPLIED);
  EQ(r[2].request_seq, 14); EQ(r[2].outcome, MOTOR_OUT_REJECTED);
}

static void stage2_monitors(void)
{
  struct fixture f;
  /* Keep receiving commands so the command watchdog cannot mask startup expiry. */
  establish(&f, MOTOR_LC_STARTING);
  f.pending_mc = -1; f.in.mc = MOTOR_CMD_MC_STARTING;
  uint8_t seq = 11;
  for (unsigned t = 31; t <= 491; t += 20) {
    rx(&f, command(0, ++seq, f.m.session), t); tick(&f, t);
    one(&f, seq, MOTOR_OUT_INHIBITED, MOTOR_R_STARTING);
  }
  tick(&f, 510); EQ(f.m.lifecycle, MOTOR_LC_STARTING);
  tick(&f, 511); EQ(f.m.lifecycle, MOTOR_LC_FAULTED);
  EQ(f.m.latched, MOTOR_R_FAULT_MCSDK); EQ(f.a.stop, 1);
  EQ(f.a.stop_cause, MOTOR_CMD_STOP_FAULT);

  for (unsigned offset = 499; offset <= 500; ++offset) {
    variant = offset;
    establish(&f, MOTOR_LC_STARTING);
    seq = 11;
    for (unsigned t = 31; t <= 491; t += 20) {
      rx(&f, command(0, ++seq, f.m.session), t); tick(&f, t);
      one(&f, seq, MOTOR_OUT_INHIBITED, MOTOR_R_STARTING);
    }
    f.pending_mc = -1; f.in.mc = MOTOR_CMD_MC_RUN;
    tick(&f, 11 + offset);
    EQ(f.m.lifecycle, MOTOR_LC_RUNNING); EQ(f.a.stop, 0); EQ(f.a.start, 0);
  }
  establish(&f, MOTOR_LC_RUNNING);
  f.in.stall_fault = 1;
  tick(&f, 20); EQ(f.m.lifecycle, MOTOR_LC_FAULTED);
  EQ(f.m.latched, MOTOR_R_FAULT_STALL); EQ(f.a.stop, 1);
  EQ(f.a.stop_cause, MOTOR_CMD_STOP_FAULT);
  f.in.stall_fault = 0;
  rx(&f, command(0, 12, f.m.session), 21); tick(&f, 21);
  one(&f, 12, MOTOR_OUT_REJECTED, MOTOR_R_FAULT_STALL); EQ(f.a.start, 0);
  struct motor_control failed_zero = command(0, 12, f.m.session); failed_zero.setpoint = 0;
  rx(&f, failed_zero, 22); tick(&f, 22);
  one(&f, 12, MOTOR_OUT_REJECTED, MOTOR_R_FAULT_STALL);
  EQ(f.m.lifecycle, MOTOR_LC_FAULTED); EQ(f.a.start, 0);

  /* Watchdog precedes monitors: a coincident stall verdict cannot prevent disarm. */
  establish(&f, MOTOR_LC_RUNNING); f.in.stall_fault = 1;
  tick(&f, 111); EQ(f.m.lifecycle, MOTOR_LC_DISARMED);
  EQ(f.a.stop_cause, MOTOR_CMD_STOP_TIMEOUT);
  f.in.stall_fault = 0;

  variant = 150;
  establish(&f, MOTOR_LC_RUNNING);
  uint8_t session = f.m.session;
  struct motor_control zero = command(0, 12, session); zero.setpoint = 0;
  rx(&f, zero, 20); tick(&f, 20); one(&f, 12, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  f.in.standstill = 1; seq = 12;
  for (unsigned t = 21; t <= 170; ++t) {
    if (t % 20 == 0) { zero.seq = ++seq; rx(&f, zero, t); }
    tick(&f, t);
    if (t % 20 == 0) one(&f, seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
    EQ(f.a.stop, 0);
    /* HOLD timer starts at t=21, the first tick with a zero target at standstill. */
    if (t < 171) EQ(f.m.lifecycle, t < 21 + MOTOR_CMD_HOLD_MS ? MOTOR_LC_RUNNING : MOTOR_LC_HOLD);
  }
  tick(&f, 171);
  EQ(f.m.lifecycle, MOTOR_LC_HOLD); EQ(f.a.stop, 0); EQ(f.m.session, session);
  rx(&f, command(0, ++seq, session), 172); tick(&f, 172);
  one(&f, seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE); EQ(f.a.start, 0); EQ(f.a.set_target, 1);
  EQ(f.m.lifecycle, MOTOR_LC_RUNNING);

  /* A standstill blip interrupts the dwell. */
  establish(&f, MOTOR_LC_RUNNING); zero = command(0, 12, f.m.session); zero.setpoint = 0;
  seq = 12;
  for (unsigned t = 20; t <= 180; ++t) {
    if (t % 20 == 0) { zero.seq = seq++; rx(&f, zero, t); }
    f.in.standstill = t != 100;
    tick(&f, t);
    if (t % 20 == 0) one(&f, zero.seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
    EQ(f.m.lifecycle, MOTOR_LC_RUNNING);
  }
  f.in.standstill = 0;
  /* Stall and Hall-loss faults come only from the speed loop's verdicts, never from current alone. */
  establish(&f, MOTOR_LC_RUNNING); seq = 11; f.in.iq_ma = 6000;
  for (unsigned t = 20; t <= 1020; t += 20) {
    rx(&f, command(0, ++seq, f.m.session), t); tick(&f, t);
    one(&f, seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE); EQ(f.m.lifecycle, MOTOR_LC_RUNNING);
  }
}

static void validation(void)
{
  struct fixture f;
  for (unsigned i = 0; i < sizeof(states)/sizeof(states[0]); ++i) {
    for (unsigned v = 0; v < 8; ++v) {
      variant = states[i] * 10 + v;
      establish(&f, states[i]);
      struct motor_control c = command(0, 12, f.m.session);
      unsigned reason;
      if (v < 4) {
        c.setpoint = v == 0 ? 2977 : v == 1 ? -2977 : 120;
        if (v == 2) c.ff_ma = 6001;
        if (v == 3) c.ff_ma = -6001;
        reason = MOTOR_R_OUT_OF_RANGE;
      } else if (v == 4) { c.mode = MOTOR_MODE_CURRENT; reason = MOTOR_R_UNSUPPORTED; }
      else if (v == 5) { f.in.busy = 1; reason = MOTOR_R_BUSY; }
      else if (v == 6) { f.in.aligned = 0; reason = MOTOR_R_NOT_ALIGNED; }
      else { c.session = (uint8_t)(f.m.session + 1); reason = MOTOR_R_BAD_SESSION; }
      rx(&f, c, 20); tick(&f, 20);
      struct motor_reply r[4]; EQ(replies(&f, r), 1);
      EQ(r[0].request_seq, 12);
      // Decided (B): validation precedence is only specified for structural errors.
      // For overlapping state/session/range/capability failures require safe
      // non-application; require exact reasons when only one error applies.
      if (states[i] == MOTOR_LC_RUNNING || states[i] == MOTOR_LC_READY || states[i] == MOTOR_LC_STARTING) {
        if (v == 6 && states[i] != MOTOR_LC_READY) {
          /* Alignment is only a precondition of READY -> STARTING in B. */
          EQ(r[0].outcome, states[i] == MOTOR_LC_STARTING ? MOTOR_OUT_INHIBITED : MOTOR_OUT_APPLIED);
          EQ(r[0].reason, states[i] == MOTOR_LC_STARTING ? MOTOR_R_STARTING : MOTOR_R_NONE);
        } else { EQ(r[0].outcome, MOTOR_OUT_REJECTED); EQ(r[0].reason, reason); }
      } else EXPECT(r[0].outcome == MOTOR_OUT_REJECTED || r[0].outcome == MOTOR_OUT_INHIBITED);
      EQ(f.a.start, 0);
      EQ(f.a.set_target, v == 6 && (states[i] == MOTOR_LC_STARTING || states[i] == MOTOR_LC_RUNNING));
    }
  }
  /* Structural rejects are queued too; they do not consume sequence numbers. */
  const unsigned errors[] = {MOTOR_R_BAD_VERSION, MOTOR_R_BAD_MODE, MOTOR_R_RESERVED_NONZERO,
    MOTOR_R_BAD_FLAGS, MOTOR_R_RESERVED_NONZERO, MOTOR_R_BAD_SELECT};
  for (unsigned v = 0; v < 6; ++v) {
    variant = 100 + v;
    establish(&f, MOTOR_LC_RUNNING);
    uint8_t data[8] = {0x11, 0, 120, 0, 0, 0, 12, f.m.session};
    unsigned id = 0x10 | wheel;
    if (v == 0) data[0] = 0x23; /* BAD_VERSION before BAD_MODE */
    if (v == 1) data[0] = 0x13;
    if (v == 2) data[1] = 5; /* reserved before bad flags */
    if (v == 3) data[1] = 1;
    if (v == 4) { data[0] = 0x10; }
    if (v == 5) id = 0x1f;
    motor_cmd_rx(&f.m, id, data, 8, 20); tick(&f, 20);
    one(&f, 12, MOTOR_OUT_REJECTED, errors[v]); EQ(f.m.last_seq, 11);
    EQ(f.m.count.rejected, 1); EQ(f.m.count.last_reject, errors[v]);
    rx(&f, command(0, 12, f.m.session), 21); tick(&f, 21);
    one(&f, 12, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  }
  establish(&f, MOTOR_LC_RUNNING);
  uint8_t data[8] = {0x11, 0, 120, 0, 0, 0, 12, f.m.session};
  const unsigned lengths[] = {0, 1, 7, 9, 64};
  for (unsigned i = 0; i < sizeof(lengths)/sizeof(lengths[0]); ++i)
    motor_cmd_rx(&f.m, 0x10 | wheel, data, lengths[i], 20);
  motor_cmd_rx(&f.m, 0x80 | wheel, data, 8, 20);
  motor_cmd_rx(&f.m, 0x10 | wheel, NULL, 8, 20);
  tick(&f, 20); empty(&f); EQ(f.m.count.bad_length, 7);
  EQ(f.m.count.rejected, 0); EQ(f.m.count.overflow, 0);
  rx_select(&f, command(1, 233, 0), 0xf, 21); tick(&f, 21);
  one(&f, 233, MOTOR_OUT_APPLIED, MOTOR_R_NONE); EQ(f.a.stop, 1);

  for (unsigned i = 0; i < sizeof(states)/sizeof(states[0]); ++i) {
    variant = 200 + states[i];
    establish(&f, states[i]); f.in.busy = 1;
    rx_select(&f, command(1, 200, 0), 0xf, 20); tick(&f, 20);
    struct motor_reply r[4]; EQ(replies(&f, r), 1);
    EQ(r[0].request_seq, 200); EQ(r[0].outcome, MOTOR_OUT_APPLIED);
    EQ(f.a.abort_busy, 1); EQ(f.a.start, 0);
    f.in.busy = 0; tick(&f, 21);
    EQ(f.a.abort_busy, 0); EQ(f.a.stop, 0); EQ(f.a.ack_fault, 0);
    EQ(f.a.set_target, 0); EQ(f.a.start, 0);
  }
  /* Hard limits are inclusive; zero-ff SPEED at both signed boundaries. */
  const int16_t bounds[] = {-2976, 2976};
  for (unsigned i = 0; i < 2; ++i) {
    establish(&f, MOTOR_LC_READY);
    struct motor_control c = command(0, 11, f.m.session); c.setpoint = bounds[i];
    rx(&f, c, 20); tick(&f, 20);
    one(&f, 11, MOTOR_OUT_INHIBITED, MOTOR_R_STARTING);
    EQ(f.a.start, 1); EQ(f.a.setpoint, bounds[i]);
  }

  // Decided (B): B's READY motion row includes zero, but a zero target needs no
  // bridge (HOLD follows motion, it never starts it). Assume accepted/no start,
  // including unaligned and busy: zero cannot cause motion.
  establish(&f, MOTOR_LC_READY); f.in.aligned = 0; f.in.busy = 1;
  struct motor_control zero = command(0, 11, f.m.session); zero.setpoint = 0;
  rx(&f, zero, 20); tick(&f, 20);
  one(&f, 11, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  EQ(f.m.lifecycle, MOTOR_LC_READY); EQ(f.a.start, 0); EQ(f.m.applied_seq, 11);
}

static void reply_content(void)
{
  struct fixture f;
  establish(&f, MOTOR_LC_RUNNING);
  f.in.rpm = -118; f.in.iq_ma = -1275;
  f.in.edges_neg = 1;
  rx(&f, command(0, 12, f.m.session), 20); tick(&f, 20);
  struct motor_reply r = one(&f, 12, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  EQ(r.lifecycle, MOTOR_LC_RUNNING); EQ(r.applied_seq, 12);
  EQ(r.first_edge, 0); EQ(r.rpm, -118); EQ(r.iq_ma, -1300);
  f.in.edges_pos = 1;
  rx(&f, command(0, 13, f.m.session), 21); tick(&f, 21);
  r = one(&f, 13, MOTOR_OUT_APPLIED, MOTOR_R_NONE); EQ(r.first_edge, 1);
  struct motor_control reverse = command(0, 14, f.m.session); reverse.setpoint = -120;
  rx(&f, reverse, 22); tick(&f, 22);
  r = one(&f, 14, MOTOR_OUT_APPLIED, MOTOR_R_NONE); EQ(r.first_edge, 0);
  ++f.in.edges_neg;
  reverse.seq = 15; rx(&f, reverse, 23); tick(&f, 23);
  r = one(&f, 15, MOTOR_OUT_APPLIED, MOTOR_R_NONE); EQ(r.first_edge, 1);
  const int16_t iq[] = {24, 25, 74, 75, -24, -25, -74, -75, 32767, -32768};
  const int16_t rounded[] = {0, 50, 50, 100, 0, -50, -50, -100, 6350, -6350};
  for (unsigned i = 0; i < sizeof(iq)/sizeof(iq[0]); ++i) {
    variant = i; f.in.iq_ma = iq[i]; reverse.seq++;
    rx(&f, reverse, 24 + i); tick(&f, 24 + i);
    r = one(&f, reverse.seq, MOTOR_OUT_APPLIED, MOTOR_R_NONE); EQ(r.iq_ma, rounded[i]);
  }
  rx(&f, command(1, 250, 0), 40); tick(&f, 40);
  r = one(&f, 250, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  EQ(r.lifecycle, MOTOR_LC_READY); EQ(r.rpm, 0); EQ(r.iq_ma, 0); EQ(r.first_edge, 0);
  /* ff rides with the setpoint to the speed loop. */
  establish(&f, MOTOR_LC_RUNNING);
  struct motor_control c = command(0, 12, f.m.session); c.ff_ma = 6000;
  rx(&f, c, 20); tick(&f, 20);
  r = one(&f, 12, MOTOR_OUT_APPLIED, MOTOR_R_NONE); EQ(r.applied_seq, 12);
  EQ(f.a.set_target, 1); EQ(f.a.ff_ma, 6000);

  for (unsigned i = 0; i < sizeof(states)/sizeof(states[0]); ++i) {
    if (states[i] == MOTOR_LC_STARTING || states[i] == MOTOR_LC_RUNNING) continue;
    variant = 100 + states[i]; establish(&f, states[i]);
    f.in.rpm = 2345; f.in.iq_ma = 32767;
    rx(&f, command(1, 99, 0), 20); tick(&f, 20);
    struct motor_reply rr[4]; EQ(replies(&f, rr), 1);
    EQ(rr[0].rpm, 0); EQ(rr[0].iq_ma, 0); EQ(rr[0].first_edge, 0);
  }
  establish(&f, MOTOR_LC_READY);
  f.in.edges_pos = UINT16_MAX;
  rx(&f, command(0, 11, f.m.session), 20); tick(&f, 20);
  one(&f, 11, MOTOR_OUT_INHIBITED, MOTOR_R_STARTING);
  f.pending_mc = -1; f.in.mc = MOTOR_CMD_MC_RUN; f.in.edges_pos = 0;
  rx(&f, command(0, 12, f.m.session), 21); tick(&f, 21);
  r = one(&f, 12, MOTOR_OUT_APPLIED, MOTOR_R_NONE); EQ(r.first_edge, 1);
}

static void robustness(void)
{
  struct fixture f;
  for (unsigned si = 0; si < sizeof(states)/sizeof(states[0]); ++si) {
    variant = states[si]; establish(&f, states[si]);
    uint8_t old = f.m.session;
    for (unsigned i = 0; i < MOTOR_CMD_INBOX + 1; ++i)
      rx(&f, command(2, (uint8_t)(20 + i), old), 20);
    tick(&f, 20);
    EQ(f.m.lifecycle, states[si] == MOTOR_LC_FAULTED ? MOTOR_LC_FAULTED : MOTOR_LC_DISARMED);
    // B: an overflow rotates the session even when FAULTED stays FAULTED.
    EXPECT(f.m.session != old); EQ(f.m.applied_seq, 0);
    EQ(f.m.count.overflow, 1); EQ(f.a.start, 0);
    // Decided (B): overflow cannot satisfy one reply per received frame. B does
    // not define the discarded inbox's replies; require fail-closed/no restart.
    struct motor_reply r[4]; replies(&f, r);
    tick(&f, 21); EQ(f.m.count.overflow, 1); EQ(f.a.start, 0);
    if (states[si] != MOTOR_LC_FAULTED) {
      rx(&f, command(2, 100, f.m.session), 22); tick(&f, 22);
      one(&f, 100, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
      rx(&f, command(0, 101, old), 23); tick(&f, 23);
      one(&f, 101, MOTOR_OUT_REJECTED, MOTOR_R_BAD_SESSION); EQ(f.a.start, 0);
    }
  }
  /* Exact capacity: eight valid frames fit; four oldest replies survive. */
  establish(&f, MOTOR_LC_READY);
  for (unsigned j = 0; j < 8; ++j) rx(&f, command(3, (uint8_t)(40 + j), 0), 20);
  tick(&f, 20); EQ(f.m.count.overflow, 0); EQ(f.m.count.replies_dropped, 4);
  struct motor_reply r[4]; EQ(replies(&f, r), 4);
  for (unsigned j = 0; j < 4; ++j) { EQ(r[j].request_seq, 40 + j); EQ(r[j].outcome, MOTOR_OUT_APPLIED); }
  rx(&f, command(3, 60, 0), 21); tick(&f, 21);
  one(&f, 60, MOTOR_OUT_APPLIED, MOTOR_R_NONE); EQ(f.m.count.replies_dropped, 4);
  /* Filled across ticks: a dropped reply must not suppress the command. */
  establish(&f, MOTOR_LC_RUNNING);
  for (unsigned j = 0; j < 6; ++j) {
    rx(&f, command(0, (uint8_t)(12 + j), f.m.session), 20 + j); tick(&f, 20 + j);
  }
  EQ(f.m.applied_seq, 17); EQ(f.m.count.replies_dropped, 2);
  EQ(replies(&f, r), 4);
  for (unsigned j = 0; j < 4; ++j) EQ(r[j].request_seq, 12 + j);
  /* Rejection accounting survives a full reply ring as well. */
  for (unsigned j = 0; j < 6; ++j) {
    struct motor_control c = command(0, 18, (uint8_t)(f.m.session + 1));
    rx(&f, c, 30 + j); tick(&f, 30 + j);
  }
  EQ(f.m.count.rejected, 6); EQ(f.m.count.last_reject, MOTOR_R_BAD_SESSION);
  EQ(f.m.count.replies_dropped, 4); EQ(replies(&f, r), 4);
  /* Index rollover through many enqueue/drain cycles. */
  establish(&f, MOTOR_LC_READY);
  for (unsigned j = 0; j < 600; ++j) {
    rx(&f, command(3, (uint8_t)j, 0), 20); tick(&f, 20);
    one(&f, (uint8_t)j, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
  }
  EQ(f.m.count.overflow, 0); EQ(f.m.count.replies_dropped, 0);

  /* A full inbox of rejected frames is still an overflow, not a motion slot. */
  establish(&f, MOTOR_LC_RUNNING);
  uint8_t bad[8] = {0x21, 0, 0, 0, 0, 0, 1, f.m.session};
  for (unsigned j = 0; j < 9; ++j)
    motor_cmd_rx(&f.m, 0x10 | wheel, bad, 8, 20);
  tick(&f, 20); EQ(f.m.count.overflow, 1);
  EQ(f.m.lifecycle, MOTOR_LC_DISARMED); EQ(f.a.stop, 1); EQ(f.a.start, 0);
  replies(&f, r);
  /* Each subsequent overflow tick is a new disarm and changes the challenge. */
  uint8_t old = f.m.session;
  for (unsigned j = 0; j < 9; ++j)
    motor_cmd_rx(&f.m, 0x10 | wheel, bad, 8, 21);
  tick(&f, 21); EQ(f.m.count.overflow, 2); EXPECT(f.m.session != old);
  EQ(f.m.lifecycle, MOTOR_LC_DISARMED); replies(&f, r);

  /* A queued frame may guess the new session: overflow must still end disarmed.
   * All these frames arrived before the overflow was handled. */
  establish(&f, MOTOR_LC_RUNNING);
  variant = 300;
  old = f.m.session;
  uint8_t predicted = (uint8_t)(1U + ((unsigned)old + 20U % 254U) % 255U);
  rx(&f, command(2, 20, predicted), 20);
  rx(&f, command(0, 21, predicted), 20);
  for (unsigned j = 2; j < 9; ++j) rx(&f, command(3, (uint8_t)(20 + j), 0), 20);
  tick(&f, 20);
  // B: frames drained in the tick of a session change cannot use the new session,
  // so a burst that guesses it still ends DISARMED with no target.
  EQ(f.m.count.overflow, 1); EQ(f.m.lifecycle, MOTOR_LC_DISARMED);
  EQ(f.a.start, 0); EQ(f.a.set_target, 0); EQ(f.m.applied_seq, 0);
  replies(&f, r);
  tick(&f, 21);
  EQ(f.m.lifecycle, MOTOR_LC_DISARMED); EQ(f.a.start, 0);
}

static void sessions(void)
{
  struct fixture f;
  const uint32_t entropy[] = {0, 1, 254, 255, 256, UINT32_MAX};
  for (unsigned i = 0; i < sizeof(entropy)/sizeof(entropy[0]); ++i) {
    variant = i; init(&f, entropy[i]);
    EQ(f.m.session, 1 + entropy[i] % 255);
  }
  init(&f, 0);
  for (unsigned j = 0; j < 600; ++j) {
    uint8_t old = f.m.session;
    uint32_t at = j * 101U;
    rx(&f, command(2, 1, old), at); tick(&f, at);
    one(&f, 1, MOTOR_OUT_APPLIED, MOTOR_R_NONE);
    tick(&f, at + 100);
    EQ(f.m.lifecycle, MOTOR_LC_DISARMED);
    EXPECT(f.m.session != old);
    EQ(f.m.session, 1U + ((unsigned)old + (at + 100) % 254U) % 255U);
    EQ(f.m.applied_seq, 0); EQ(f.a.start, 0);
  }
}

struct test_case { const char *name; void (*run)(void); };
int main(int argc, char **argv)
{
  setvbuf(stdout, NULL, _IOLBF, 0);
  const struct test_case cases[] = {
    {"transitions", transitions}, {"ordering", ordering}, {"aged_lifecycle", aged_lifecycle},
    {"ordering_rejects", ordering_rejects}, {"watchdog", watchdog},
    {"sequences", sequences}, {"faults", faults}, {"stage2_monitors", stage2_monitors},
    {"validation", validation}, {"reply_content", reply_content},
    {"robustness", robustness}, {"sessions", sessions}
  };
  unsigned ran = 0;
  if (argc > 2) { fprintf(stderr, "usage: %s [case]\n", argv[0]); return 2; }
  for (unsigned i = 0; i < sizeof(cases)/sizeof(cases[0]); ++i) {
    if (argc == 2 && strcmp(argv[1], cases[i].name)) continue;
    case_name = cases[i].name;
    unsigned before = failures;
    for (wheel = 1; wheel <= 8; wheel <<= 1) { variant = 0; cases[i].run(); }
    printf("%s: %s (%u failures)\n", case_name, before == failures ? "PASS" : "FAIL", failures - before);
    ++ran;
  }
  if (!ran) { fprintf(stderr, "unknown case: %s\n", argv[1]); return 2; }
  printf("%u cases, %u checks, %u failures (all four wheels)\n", ran, checks, failures);
  CHECK_EQ(failures, 0);
  return 0;
}
