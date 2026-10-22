#define _POSIX_C_SOURCE 200809L
#include "esc_ctl.h"
#include "gen_esc_defaults.h"
#include "motor_protocol.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

static double now_s(void)
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (double)t.tv_sec + (double)t.tv_nsec * 1e-9;
}

static void pause_poll(void) { nanosleep(&(struct timespec){0, 2000000}, NULL); }

void esc_ctl_init(esc_ctl_t *c, ik_link_t *l, double timeout_s)
{
  *c = (esc_ctl_t){.link = l, .timeout_s = timeout_s};
}

static int send_control(esc_ctl_t *c, uint32_t select, const struct motor_control *control)
{
  uint32_t id;
  uint8_t data[MOTOR_PROTOCOL_DLC];
  enum motor_reason reason = motor_control_encode(control, select, &id, data);
  if (reason != MOTOR_R_NONE) {
    snprintf(c->link->err, sizeof c->link->err, "CONTROL: %s", ik_reason_name(reason));
    return -1;
  }
  return ik_send_raw8(c->link, id, data, c->timeout_s);
}

int esc_ctl_disable_all(esc_ctl_t *c)
{
  for (int w = 0; w < IK_WHEELS; w++) c->wheel[w].armed = false;
  return send_control(c, MOTOR_CAN_SELECT_MASK, &(struct motor_control){.mode = MOTOR_MODE_DISABLE});
}

static int wheel_error(esc_ctl_t *c, int w, const char *operation, const ik_link_row_t *r)
{
  snprintf(c->link->err, sizeof c->link->err, "%s %s: %s %s %s (session %u)", IK_WHEEL_POS[w], operation,
           r->valid ? r->lifecycle_name : "unreadable link", r->valid ? r->outcome_name : "",
           r->valid ? r->reason_name : "", r->valid ? r->session : 0);
  return -1;
}

static int wait_reply(esc_ctl_t *c, int w, uint32_t before, uint8_t seq, ik_link_row_t *reply)
{
  double deadline = now_s() + c->timeout_s;
  for (;;) {
    if (ik_link_row(c->link, w, reply)) return -1;
    if (reply->valid && reply->reply_count != before && reply->request_seq == seq) return 0;
    if (now_s() >= deadline) return wheel_error(c, w, "CONTROL reply timed out", reply);
    pause_poll();
  }
}

int esc_ctl_arm(esc_ctl_t *c, int w)
{
  if (w < 0 || w >= IK_WHEELS) {
    snprintf(c->link->err, sizeof c->link->err, "wheel must be 0..3");
    return -1;
  }
  esc_ctl_wheel_t *s = &c->wheel[w];
  s->armed = false;
  ik_link_row_t before_row, reply;
  if (ik_link_row(c->link, w, &before_row)) return -1;
  if (!before_row.valid) return wheel_error(c, w, "ARM", &before_row);
  struct motor_control control = {.mode = MOTOR_MODE_DISABLE, .flags = MOTOR_FLAG_ARM, .seq = ++s->seq};
  if (send_control(c, IK_WHEEL_SELECT[w], &control) ||
      wait_reply(c, w, before_row.reply_count, control.seq, &reply)) return -1;
  if (reply.outcome != MOTOR_OUT_REJECTED || reply.reason != MOTOR_R_BAD_SESSION || !reply.session)
    return wheel_error(c, w, "ARM probe rejected", &reply);
  s->session = (uint8_t)reply.session;
  control.session = s->session;
  control.seq = ++s->seq;
  uint32_t before = reply.reply_count;
  if (send_control(c, IK_WHEEL_SELECT[w], &control) || wait_reply(c, w, before, control.seq, &reply)) return -1;
  if (reply.outcome != MOTOR_OUT_APPLIED || reply.session != s->session ||
      reply.lifecycle < MOTOR_LC_READY || reply.lifecycle > MOTOR_LC_HOLD)
    return wheel_error(c, w, "ARM failed", &reply);
  s->armed = true;
  return 0;
}

int esc_ctl_speed(esc_ctl_t *c, int w, int rpm)
{
  if (w < 0 || w >= IK_WHEELS || !c->wheel[w].armed || rpm < -ESC_MAX_APPLICATION_SPEED_RPM ||
      rpm > ESC_MAX_APPLICATION_SPEED_RPM) {
    snprintf(c->link->err, sizeof c->link->err, "SPEED requires an armed wheel (0..3) and |rpm| <= %d",
             ESC_MAX_APPLICATION_SPEED_RPM);
    return -1;
  }
  esc_ctl_wheel_t *s = &c->wheel[w];
  struct motor_control control = {.mode = MOTOR_MODE_SPEED, .setpoint = (int16_t)rpm,
    .seq = ++s->seq, .session = s->session};
  return send_control(c, IK_WHEEL_SELECT[w], &control);
}

int esc_ctl_clear_faults(esc_ctl_t *c)
{
  ik_link_t *l = c->link;
  uint32_t state, seq, done;
  ik_link_row_t before[IK_WHEELS], rows[IK_WHEELS];
  if (ik_read(l, IK_STATE, &state)) return -1;
  if (state != 0) {
    snprintf(l->err, sizeof l->err, "drive not stopped; CLEAR_FAULT refused");
    return -1;
  }
  /* Acknowledge the Nucleo's stop latch as well as clearing the ESC causes. */
  if (ik_write(l, IK_ENABLE, 0) || ik_link_rows(l, before) || ik_read(l, IK_CLEAR_SEQ, &seq)) return -1;
  for (int w = 0; w < IK_WHEELS; w++) c->wheel[w].armed = false;
  if (ik_write(l, IK_CLEAR_SEQ, ++seq)) return -1;
  double deadline = now_s() + c->timeout_s;
  for (;;) {
    if (ik_read(l, IK_CLEAR_DONE, &done)) return -1;
    if (done == seq) break;
    if (now_s() >= deadline) {
      snprintf(l->err, sizeof l->err, "CLEAR_FAULT: clear_done timed out");
      return -1;
    }
    pause_poll();
  }
  deadline = now_s() + c->timeout_s;
  for (;;) {
    if (ik_link_rows(l, rows)) return -1;
    bool ok = true;
    for (int w = 0; w < IK_WHEELS; w++)
      if (!before[w].valid || !rows[w].valid || rows[w].reply_count == before[w].reply_count ||
          rows[w].lifecycle > MOTOR_LC_HOLD) ok = false;
    if (ok) return 0;
    if (now_s() >= deadline) {
      snprintf(l->err, sizeof l->err, "CLEAR_FAULT timed out:");
      for (int w = 0; w < IK_WHEELS; w++) {
        size_t used = strlen(l->err);
        snprintf(l->err + used, sizeof l->err - used, " %s %s/%s%s;", IK_WHEEL_POS[w],
                 rows[w].valid ? rows[w].lifecycle_name : "unreadable",
                 rows[w].valid ? rows[w].reason_name : "UNKNOWN",
                 rows[w].valid && rows[w].reply_count != before[w].reply_count ? "" : " (no fresh reply)");
      }
      return -1;
    }
    pause_poll();
  }
}

int esc_ctl_exchange(esc_ctl_t *c, uint32_t id, const uint8_t data[8], ik_link_row_t replies[IK_WHEELS])
{
  ik_link_row_t before[IK_WHEELS];
  for (int w = 0; w < IK_WHEELS; w++) if (id & IK_WHEEL_SELECT[w]) {
    if (ik_link_row(c->link, w, &before[w])) return -1;
    if (!before[w].valid) return wheel_error(c, w, "reply baseline", &before[w]);
  }
  if (ik_send_raw8(c->link, id, data, c->timeout_s)) return -1;
  for (int w = 0; w < IK_WHEELS; w++)
    if ((id & IK_WHEEL_SELECT[w]) && wait_reply(c, w, before[w].reply_count, data[6], &replies[w])) return -1;
  return 0;
}

int esc_ctl_disable(esc_ctl_t *c, int w)
{
  uint32_t id;
  uint8_t data[8];
  ik_link_row_t replies[IK_WHEELS];
  if (w < 0 || w >= IK_WHEELS) return -1;
  struct motor_control control = {.mode = MOTOR_MODE_DISABLE, .seq = ++c->wheel[w].seq};
  if (motor_control_encode(&control, IK_WHEEL_SELECT[w], &id, data) != MOTOR_R_NONE) return -1;
  c->wheel[w].armed = false;
  return esc_ctl_exchange(c, id, data, replies);
}

/* Invalid structural frames start with codec bytes, then corrupt only the field under test. */
enum reject_kind { BAD_VERSION, BAD_MODE, BAD_FLAGS, RESERVED, BAD_SELECT, BAD_SESSION, NOT_ARMED,
                   RANGE_RPM, RANGE_FF, STALE, AMBIGUOUS, JUMP, DUPLICATE, CONFLICT, UNSUPPORTED };
static int reject_frame(enum reject_kind kind, int w, uint8_t seq, uint8_t session, uint32_t *id, uint8_t data[8])
{
  if (w < 0 || w >= IK_WHEELS) return -1;
  struct motor_control c = {.mode = MOTOR_MODE_SPEED, .seq = (uint8_t)(seq + 1U), .session = session};
  switch (kind) {
    case BAD_SESSION: c.mode = MOTOR_MODE_DISABLE; c.flags = MOTOR_FLAG_ARM; c.session = 0; break;
    case RANGE_RPM: c.setpoint = ESC_MAX_APPLICATION_SPEED_RPM + 1; break;
    case RANGE_FF: c.ff_ma = ESC_IQMAX_A * 1000 + 1; break;
    case STALE: c.seq = (uint8_t)(seq - 1U); break;
    case AMBIGUOUS: c.seq = (uint8_t)(seq - 128U); break;
    case JUMP: c.seq = (uint8_t)(seq + 17U); break;
    case DUPLICATE: c.seq = seq; break;
    /* Range checks follow sequence checks: even broken conflict handling cannot start motion. */
    case CONFLICT: c.seq = seq; c.setpoint = ESC_MAX_APPLICATION_SPEED_RPM + 1; break;
    case UNSUPPORTED: c.mode = MOTOR_MODE_CURRENT; break;
    default: break;
  }
  if (motor_control_encode(&c, IK_WHEEL_SELECT[w], id, data) != MOTOR_R_NONE) return -1;
  switch (kind) {
    case BAD_VERSION: data[0] = 0x21; break;
    case BAD_MODE: data[0] = 0x13; break;
    case BAD_FLAGS: data[1] = MOTOR_FLAG_ARM; break;
    case RESERVED: data[1] = 4; break;
    case BAD_SELECT: *id |= IK_WHEEL_SELECT[(w + 1) % IK_WHEELS]; break;
    default: break;
  }
  return 0;
}
#define BUILDER(name, kind) \
  static int name(int w, uint8_t seq, uint8_t session, uint32_t *id, uint8_t data[8]) \
  { return reject_frame(kind, w, seq, session, id, data); }
BUILDER(bad_version, BAD_VERSION)
BUILDER(bad_mode, BAD_MODE)
BUILDER(bad_flags, BAD_FLAGS)
BUILDER(reserved, RESERVED)
BUILDER(bad_select, BAD_SELECT)
BUILDER(bad_session, BAD_SESSION)
BUILDER(not_armed, NOT_ARMED)
BUILDER(range_rpm, RANGE_RPM)
BUILDER(range_ff, RANGE_FF)
BUILDER(stale, STALE)
BUILDER(ambiguous, AMBIGUOUS)
BUILDER(jump, JUMP)
BUILDER(duplicate, DUPLICATE)
BUILDER(conflict, CONFLICT)
BUILDER(unsupported, UNSUPPORTED)
#undef BUILDER

const esc_reject_case_t *esc_ctl_reject_cases(size_t *count)
{
  static const esc_reject_case_t cases[] = {
    {"BAD_VERSION", bad_version, MOTOR_OUT_REJECTED, MOTOR_R_BAD_VERSION, false},
    {"BAD_MODE", bad_mode, MOTOR_OUT_REJECTED, MOTOR_R_BAD_MODE, false},
    {"BAD_FLAGS", bad_flags, MOTOR_OUT_REJECTED, MOTOR_R_BAD_FLAGS, false},
    {"RESERVED_NONZERO", reserved, MOTOR_OUT_REJECTED, MOTOR_R_RESERVED_NONZERO, false},
    {"BAD_SELECT", bad_select, MOTOR_OUT_REJECTED, MOTOR_R_BAD_SELECT, false},
    {"BAD_SESSION", bad_session, MOTOR_OUT_REJECTED, MOTOR_R_BAD_SESSION, false},
    {"NOT_ARMED", not_armed, MOTOR_OUT_REJECTED, MOTOR_R_NOT_ARMED, true},
    {"OUT_OF_RANGE_RPM", range_rpm, MOTOR_OUT_REJECTED, MOTOR_R_OUT_OF_RANGE, false},
    {"OUT_OF_RANGE_FF", range_ff, MOTOR_OUT_REJECTED, MOTOR_R_OUT_OF_RANGE, false},
    {"STALE", stale, MOTOR_OUT_REJECTED, MOTOR_R_STALE, false},
    {"SEQ_AMBIGUOUS", ambiguous, MOTOR_OUT_REJECTED, MOTOR_R_SEQ_AMBIGUOUS, false},
    {"SEQ_JUMP", jump, MOTOR_OUT_REJECTED, MOTOR_R_SEQ_JUMP, false},
    {"DUPLICATE", duplicate, MOTOR_OUT_DUPLICATE, MOTOR_R_NONE, false},
    {"CONFLICT", conflict, MOTOR_OUT_REJECTED, MOTOR_R_CONFLICT, false},
    {"UNSUPPORTED", unsupported, MOTOR_OUT_REJECTED, MOTOR_R_UNSUPPORTED, false},
  };
  *count = sizeof cases / sizeof *cases;
  return cases;
}

static uint32_t latency_edge(unsigned bin) { return bin == IK_LAT_BINS - 1 ? UINT32_MAX : (bin + 1U) * 250U; }

esc_latency_t esc_ctl_latency(const uint64_t bins[IK_LAT_BINS])
{
  esc_latency_t s = {0};
  for (unsigned b = 0; b < IK_LAT_BINS; b++) {
    s.count += bins[b];
    if (bins[b]) s.max_us = latency_edge(b);
  }
  if (!s.count) return s;
  uint64_t cumulative = 0;
  const uint64_t p50 = s.count / 2 + s.count % 2;
  const uint64_t p99 = s.count - s.count / 100;
  for (unsigned b = 0; b < IK_LAT_BINS; b++) {
    cumulative += bins[b];
    if (!s.p50_us && cumulative >= p50) s.p50_us = latency_edge(b);
    if (!s.p99_us && cumulative >= p99) s.p99_us = latency_edge(b);
  }
  return s;
}

double esc_ctl_bus_load(uint64_t frames8, uint64_t frames4, double seconds)
{
  return seconds > 0 ? (130.0 * (double)frames8 + 100.0 * (double)frames4) / (seconds * 5000.0) : 0;
}

int esc_ctl_stream_tx(uint32_t tx, uint32_t *per_wheel)
{
  if (tx < 3 || (tx - 3U) % IK_WHEELS) return -1;
  *per_wheel = (tx - 3U) / IK_WHEELS + 3U;
  return 0;
}
