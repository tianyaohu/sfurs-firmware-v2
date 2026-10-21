#include "ik_drive.h"
#include "can_datatypes.h"
#include "friction_ff.h"
#include "motor_protocol.h"
#include "rover_kinematics.h"
#include <math.h>
#include <stdatomic.h>
#include <string.h>

_Static_assert(sizeof(struct ik_drive_link) == IK_DRIVE_LINK_WORDS * 4U, "link row ABI");
_Static_assert(IK_DRIVE_PERIOD_MS == MOTOR_PROTOCOL_PERIOD_MS, "the ESC interpolates over this period");

volatile const uint32_t ik_drive_abi = IK_DRIVE_ABI;
volatile int32_t ik_drive_vx_mmps, ik_drive_vy_mmps, ik_drive_omega_mradps;
volatile uint32_t ik_drive_enable, ik_drive_heartbeat;
volatile int32_t ik_drive_accel_rpmps;
volatile uint32_t ik_drive_raw_id, ik_drive_raw_data, ik_drive_raw_data_hi, ik_drive_raw_len, ik_drive_raw_seq;
volatile uint32_t ik_drive_clear_seq, ik_drive_clear_done;
volatile uint32_t ik_drive_ff_enable;
volatile int32_t ik_drive_ff_coulomb_xy_mn, ik_drive_ff_viscous_xy_mn_per_mps;
volatile int32_t ik_drive_ff_coulomb_w_mnm, ik_drive_ff_viscous_w_mnm_per_radps;
volatile int32_t ik_drive_ff_a_per_nm_milli;
volatile int32_t ik_drive_ff_scale_pct[4] = {100, 100, 100, 100};
volatile uint32_t ik_drive_state, ik_drive_raw_done, ik_drive_fault, ik_drive_armed;
volatile int32_t ik_drive_target_rpm[4], ik_drive_rpm[4], ik_drive_ff_ma[4];
volatile struct ik_drive_link ik_drive_link[4];
volatile uint32_t ik_drive_tx_count, ik_drive_tx_fail, ik_drive_bad_input;

static const uint32_t SELECT[4] = {MOTOR_NE_CAN_ID, MOTOR_SE_CAN_ID, MOTOR_SW_CAN_ID, MOTOR_NW_CAN_ID};

/* Per-wheel sender state; main-loop context only, like ik_drive_rx. */
static struct {
  uint8_t seq;           /* sequence of the last motion frame (an ARM reuses it as the baseline) */
  uint8_t sent_seq;      /* seq of the frame sent this period */
  uint8_t sent, replied; /* a frame was due this period (queued or not) / its reply came back */
  uint8_t rearm;         /* the ESC rejected our sequence: re-baseline with ARM */
  int8_t sign;           /* sign of the last setpoint sent, to see a start */
  uint8_t start_seq, start_pending; /* the frame whose reply times the current start */
} wheel[4];

static uint32_t last_heartbeat, heartbeat_tick, period_tick, hold_tick, stop_frames;
static int heartbeat_seen;
static float command[4];

static int send_frame(ik_send_fn send, uint32_t id, const uint8_t *data, uint32_t len)
{
  if (send(id, data, len)) { ++ik_drive_tx_count; return 1; }
  ++ik_drive_tx_fail;
  return 0;
}

static int send_control(ik_send_fn send, const struct motor_control *c, uint32_t select)
{
  uint32_t id;
  uint8_t data[MOTOR_PROTOCOL_DLC];
  if (motor_control_encode(c, select, &id, data) != MOTOR_R_NONE) { ++ik_drive_tx_fail; return 0; }
  return send_frame(send, id, data, MOTOR_PROTOCOL_DLC);
}

static int disable_all(ik_send_fn send)
{
  const struct motor_control c = { .mode = MOTOR_MODE_DISABLE };
  return send_control(send, &c, MOTOR_NE_CAN_ID | MOTOR_SE_CAN_ID | MOTOR_SW_CAN_ID | MOTOR_NW_CAN_ID);
}

/* Seqlock for the debugger: odd while writing, even when stable. */
static void link_open(unsigned w)
{
  ++ik_drive_link[w].generation;
  atomic_thread_fence(memory_order_seq_cst);
}

static void link_close(unsigned w)
{
  atomic_thread_fence(memory_order_seq_cst);
  ++ik_drive_link[w].generation;
}

static int armed_lifecycle(uint32_t lc)
{
  return lc == MOTOR_LC_READY || lc == MOTOR_LC_STARTING || lc == MOTOR_LC_RUNNING || lc == MOTOR_LC_HOLD;
}

int ik_drive_rx(uint32_t id, const uint8_t *data, uint32_t len, uint32_t now_us)
{
  if ((id & MOTOR_CAN_ID_BASE_MASK) != MOTOR_CAN_ID_CONTROL_REPLY) return 0;
  unsigned w = 4U;
  for (unsigned i = 0; i < 4U; ++i)
    if ((id & MOTOR_CAN_SELECT_MASK) == SELECT[i]) w = i;
  struct motor_reply r;
  if (w == 4U || motor_reply_decode(data, len, &r) != MOTOR_R_NONE) return 1; /* ours, but malformed */
  volatile struct ik_drive_link *l = &ik_drive_link[w];
  link_open(w);
  l->lifecycle = r.lifecycle;
  l->outcome = r.outcome;
  l->reason = r.reason;
  l->session = r.session;
  l->request_seq = r.request_seq;
  l->applied_seq = r.applied_seq;
  l->first_edge = r.first_edge;
  l->rpm = r.rpm;
  l->iq_ma = r.iq_ma;
  ++l->reply_count;
  l->reply_us = now_us;
  if (r.outcome == MOTOR_OUT_REJECTED) ++l->rejects;
  /* The reply to a start's first frame times when that ESC applied it (synchronization metric). */
  if (wheel[w].start_pending && r.request_seq == wheel[w].start_seq &&
      (r.outcome == MOTOR_OUT_APPLIED || r.outcome == MOTOR_OUT_INHIBITED)) {
    l->start_reply_us = now_us;
    if (l->start_count == 1U) {
      l->first_start_reply_us = now_us;
      l->first_start_valid = 1U;
    }
    wheel[w].start_pending = 0U;
  }
  link_close(w);
  if (wheel[w].sent && r.request_seq == wheel[w].sent_seq) wheel[w].replied = 1U;
  /* After a Nucleo restart the ESC may still be armed with another baseline. */
  if (r.outcome == MOTOR_OUT_REJECTED &&
      (r.reason == MOTOR_R_STALE || r.reason == MOTOR_R_SEQ_JUMP || r.reason == MOTOR_R_SEQ_AMBIGUOUS ||
       r.reason == MOTOR_R_CONFLICT || r.reason == MOTOR_R_BAD_SESSION))
    wheel[w].rearm = 1U;
  if (armed_lifecycle(r.lifecycle)) ik_drive_armed |= 1U << w;
  else ik_drive_armed &= ~(1U << w);
  return 1;
}

/* Target wheel RPM for the requested twist; zero when not driving or invalid. */
static void targets(int drive, float target[4])
{
  for (unsigned i = 0; i < 4U; ++i) target[i] = 0.0f;
  if (!drive) return;
  const int32_t vx = ik_drive_vx_mmps, vy = ik_drive_vy_mmps, omega = ik_drive_omega_mradps;
  if (vx < -IK_DRIVE_MAX_LINEAR_MMPS || vx > IK_DRIVE_MAX_LINEAR_MMPS ||
      vy < -IK_DRIVE_MAX_LINEAR_MMPS || vy > IK_DRIVE_MAX_LINEAR_MMPS ||
      omega < -IK_DRIVE_MAX_OMEGA_MRADPS || omega > IK_DRIVE_MAX_OMEGA_MRADPS ||
      !rover_ik(rover_ik_provisional, vx / 1000.0f, vy / 1000.0f, omega / 1000.0f, target, 0)) {
    ++ik_drive_bad_input;
    for (unsigned i = 0; i < 4U; ++i) target[i] = 0.0f;
  }
}

/* Friction feed-forward for the requested twist, mA per wheel. The model runs on the target
 * twist, not the slewed wheel speed, so the current is there from the first frame; the ESC
 * slews it and applies it only along its speed reference. */
static void feed_forward(int on, int32_t ma[4])
{
  for (unsigned i = 0; i < 4U; ++i) ma[i] = 0;
  if (!on) return;
  const struct friction_ff_params p = {
    .coulomb_xy_n = (float)ik_drive_ff_coulomb_xy_mn / 1000.0f,
    .viscous_xy_n_per_mps = (float)ik_drive_ff_viscous_xy_mn_per_mps / 1000.0f,
    .coulomb_w_nm = (float)ik_drive_ff_coulomb_w_mnm / 1000.0f,
    .viscous_w_nm_per_radps = (float)ik_drive_ff_viscous_w_mnm_per_radps / 1000.0f,
    .motor_a_per_nm = (float)ik_drive_ff_a_per_nm_milli / 1000.0f,
    .wheel_scale = {(float)ik_drive_ff_scale_pct[0] / 100.0f, (float)ik_drive_ff_scale_pct[1] / 100.0f,
                    (float)ik_drive_ff_scale_pct[2] / 100.0f, (float)ik_drive_ff_scale_pct[3] / 100.0f},
  };
  float amps[4];
  if (!friction_ff_wheel_amps(rover_ik_provisional, &p, (float)ik_drive_vx_mmps / 1000.0f,
                              (float)ik_drive_vy_mmps / 1000.0f, (float)ik_drive_omega_mradps / 1000.0f, amps))
    return; /* invalid parameters or twist (targets() counts a bad twist): no feed-forward */
  for (unsigned i = 0; i < 4U; ++i) {
    int32_t v = (int32_t)lroundf(amps[i] * 1000.0f);
    if (v > IK_DRIVE_FF_MAX_MA) v = IK_DRIVE_FF_MAX_MA;
    if (v < -IK_DRIVE_FF_MAX_MA) v = -IK_DRIVE_FF_MAX_MA;
    ma[i] = v;
  }
}

/* Move every wheel toward its target; the wheel needing the largest change
 * moves at the acceleration limit and the others proportionally, so the
 * body accelerates along the commanded twist instead of curving. */
static void slew(const float target[4])
{
  const int32_t accel = ik_drive_accel_rpmps > 0 ? ik_drive_accel_rpmps : IK_DRIVE_DEFAULT_ACCEL_RPMPS;
  const float max_step = (float)accel * (float)IK_DRIVE_PERIOD_MS / 1000.0f;
  float largest = 0.0f;
  for (unsigned i = 0; i < 4U; ++i) {
    const float delta = fabsf(target[i] - command[i]);
    if (delta > largest) largest = delta;
  }
  const float scale = largest > max_step ? max_step / largest : 1.0f;
  for (unsigned i = 0; i < 4U; ++i) {
    command[i] = scale >= 1.0f ? target[i] : command[i] + (target[i] - command[i]) * scale;
    ik_drive_target_rpm[i] = (int32_t)lroundf(target[i]);
    ik_drive_rpm[i] = (int32_t)lroundf(command[i]);
  }
}

/* While stopped: one-shot raw frames and fault clearing for the tools. */
static void stopped_services(ik_send_fn send)
{
  const uint32_t raw_seq = ik_drive_raw_seq;
  if (raw_seq != ik_drive_raw_done) {
    uint8_t data[8];
    const uint32_t lo = ik_drive_raw_data, hi = ik_drive_raw_data_hi;
    CAN_PAYLOAD_FROM_U32(data, lo);
    CAN_PAYLOAD_FROM_U32(data + 4, hi);
    if (send_frame(send, ik_drive_raw_id & 0x7FFU, data, ik_drive_raw_len == 8U ? 8U : 4U))
      ik_drive_raw_done = raw_seq;
  }
  const uint32_t clear_seq = ik_drive_clear_seq;
  if (clear_seq != ik_drive_clear_done) {
    int ok = 1;
    for (unsigned w = 0; w < 4U; ++w) { /* CLEAR_FAULT needs no session: it only leads to DISARMED */
      const struct motor_control c = { .mode = MOTOR_MODE_DISABLE, .flags = MOTOR_FLAG_CLEAR_FAULT };
      ok &= send_control(send, &c, SELECT[w]);
    }
    if (ok) ik_drive_clear_done = clear_seq;
  }
}

/* A new drive session: every link is unknown until its ESC answers a probe. */
static void begin_session(uint32_t now)
{
  memset(wheel, 0, sizeof wheel);
  for (unsigned w = 0; w < 4U; ++w) {
    link_open(w);
    ik_drive_link[w].lifecycle = IK_LINK_NONE;
    ik_drive_link[w].misses = 0U;
    ik_drive_link[w].start_count = 0U;
    ik_drive_link[w].first_start_valid = 0U;
    link_close(w);
    command[w] = 0.0f;
  }
  ik_drive_armed = 0U;
  hold_tick = now;
}

/* Count the replies to last period's frames; a wheel that keeps missing them is lost. */
static void check_replies(void)
{
  for (unsigned w = 0; w < 4U; ++w) {
    if (!wheel[w].sent) continue;
    /* The start's reply had its period to arrive; a late one must not time a later start
     * after the 8-bit sequence wraps. */
    wheel[w].start_pending = 0U;
    link_open(w);
    volatile struct ik_drive_link *l = &ik_drive_link[w];
    /* A frame that could not even be queued counts as a miss: that wheel is not commanded. */
    l->misses = wheel[w].replied ? 0U : l->misses + 1U;
    const uint32_t had = ik_drive_fault & ((1U << w) | (1U << (4U + w)));
    if (l->misses >= IK_DRIVE_LOST_PERIODS) ik_drive_fault |= 1U << (4U + w);
    if (l->lifecycle == MOTOR_LC_FAULTED) ik_drive_fault |= 1U << w;
    if (!had && (ik_drive_fault & ((1U << w) | (1U << (4U + w)))))
      l->fault_reason = l->lifecycle == MOTOR_LC_FAULTED ? l->reason : 0U; /* keep the cause */
    link_close(w);
  }
}

static void send_wheel(ik_send_fn send, unsigned w, int16_t rpm, int16_t ff_ma)
{
  const uint32_t lc = ik_drive_link[w].lifecycle;
  struct motor_control c = { .session = (uint8_t)ik_drive_link[w].session };
  if (lc == IK_LINK_NONE || lc == MOTOR_LC_DISARMED || wheel[w].rearm) {
    /* Probe with session 0 (always rejected, the reply announces the session) or arm with it. */
    c.mode = MOTOR_MODE_DISABLE;
    c.flags = MOTOR_FLAG_ARM;
    c.seq = wheel[w].seq;
    if (lc == IK_LINK_NONE) c.session = 0U;
    wheel[w].rearm = 0U;
  } else {
    c.mode = MOTOR_MODE_SPEED;
    c.setpoint = rpm;
    c.ff_ma = ff_ma;
    c.seq = ++wheel[w].seq;
    const int8_t sign = (int8_t)((rpm > 0) - (rpm < 0));
    if (sign != 0 && sign != wheel[w].sign) { /* a start from zero or a reversal */
      wheel[w].start_seq = c.seq;
      wheel[w].start_pending = 1U;
      link_open(w);
      ++ik_drive_link[w].start_count;
      link_close(w);
    }
    wheel[w].sign = sign;
  }
  wheel[w].sent_seq = c.seq;
  wheel[w].replied = 0U;
  (void)send_control(send, &c, SELECT[w]);
  wheel[w].sent = 1U;
}

void ik_drive_tick(uint32_t now, ik_send_fn send)
{
  const uint32_t heartbeat = ik_drive_heartbeat;
  if (heartbeat != last_heartbeat) {
    last_heartbeat = heartbeat;
    heartbeat_tick = now;
    heartbeat_seen = 1;
  }
  if (ik_drive_enable == 0U) ik_drive_fault = 0U; /* the host has seen it */
  const int fresh = heartbeat_seen && (uint32_t)(now - heartbeat_tick) < IK_DRIVE_HEARTBEAT_MS;
  const int drive = fresh && ik_drive_enable != 0U && ik_drive_fault == 0U;

  if (ik_drive_state == IK_STOPPED) {
    if (!drive) {
      stopped_services(send);
      return;
    }
    ik_drive_state = IK_DRIVING;
    period_tick = now - IK_DRIVE_PERIOD_MS;
    begin_session(now);
  }
  if ((uint32_t)(now - period_tick) < IK_DRIVE_PERIOD_MS) return;
  period_tick = now;

  if (ik_drive_state == IK_DRIVING) {
    check_replies();
    if (ik_drive_fault != 0U) { /* a FAULTED or LOST wheel stops the robot now, not next period */
      ik_drive_state = IK_STOPPING;
      stop_frames = disable_all(send) ? 1U : 0U;
      for (unsigned w = 0; w < 4U; ++w) wheel[w].sent = 0U;
      return;
    }
  }
  if (ik_drive_state == IK_STOPPING) {
    if (drive) {
      ik_drive_state = IK_DRIVING;
    } else {
      if (disable_all(send) && ++stop_frames >= IK_DRIVE_STOP_FRAMES) ik_drive_state = IK_STOPPED;
      for (unsigned w = 0; w < 4U; ++w) wheel[w].sent = 0U;
      return;
    }
  }

  /* Motion only once every wheel is armed, so all four start from the same frame. */
  const int go = drive && ik_drive_armed == 0xFU;
  float target[4];
  targets(go, target);
  slew(target);
  int32_t ff_ma[4];
  feed_forward(go && ik_drive_ff_enable != 0U, ff_ma);
  int moving = drive;
  for (unsigned w = 0; w < 4U; ++w) {
    ik_drive_ff_ma[w] = ff_ma[w];
    send_wheel(send, w, (int16_t)ik_drive_rpm[w], (int16_t)ff_ma[w]);
    moving |= ik_drive_rpm[w] != 0;
  }
  if (moving) {
    hold_tick = now;
  } else if ((uint32_t)(now - hold_tick) >= IK_DRIVE_HOLD_MS) {
    ik_drive_state = IK_STOPPING;
    stop_frames = 0U;
  }
}
